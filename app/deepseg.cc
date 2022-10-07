/* This is licenced software, @see LICENSE file.
 * Authors - @see AUTHORS file.
==============================================================================*/

#include <unistd.h>
#include <cstdio>
#include <chrono>
#include <string>
#include <thread>
#include <mutex>
#include <fstream>
#include <istream>
#include <regex>
#include <optional>
#include <utility>
#include <condition_variable>
#include <csignal>
#include <cstdarg>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "videoio/loopback.h"
#include "lib/libbackscrub.h"
#include "background.h"

// Temporary declaration of utility class until we merge experimental!
class on_scope_exit final {
private:
	const std::function<void()> dtor;
public:
	explicit inline on_scope_exit(const std::function<void()>& f) : dtor(f) {}
	on_scope_exit() = delete;
	on_scope_exit(const on_scope_exit&) = delete;
	inline ~on_scope_exit() {
		if(dtor) {
			dtor();
		}
	}
};

// Due to weirdness in the C(++) preprocessor, we have to nest stringizing macros to ensure expansion
// http://gcc.gnu.org/onlinedocs/cpp/Stringizing.html, use _STR(<raw text or macro>).
#define __STR(X) #X
#define _STR(X) __STR(X)

// Ensure we have a default search location for resource files
#ifndef INSTALL_PREFIX
#error No INSTALL_PREFIX defined at compile time
#endif

#define DEBUG_WIN_NAME "Backscrub " _STR(DEEPSEG_VERSION) " ('?' for help)"

int fourCcFromString(const std::string& in) {
	if (in.empty())
		return 0;

	if (in.size() <= 4)
	{
		// fourcc codes are up to 4 bytes long, right-space-padded and upper-case
		// c.f. http://ffmpeg.org/doxygen/trunk/isom_8c-source.html and
		// c.f. https://www.fourcc.org/codecs.php
		std::array<uint8_t, 4> a = {' ', ' ', ' ', ' '};
		for (size_t i = 0; i < in.size(); ++i)
			a[i] = ::toupper(in[i]);
		return cv::VideoWriter::fourcc(a[0], a[1], a[2], a[3]);
	}
	else if (in.size() == 8)
	{
		// Most people seem to agree on 0x47504A4D being the fourcc code of "MJPG", not the literal translation
		// 0x4D4A5047. This is also what ffmpeg expects.
		return std::stoi(in, nullptr, 16);
	}
	return 0;
}

// Parse a geometry specification
std::optional<std::pair<int, int>> geometryFromString(const std::string& in) {
	int w, h;
	if (sscanf(in.c_str(), "%dx%d", &w, &h)!=2) {
		return {};
	}
	return std::pair<int, int>(w, h);
}

// OpenCV helper functions
cv::Mat convert_rgb_to_yuyv(cv::Mat input) {
	cv::Mat tmp;
	cv::cvtColor(input, tmp, cv::COLOR_RGB2YUV);
	std::vector<cv::Mat> yuv;
	cv::split(tmp, yuv);
	cv::Mat yuyv(tmp.rows, tmp.cols, CV_8UC2);
	uint8_t* outdata = (uint8_t*)yuyv.data;
	uint8_t* ydata = (uint8_t*)yuv[0].data;
	uint8_t* udata = (uint8_t*)yuv[1].data;
	uint8_t* vdata = (uint8_t*)yuv[2].data;
	for (unsigned int i = 0; i < yuyv.total(); i += 2) {
		uint8_t u = (uint8_t)(((int)udata[i]+(int)udata[i+1])/2);
		uint8_t v = (uint8_t)(((int)vdata[i]+(int)vdata[i+1])/2);
		outdata[2*i+0] = ydata[i+0];
		outdata[2*i+1] = v;
		outdata[2*i+2] = ydata[i+1];
		outdata[2*i+3] = u;
	}
	return yuyv;
}

cv::Mat alpha_blend(cv::Mat srca, cv::Mat srcb, cv::Mat mask) {
	// alpha blend two (8UC3) source images using a mask (8UC1, 255=>srca, 0=>srcb), adapted from:
	// https://www.learnopencv.com/alpha-blending-using-opencv-cpp-python/
	// "trust no-one" => we're about to mess with data pointers
	assert(srca.rows == srcb.rows);
	assert(srca.cols == srcb.cols);
	assert(mask.rows == srca.rows);
	assert(mask.cols == srca.cols);
	assert(srca.type() == CV_8UC3);
	assert(srcb.type() == CV_8UC3);
	assert(mask.type() == CV_8UC1);
	cv::Mat out = cv::Mat::zeros(srca.size(), srca.type());
	uint8_t *optr = (uint8_t*)out.data;
	uint8_t *aptr = (uint8_t*)srca.data;
	uint8_t *bptr = (uint8_t*)srcb.data;
	uint8_t *mptr = (uint8_t*)mask.data;
	int npix = srca.rows * srca.cols;
	for (int pix = 0; pix < npix; ++pix) {
		// blending weights
		int aw = (int)(*mptr++), bw = 255-aw;
		// blend each channel byte
		*optr++ = (uint8_t)(( (int)(*aptr++)*aw + (int)(*bptr++)*bw )/255);
		*optr++ = (uint8_t)(( (int)(*aptr++)*aw + (int)(*bptr++)*bw )/255);
		*optr++ = (uint8_t)(( (int)(*aptr++)*aw + (int)(*bptr++)*bw )/255);
	}
	return out;
}

// timing helpers
typedef std::chrono::high_resolution_clock::time_point timestamp_t;
typedef struct {
	timestamp_t bootns;
	timestamp_t lastns;
	timestamp_t lockns;
	timestamp_t copyns;
	timestamp_t prepns;
	timestamp_t maskns;
	timestamp_t postns;
	timestamp_t v4l2ns;
	timestamp_t grabns;
	timestamp_t retrns;
} timinginfo_t;

timestamp_t timestamp() {
	return std::chrono::high_resolution_clock::now();
}
long diffnanosecs(timestamp_t t1, timestamp_t t2) {
	return std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t2).count();
}

// encapsulation of mask calculation logic and threading
class CalcMask final {
protected:
	enum class thread_state { RUNNING, DONE };
	volatile thread_state state;
	void *maskctx;
	timestamp_t t0;
	// buffers
	cv::Mat mask1;
	cv::Mat mask2;
	cv::Mat *mask_current;
	cv::Mat *mask_out;
	cv::Mat frame1;
	cv::Mat frame2;
	cv::Mat *frame_current;
	cv::Mat *frame_next;
	// thread synchronisation
	std::mutex lock_frame;
	std::mutex lock_mask;
	std::condition_variable condition_new_frame;
	bool new_frame;
	bool new_mask;
	std::thread thread;

	void run() {
		cv::Mat *raw_tmp;
		timestamp_t tloop;

		while(thread_state::RUNNING == this->state) {
			tloop = t0 = timestamp();
			/* actual handling */
			{
				std::unique_lock<std::mutex> hold(lock_frame);
				while (!new_frame) {
					condition_new_frame.wait(hold);
				}
				// change frame buffer pointer
				new_frame = false;
				raw_tmp = frame_next;
				frame_next = frame_current;
				frame_current = raw_tmp;
			}
			waitns = diffnanosecs(timestamp(), t0);
			t0 = timestamp();
			if(!bs_maskgen_process(maskctx, *frame_current, *mask_current)) {
				fprintf(stderr, "failed to process video frame\n");
				exit(1);
			}
			std::unique_lock<std::mutex> hold(lock_mask);
			raw_tmp = mask_out;
			mask_out = mask_current;
			mask_current = raw_tmp;
			new_mask = true;
			loopns = diffnanosecs(timestamp(), tloop);
		}
	}

	// timing callbacks
	static void onprep(void *ctx) {
		CalcMask *cls = (CalcMask *)ctx;
		cls->prepns = diffnanosecs(timestamp(), cls->t0);
		cls->t0 = timestamp();
	}
	static void oninfer(void *ctx) {
		CalcMask *cls = (CalcMask *)ctx;
		cls->tfltns = diffnanosecs(timestamp(), cls->t0);
		cls->t0 = timestamp();
	}
	static void onmask(void *ctx) {
		CalcMask *cls = (CalcMask *)ctx;
		cls->maskns = diffnanosecs(timestamp(), cls->t0);
		cls->t0 = timestamp();
	}

public:
	long waitns;
	long prepns;
	long tfltns;
	long maskns;
	long loopns;

	CalcMask(const std::string& modelname,
	         int threads,
	         int width,
	         int height,
	         bool debug) {
		maskctx = bs_maskgen_new(modelname.c_str(), threads, width, height, debug, nullptr, onprep, oninfer, onmask, this);
		if (!maskctx) {
			throw "Could not create mask context";
		}

		// Do all other initialization …
		frame_next = &frame1;
		frame_current = &frame2;
		mask_current = &mask1;
		mask_out = &mask2;
		new_frame = false;
		new_mask = false;
		state = thread_state::RUNNING;
		thread = std::thread(&CalcMask::run, this);
	}

	~CalcMask() {
		// mark as done
		state = thread_state::DONE;
		// wake up processing thread
		new_frame = true;
		condition_new_frame.notify_all();
		// collect termination
		thread.join();
		bs_maskgen_delete(maskctx);
	}

	void set_input_frame(cv::Mat &frame) {
		std::lock_guard<std::mutex> hold(lock_frame);
		*frame_next = frame.clone();
		new_frame = true;
		condition_new_frame.notify_all();
	}

	void get_output_mask(cv::Mat &out) {
		if (new_mask) {
			std::lock_guard<std::mutex> hold(lock_mask);
			out = mask_out->clone();
			new_mask = false;
		}
	}
};

static bool is_number(const std::string &s) {
	return !s.empty() && std::all_of(s.begin(), s.end(), ::isdigit);
}

std::optional<std::string> resolve_path(const std::string& provided, const std::string& type) {
	std::string result;
	// Check for network (URI) schema and return as-is
	// https://www.rfc-editor.org/rfc/rfc3986#section-3.1
	// however we require at least two chars in the scheme to allow driver letters to work on Windows..
	if (std::regex_match(provided, std::regex("^[[:alpha:]][[:alnum:]+-.]{1,}:.*$"))) {
		return provided;
	}
	// We use std::ifstream to check we can open each test path read-only, in order:
	// 1. exactly what was provided
	if (std::ifstream(provided).good()) {
		return provided;
	}
	// to emulate PATH search behaviour (rule of least surprise), we stop here if provided has path separators
	if (provided.find('/') != provided.npos) {
		return {};
	}
	// 2. BACKSCRUB_PATH prefixes if set
	if (getenv("BACKSCRUB_PATH") != nullptr) {
		// getline trick: https://stackoverflow.com/questions/5167625/splitting-a-c-stdstring-using-tokens-e-g
		std::istringstream bsp(getenv("BACKSCRUB_PATH"));
		while (std::getline(bsp, result, ':')) {
			result += "/" + type + "/" + provided;
			if (std::ifstream(result).good()) {
				return result;
			}
		}
	}
	// 3. XDG standard data location
	result = getenv("XDG_DATA_HOME") ? getenv("XDG_DATA_HOME") : std::string() + getenv("HOME") + "/.local/share";
	result += "/backscrub/" + type + "/" + provided;
	if (std::ifstream(result).good())
		return result;
	// 4. prefixed with compile-time install path
	result = std::string() + _STR(INSTALL_PREFIX) + "/share/backscrub/" + type + "/" + provided;
	if (std::ifstream(result).good()) {
		return result;
	}
	// 5. relative to current binary location
	// (https://stackoverflow.com/questions/933850/how-do-i-find-the-location-of-the-executable-in-c)
	char binloc[1024];
	ssize_t n = readlink("/proc/self/exe", binloc, sizeof(binloc));
	if (n > 0) {
		binloc[n] = 0;
		result = binloc;
		size_t pos = result.rfind('/');
		pos = result.rfind('/', pos-1);
		if (pos != result.npos) {
			result.erase(pos);
			result += "/share/backscrub/" + type + "/" + provided;
			if (std::ifstream(result).good()) {
				return result;
			}
			// development folder?
			result.erase(pos);
			result += "/" + type + "/" + provided;
			if (std::ifstream(result).good()) {
				return result;
			}
		}
	}
	return {};
}

// Helper for command line parsing and output of the related messages
static void printVersion(const char *name, FILE *out) {
	fprintf(out, "%s version %s\n  (Tensorflow: build %s, run-time %s)\n", name, _STR(DEEPSEG_VERSION), _STR(TF_VERSION), bs_tensorflow_version());
	fprintf(out, "  (OpenCV: version %s)\n", CV_VERSION);
	fprintf(out, "(c) 2021 by floe@butterbrot.org & contributors\n");
	fprintf(out, "https://github.com/floe/backscrub\n");
}

static void usage(const char *name, int exitCode, bool syntaxOnly, const char *message) {
	FILE *out = stderr;
	if (exitCode == 0) {
		out = stdout;
	}
	if (!syntaxOnly) {
		printVersion(name, out);
	}
	fprintf(out, "\n");
	fprintf(out, "usage:\n");
	fprintf(out, "  backscrub [-?] [-d] [-p] <-c CAPTURE_DEVICE> <-v VIRTUAL_DEVICE>\n");
	fprintf(out, "    [--cg WIDTHxHEIGHT] [--vg WIDTHxHEIGHT] [-t THREADS] [-m MODEL]\n");
	fprintf(out, "    [-b BACKGROUND] [-p FILTER:VALUE] [-mf FPS] [-rf RATE] [-vp]\n");
	if (!syntaxOnly) {
		fprintf(out, "\n");
		fprintf(out, "-?|[-]-help\n");
		fprintf(out, "       Display this usage information\n");
		fprintf(out, "--version\n");
		fprintf(out, "        Print version and exit\n");
		fprintf(out, "-d\n");
		fprintf(out, "        Increase debug level\n");
		fprintf(out, "-dt|--debug-time\n");
		fprintf(out, "        Dispay timing informations\n");
		fprintf(out, "-s|..show-progress\n");
		fprintf(out, "        Show progress bar\n");
		fprintf(out, "-c| --camera <Camera Device>\n");
		fprintf(out, "        Specify the video capture (source) device\n");
		fprintf(out, "-v|--virtiual <Virtual Device>\n");
		fprintf(out, "        Specify the virtual camera (sink) device\n");
		fprintf(out, "-w|--width <WIDTH>\n");
		fprintf(out, "        DEPRECATED: Specify the video stream width\n");
		fprintf(out, "-h|--height <HEIGHT>\n");
		fprintf(out, "        DEPRECATED: Specify the video stream height\n");
		fprintf(out, "--cg|-cg|--camera-geometry <WIDTHxHEIGHT>\n");
		fprintf(out, "        Specify the capture device geometry as WIDTHxHEIGHT\n");
		fprintf(out, "--vg|-vg|virtual-geometry <WIDTHxHEIGHT>\n");
		fprintf(out, ";       Specify the virtual camera geometry as WIDTHxHEIGHT\n");
		fprintf(out, "-f|--format <Format>\n");
		fprintf(out, "        Specify the camera video format, i.e. MJPG or 47504A4D.\n");
		fprintf(out, "-t|--thread <Number of Threads>\n");
		fprintf(out, "        Specify the number of threads used for processing\n");
		fprintf(out, "-b|..background <Background>\n");
		fprintf(out, "        Specify the background (any local or network OpenCV source\n");
		fprintf(out, "          e.g. local:   backgrounds/total_landscaping.jpg\n");
		fprintf(out, "          network: https://git.io/JE9o5\n");
		fprintf(out, "-m|--model <Model>\n");
		fprintf(out, "        Specify the TFLite model used for segmentation\n");
		fprintf(out, "-p|--post-processing\n");
		fprintf(out, "       Add post-processing steps\n");
		fprintf(out, "-p|--post-processing <bgblur:STRENGTH>\n");
		fprintf(out, "        Blur the video background\n");
		fprintf(out, "-H\n");
		fprintf(out, "        Mirror the output horizontally\n");
		fprintf(out, "-V\n");
		fprintf(out, "        Mirror the output vertically\n");
		fprintf(out, "-mf|--max-fps <FPS>\n");
		fprintf(out, "        Limit the camera frame rate (may be iusefull while using a HDMI grabber)\n");
		fprintf(out, "-vd|--video-delayed\n");
		fprintf(out, "        Normaly the image send is build with the mask of the previous frame\n");
		fprintf(out, "        If this is set the previous frame is chosed. This may be disturbing\n");
		fprintf(out, "        ih the frame rate or the CPU is too slow.\n");
	}
	if (message) {
		fprintf(out,"\n%s\n", message);
	}
	exit(exitCode);
}

static void usage(const char *name, int exitcode, bool syntaxOnly) {
	usage(name, exitcode, syntaxOnly, nullptr);
}

static void usage(const char *name, int exitcode) {
	usage(name, exitcode, false, nullptr);
}

static char *message(const char *format, ...) {
	static char buf[81];
	std::va_list args;
	va_start(args, format);	
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	return buf;
}


inline bool _hasValidArgument(const char *name, int arg, int argc, char **argv) {
	if (arg+1 >= argc) {
		usage(name, 1, true, message("Option %s require a value", argv[arg]));
	}
	if ( *argv[arg+1] == '-') {
		usage(name, 1, true, message("Option %s require a value found: %s", argv[arg],argv[arg+1]));
	}
	return true;
}
#define hasValidArgument _hasValidArgument(name, arg, argc, argv)

inline bool cmp(const char *in, const char *opt1) {
	return strcmp(in, opt1) == 0;
}

inline bool cmp(const char *in, const char *opt1, const char *opt2) {
	return strcmp(in, opt1) == 0 ||
	       strcmp(in, opt2) == 0;
}

inline bool cmp(const char *in, const char *opt1, const char *opt2, const char *opt3) {
	return strcmp(in, opt1) == 0 ||
	       strcmp(in, opt2) == 0 ||
	       strcmp(in, opt3) == 0;
}

int main(int argc, char* argv[]) try {
	timinginfo_t ti;
	ti.bootns = timestamp();
	int debug = 0;
	bool showProgress = false;
	bool showBackground = true;
	bool showMask = true;
	bool showFPS = true;
	bool showHelp = false;
	int threads = 2;
	int width = 640;
	int height = 480;
	bool setWorH = false;
	std::optional<std::pair<int, int>> capGeo = {};
	std::optional<std::pair<int, int>> vidGeo = {};
	const char *back = nullptr;
	const char *vcam = nullptr;
	const char *ccam = nullptr;
	bool flipHorizontal = false;
	bool flipVertical = false;
	int fourcc = 0;
	int blur_strength = 0;
	cv::Rect crop_region(0, 0, 0, 0);
	int maxFps = 0;
	int fps = 0;
	int fpsDivisor = 0;
	bool debugTiming = false;
	bool sendPrevious = false;

	const char* modelname = "selfiesegmentation_mlkit-256x256-2021_01_19-v1215.f16.tflite";
	char * name = strrchr(argv[0], '/');
	if (name) {
		name++;
	 } else {
		name = argv[0];
	}
    if (argc == 1) {
		usage(name, 0, false);
	}

	for (int arg = 1; arg < argc; arg++) {
		if (cmp(argv[arg], "-?", "-help", "--help")) {
			usage(name, 0);
		} else if (cmp(argv[arg], "-d")) {
			++debug;
		} else if (cmp(argv[arg], "-s")) {
			showProgress = true;
		} else if (cmp(argv[arg], "-H")) {
			flipHorizontal = !flipHorizontal;
		} else if (cmp(argv[arg], "-V")) {
			flipVertical = !flipVertical;
		} else if (cmp(argv[arg], "-v")) {
			if (hasValidArgument) {
				vcam = argv[++arg];
			}
		} else if (cmp(argv[arg], "-c")) {
			if (hasValidArgument) {
				ccam = argv[++arg];
			}
		} else if (cmp(argv[arg], "-b")) {
			if (hasValidArgument) {
				back = argv[++arg];
			}
		} else if (cmp(argv[arg], "-m")) {
			if (hasValidArgument) {
				modelname = argv[++arg];
			}
		} else if (cmp(argv[arg], "-p")) {
			if (hasValidArgument) {
				std::string option = argv[++arg];
				std::string key = option.substr(0, option.find(":"));
				std::string value = option.substr(option.find(":")+1);
				if (key == "bgblur") {
					if (is_number(value)) {
						blur_strength = std::stoi(value);
						if (blur_strength % 2 == 0) {
							usage(name, 1, true, "strength value must be odd" );
						}
					} else {
						printf("No strength value supplied, using default strength 25\n");
						blur_strength = 25;
					}
				} else {
					usage(name, 1, false, message("Unknown post-processing option: %s", option.c_str()));
				}
			}
		// deprecated width/height switches (implicitly capture and virtual camera size)
		} else if (cmp(argv[arg], "-w")) {
			if (hasValidArgument && sscanf(argv[++arg], "%d", &width)) {
				if (!width) {
					usage(name, 1, true, message("Option %s require a valid value", argv[arg-1]));
				}
				// if the width is odd we will have color error!
				width = width + width % 2;
				setWorH = true;
			}
		} else if (cmp(argv[arg], "-h")) {
			if (hasValidArgument && sscanf(argv[++arg], "%d", &height)) {
				if (!height) {
					usage(name, 1, true, message("Option %s require a valid value", argv[arg-1]));
				}
				setWorH = true;
			}
		// replacement geometry switches (separate capture and virtual camera size)
		} else if (cmp(argv[arg], "--cg", "-cg", "--camera-geometry")) {
			if (hasValidArgument) {
				capGeo = geometryFromString(argv[++arg]);
				if (capGeo->first < 1 || capGeo->second < 1) {
					usage(name, 1, true, message("%s wrong geometry %s", argv[arg-1], argv[arg]));
				}				
			}
		} else if (cmp(argv[arg], "--vg", "-vg", "--virtual-geometry")) {
			if (hasValidArgument) {
				vidGeo = geometryFromString(argv[++arg]);
				if (vidGeo->first < 1 || vidGeo->second < 1) {
					usage(name, 1, true, message("%s wrong geometry %s", argv[arg-1], argv[arg]));
				}
				// if the width is odd we will have color error!
				vidGeo->first += vidGeo->first % 2;
			}
		} else if (cmp(argv[arg], "-f")) {
			if (hasValidArgument) {
				fourcc = fourCcFromString(argv[++arg]);
				if (!fourcc) {
					usage(name, 1, false, "Option -f require a valid value");
				}
			}
		} else if (cmp(argv[arg], "-t")) {
			if (hasValidArgument && sscanf(argv[++arg], "%d", &threads)) {
				if (!threads) {
					usage(name, 1, true, message("Option %s require a valid value", argv[arg-1]));
				}
			}
		// for hdmi grabber with to heigh frame rate
		} else if (cmp(argv[arg], "--max-fps", "-mf")) {
			if (hasValidArgument && sscanf(argv[++arg], "%d", &maxFps)) {
				if (maxFps <= 0) {
					usage(name, 1, true, message("Option %s require a valid value", argv[arg-1]));
				}
			}
		// proposal
		} else if (cmp(argv[arg], "-dd")) {
			debug = 2;
		} else if (cmp(argv[arg], "--version")) {
			printVersion(name, stdout);
			exit(0);
		} else if (cmp(argv[arg], "--debug-timing", "-dt")) {
			debugTiming = true;
		} else if (cmp(argv[arg], "--video-delayed", "-vd")) {
			sendPrevious = true;
		// end of parser
		} else {
			usage(name, 1, true, message("Unknown option: %s", argv[arg]));
		}
	}

	if (ccam == nullptr) {
		usage(name, 1, false, "Option -c is mandatory");
	}
	if (vcam == nullptr) {
		usage(name, 1, false, "Option -v is mandatory");
	}

	// check aspect ration 2.726:1 is thee max aspect ratio found on smartphone
	if (vidGeo) {
		if ((float)vidGeo->first/vidGeo->second > 2.726||
		    (float)vidGeo->second/vidGeo->first >  2.726) {
			usage(name, 1, true, message(
			      "Wrong --vg (--video-geometry) parameter %dx%d, aspect ratio to big",
			      vidGeo->first, vidGeo->second));
		}
	}

	// prevent use of both deprecated and current switches
	if (setWorH && (capGeo || vidGeo)) {
		usage(name, 1, true, "Error: (DEPRECATED) -w/-h used in conjunction with --cg/--vg.");
	}
	// set capture device geometry from deprecated switches if not set already
	if (!capGeo) {
		capGeo = std::pair<int, int>(width, height);
	}

	std::string s_ccam(ccam);
	std::string s_vcam(vcam);
	// permit unprefixed device names
	if (s_ccam.rfind("/dev/", 0) != 0) {
		s_ccam = "/dev/" + s_ccam;
	}
	if (s_vcam.rfind("/dev/", 0) != 0) {
		s_vcam = "/dev/" + s_vcam;
	}
	std::optional<std::string> s_model = resolve_path(modelname, "models");
	std::optional<std::string> s_backg = back ? resolve_path(back, "backgrounds") : std::nullopt;
	// open capture early to resolve true geometry
	cv::VideoCapture cap(s_ccam.c_str(), cv::CAP_V4L2);
	if(!cap.isOpened()) {
		perror("failed to open capture device");
		exit(1);
	}
	// set fourcc (if specified) /before/ attempting to set geometry (@see issue146)
	if (fourcc) {
		cap.set(cv::CAP_PROP_FOURCC, fourcc);
	}
	cap.set(cv::CAP_PROP_FRAME_WIDTH,  capGeo->first);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, capGeo->second);
	cap.set(cv::CAP_PROP_CONVERT_RGB, true);
	std::optional<std::pair<int, int>> tmpGeo = std::pair<int, int>(
	    (int)cap.get(cv::CAP_PROP_FRAME_WIDTH),
	    (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)
	);

	fps = (int)cap.get(cv::CAP_PROP_FPS);
	fpsDivisor = 1;
	if (maxFps > 0) {
		fpsDivisor = ((fps+maxFps-1) / maxFps);
	}
	if (tmpGeo != capGeo) {
		fprintf(stdout, "Warning: capture device geometry changed from requested values.\n");
		capGeo = tmpGeo;
	}
	if (!vidGeo) {
		vidGeo = capGeo;
	}
	// aspect ratio changed? warn
	// NB: we calculate this way round to avoid comparing doubles..
	int expWidth = (int)((double)vidGeo->second * (double)capGeo->first/(double)capGeo->second);
	if (expWidth != vidGeo->first) {
		fprintf(stdout, "Warning: virtual camera aspect ratio does not match capture device.\n");
	}

	if (capGeo != vidGeo) {
		crop_region = bs_calc_cropping(
		               capGeo->first, capGeo->second,
		               vidGeo->first, vidGeo->second);
	}

	if (debug) {
		// dump settings..
		printVersion(name, stderr);
		fprintf(stderr, "debug:   %d\n", debug);
		fprintf(stderr, "ccam:    %s\n", s_ccam.c_str());
		fprintf(stderr, "vcam:    %s\n", s_vcam.c_str());
		fprintf(stderr, "capGeo:  %dx%d\n", capGeo->first, capGeo->second);
		fprintf(stderr, "cam Fps: %d (max: %d, real: %.2g)\n",fps, maxFps?maxFps:fps, (float)fps/fpsDivisor);
		fprintf(stderr, "vidGeo:  %dx%d\n", vidGeo->first, vidGeo->second);
		fprintf(stderr, "flip_h:  %s\n", flipHorizontal ? "yes" : "no");
		fprintf(stderr, "flip_v:  %s\n", flipVertical ? "yes" : "no");
		fprintf(stderr, "threads: %d\n", threads);
		fprintf(stderr, "back:    %s => %s\n", back ? back : "(none)", s_backg ? s_backg->c_str() : "(none)");
		fprintf(stderr, "model:   %s => %s\n\n", modelname ? modelname : "(none)", s_model ? s_model->c_str() : "(none)");
	}

	// No model - stop here
	if (!s_model) {
		fprintf(stderr, "Error: unable to load specified model: %s\n", modelname);
		exit(1);
	}

	// Create debug window early (ensures highgui is correctly initialised on this thread)
	if (debug > 1) {
		cv::namedWindow(DEBUG_WIN_NAME, cv::WINDOW_AUTOSIZE | cv::WINDOW_GUI_EXPANDED);
	}

	// Load background if specified
	auto pbk(s_backg ? load_background(s_backg.value(), debug) : nullptr);
	if (!pbk) {
		if (s_backg) {
			printf("Warning: could not load background image, defaulting to green\n");
		}
	}
	// default green screen background (at video true geometry)
	cv::Mat bg(vidGeo->second, vidGeo->first, CV_8UC3, cv::Scalar(0, 255, 0));

	// Virtual camera (at specified geometry)
	int lbfd = loopback_init(s_vcam, vidGeo->first, vidGeo->second, debug);
	if(lbfd < 0) {
		fprintf(stderr, "Failed to initialize vcam device.\n");
		exit(1);
	}

	on_scope_exit lbfd_closer([lbfd]() {
		loopback_free(lbfd);
	});

	// Processing components, at virtual true geometry
	cv::Mat mask(vidGeo->second, vidGeo->first, CV_8U);

	cv::Mat raw[2]; // 2 buffer in order to avoid copy
	int idx = 0;    // for selection of buffer
	CalcMask ai(*s_model, threads, vidGeo->first, vidGeo->second, debug);

	ti.lastns = timestamp();
	if (debug) {
		fprintf(stderr, "Startup: %ldns\n", diffnanosecs(ti.lastns,ti.bootns));
	}

	bool filterActive = true;
	cv::Mat current, previous = {};
	// mainloop
	int skip = fpsDivisor;
	while(true) {
		// grab new frame from cam
		cap.grab();
		ti.grabns = timestamp();
		// copy new frame to buffer
		cap.retrieve(raw[idx]);
		ti.retrns = timestamp();

		if (raw[idx].rows == 0 || raw[idx].cols == 0) {
			continue; // sanity check
		}
		if (skip < fpsDivisor) {
			skip++;
			continue;
		} else {
			skip = 1;
		}

		if (crop_region.x || crop_region.y) {
			raw[idx](crop_region).copyTo(raw[idx]);
		}
		if ( raw[idx].cols != vidGeo->first || raw[idx].rows != vidGeo->second) {
			cv:resize(raw[idx], raw[idx], cv::Size(vidGeo->first,vidGeo->second));
		}

		if (sendPrevious) {
			if (!raw[(idx+1)&1].cols) {
				raw[(idx+1)&1] = raw[idx]; // only for the first video frame!
			}
		}

		ai.set_input_frame(raw[idx]);
		ti.copyns = timestamp();

		if(sendPrevious) {
			idx = (idx + 1) & 1;
		}

		// do background detection magic
		ai.get_output_mask(mask);
		ti.copyns = timestamp();

		if (filterActive) {
			// get background frame:
			// - specified source if set
			// - copy of input video if blur_strength != 0
			// - default green (initial value)
			bool canBlur = false;
			if (pbk) {
				if (grab_background(pbk, vidGeo->first, vidGeo->second, bg) < 0) {
					throw "Failed to read background frame";
				}
				canBlur = true;
			} else if (blur_strength) {
				raw[idx].copyTo(bg);
				canBlur = true;
			}
			// blur frame if requested (unless it's just green)
			if (canBlur && blur_strength) {
				cv::GaussianBlur(bg,bg,cv::Size(blur_strength,blur_strength), 0);
			}
			ti.prepns = timestamp();
			// alpha blend background over foreground using mask
			raw[idx] = alpha_blend(bg, raw[idx], mask);
		} else {
			ti.prepns = timestamp();
		}
		ti.maskns = timestamp();

		if (flipHorizontal && flipVertical) {
			cv::flip(raw[idx], raw[idx], -1);
		} else if (flipHorizontal) {
			cv::flip(raw[idx], raw[idx], 1);
		} else if (flipVertical) {
			cv::flip(raw[idx], raw[idx], 0);
		}
		ti.postns = timestamp();

		// write frame to v4l2loopback as YUYV
		raw[idx] = convert_rgb_to_yuyv(raw[idx]);
		int framesize = raw[idx].step[0]*raw[idx].rows;
		while (framesize > 0) {
			int ret = write(lbfd,raw[idx].data,framesize);
			if(ret <= 0) {
				perror("writing to loopback device");
				exit(1);
			}
			framesize -= ret;
		}
		ti.v4l2ns=timestamp();

		if (!debug && !debugTiming) {
			if (showProgress) {
				printf(".");
				fflush(stdout);
			}
			continue;
		}

		// timing details..
		double mfps = 1e9/diffnanosecs(ti.v4l2ns,ti.lastns);
		double afps = 1e9/ai.loopns;
		if (debugTiming) {
			printf("main [grab:%7.4f retr:%7.4f copy:%7.4f prep:%7.4f mask:%7.4f post:%7.4f v4l2:%7.4f tot:%7.4f FPS: %5.2f] ai: [wait:%7.4f prep:%7.4f tflt:%7.4f mask:%7.4f tot:%7.4f FPS: %5.2f] \e[K\r",
				diffnanosecs(ti.grabns,ti.lastns)/1000000.0,
				diffnanosecs(ti.retrns,ti.grabns)/1000000.0,
				diffnanosecs(ti.copyns,ti.retrns)/1000000.0,
				diffnanosecs(ti.prepns,ti.copyns)/1000000.0,
				diffnanosecs(ti.maskns,ti.prepns)/1000000.0,
				diffnanosecs(ti.postns,ti.maskns)/1000000.0,
				diffnanosecs(ti.v4l2ns,ti.postns)/1000000.0,
				diffnanosecs(ti.v4l2ns,ti.grabns)/1000000.0,
				mfps,
				ai.waitns/1000000.0,
				ai.prepns/1000000.0,
				ai.tfltns/1000000.0,
				ai.maskns/1000000.0,
				(ai.prepns+ai.tfltns+ai.maskns)/1000000.0,
				afps
			);
			fflush(stdout);
		}
		ti.lastns = timestamp();
		if (debug < 2) {
			continue;
		}

		cv::Mat test;
		cv::cvtColor(raw[idx],test,cv::COLOR_YUV2BGR_YUYV);
		// frame rates & sizes at the bottom
		if (showFPS) {
			char status[80];
			snprintf(status, sizeof(status), "MainFPS: %5.2f AiFPS: %5.2f (%dx%d->%dx%d)",
			         mfps, afps, capGeo->first, capGeo->second, vidGeo->first, vidGeo->second);
			cv::putText(test, status, cv::Point(5, test.rows-5), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 255));
		}
		// keyboard help
		if (showHelp) {
			static const std::string help[] = {
			    "Keyboard help:",
			     " q: quit",
			     " s: switch filter on/off",
			     " h: toggle horizontal flip",
			     " v: toggle vertical flip",
			     " f: toggle FPS display on/off",
			     " b: toggle background display on/off",
			     " m: toggle mask display on/off",
			     " ?: toggle this help text on/off"
			};
			for (int i=0; i<sizeof(help)/sizeof(std::string); i++) {
				cv::putText(test, help[i], cv::Point(10,test.rows/2+i*15), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255));
			}
		}
		// background as pic-in-pic
		if (showBackground && pbk) {
			cv::Mat thumb;
			grab_thumbnail(pbk, thumb);
			if (!thumb.empty()) {
				int h = thumb.rows*160/thumb.cols;
				if ((h < raw[idx].rows*3/4 || thumb.cols < raw[idx].cols/2) && h > 50) {
					cv::Rect cr = bs_calc_cropping(thumb.cols, thumb.rows, 160, h);
					thumb(cr).copyTo(thumb);
					cv::Rect r = cv::Rect(0, 0, thumb.cols, thumb.rows);
					cv::Mat tri = test(r);
					thumb.copyTo(tri);
					cv::rectangle(test, r, cv::Scalar(255,255,255));
				}
			}
		}
		// mask as pic-in-pic
		if (showMask) {
			if (!mask.empty()) {
				cv::Mat smask, cmask;
				int mheight = mask.rows*160/mask.cols;
				if ( mheight < raw[idx].rows*3/4 || mask.cols < raw[idx].cols/2) {
					cv::resize(mask, smask, cv::Size(160, mheight));
					cv::cvtColor(smask, cmask, cv::COLOR_GRAY2BGR);
					cv::Rect r = cv::Rect(raw[idx].cols-160, 0, 160,mheight);
					cv::Mat mri = test(r);
					cmask.copyTo(mri);
					cv::rectangle(test, r, cv::Scalar(255,255,255));
					cv::putText(test, "Mask", cv::Point(raw[idx].cols-155,115), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,255,255));
				}
			}
		}
		cv::imshow(DEBUG_WIN_NAME, test);

		auto keyPress = cv::waitKey(1);
		switch(keyPress) {
			case 'q':
				std::raise(SIGINT);
				break;
			case 's':
				filterActive = !filterActive;
				break;
			case 'h':
				flipHorizontal = !flipHorizontal;
				break;
			case 'v':
				flipVertical = !flipVertical;
				break;
			case 'f':
				showFPS = !showFPS;
				break;
			case 'b':
				showBackground = !showBackground;
				break;
			case 'm':
				showMask = !showMask;
				break;
			case '?':
				showHelp = !showHelp;
				break;
		}
	}

	printf("\n");
	return 0;
} catch(const char* msg) {
	fprintf(stderr, "Error: %s\n", msg);
	return 1;
}

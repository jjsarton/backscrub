/* This is licensed software, @see LICENSE file.
 * Authors - @see AUTHORS file. */

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "loopback.h"

void print_format(struct v4l2_format*vid_format) {
	fprintf(stderr, "vid_format->type                = %d\n",	vid_format->type );
	fprintf(stderr, "vid_format->fmt.pix.width       = %d\n",	vid_format->fmt.pix.width );
	fprintf(stderr, "vid_format->fmt.pix.height      = %d\n",	vid_format->fmt.pix.height );
	fprintf(stderr, "vid_format->fmt.pix.pixelformat = %d\n",	vid_format->fmt.pix.pixelformat);
	fprintf(stderr, "vid_format->fmt.pix.sizeimage   = %d\n",	vid_format->fmt.pix.sizeimage );
	fprintf(stderr, "vid_format->fmt.pix.field       = %d\n",	vid_format->fmt.pix.field );
	fprintf(stderr, "vid_format->fmt.pix.bytesperline= %d\n",	vid_format->fmt.pix.bytesperline );
	fprintf(stderr, "vid_format->fmt.pix.colorspace  = %d\n",	vid_format->fmt.pix.colorspace );
	fprintf(stderr, "\n");
}

int loopback_init(const std::string& device, int w, int h, int debug) {

	struct v4l2_capability vid_caps;
	struct v4l2_format vid_format;

	size_t linewidth = w * 2;
	size_t framesize = h * linewidth;

	int ret_code;

	int fdwr = open(device.c_str(), O_RDWR|O_CLOEXEC);
	if(fdwr < 0) {
		fprintf(stderr, "%s:%d(%s): Failed to open video device: %s\n", __FILE__, __LINE__, __func__, strerror(errno));
		return -1;
	}

	ret_code = ioctl(fdwr, VIDIOC_QUERYCAP, &vid_caps);
	if(ret_code < 0) {
		fprintf(stderr, "%s:%d(%s): Failed to query device capabilities: %s\n", __FILE__, __LINE__, __func__, strerror(errno));
		close(fdwr);
		return -1;
	}

	memset(&vid_format, 0, sizeof(vid_format));
	vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	vid_format.fmt.pix.width = w;
	vid_format.fmt.pix.height = h;
	vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	vid_format.fmt.pix.sizeimage = framesize;
	vid_format.fmt.pix.field = V4L2_FIELD_NONE;
	vid_format.fmt.pix.bytesperline = linewidth;
	vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

	ret_code = ioctl(fdwr, VIDIOC_S_FMT, &vid_format);
	if(ret_code < 0) {
		fprintf(stderr, "%s:%d(%s): Failed to set device video format: %s\n", __FILE__, __LINE__, __func__, strerror(errno));
		close(fdwr);
		return -1;
	}

	ret_code = ioctl(fdwr, VIDIOC_STREAMON, &vid_format.type);

	if(ret_code < 0) {
		fprintf(stderr, "%s:%d(%s): Failed to start streaming: %s\n", __FILE__, __LINE__, __func__, strerror(errno));
		close(fdwr);
		return -1;
	}

	if (debug)
		print_format(&vid_format);

	return fdwr;
}

int loopback_free(int fdwr) {

	const v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	int ret_code = ioctl(fdwr, VIDIOC_STREAMOFF, &type);

	if(ret_code < 0) {
		fprintf(stderr, "%s:%d(%s): Failed to stop streaming: %s\n", __FILE__, __LINE__, __func__, strerror(errno));
		return -1;
	}

	ret_code = close(fdwr);
	if(ret_code < 0) {
		fprintf(stderr, "%s:%d(%s): Failed to close video device: %s\n", __FILE__, __LINE__, __func__, strerror(errno));
		return -1;
	}

	return 0;
}

#ifdef standalone

#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

int main(int argc, char* argv[]) {

	std::string video_device("/dev/video1");

	size_t linewidth = FRAME_WIDTH  * 2;
	size_t framesize = FRAME_HEIGHT * linewidth;

	if(argc>1) {
		video_device=argv[1];
		printf("using output device: %s\n", video_device.c_str());
	}

	int fdwr = loopback_init(video_device,FRAME_WIDTH,FRAME_HEIGHT);
	if(fdwr < 0) {
		fprintf(stderr, "Failed to initialize output device %s\n", video_device.c_str());
		exit(1);
	}

	uint8_t* buffer = (uint8_t*)malloc(framesize);

	while (true) {
		write(fdwr, buffer, framesize);
		usleep(100000);
		uint64_t* front = (uint64_t*)(buffer);
		*front += 12345;
	}

	pause();

	close(fdwr);

	free(buffer);

	return 0;
}

#endif

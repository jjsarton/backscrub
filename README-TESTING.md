# Test version

## Revision 3

* Fix wrong mutex handling! 
* Added text to usage()

## Revision 2:

* Removed the test option --resize-first, the old behaviour was wrong.
  resizing to the output size is done if neccessary before processing the
  datas.
* Optimization of code.
* The coding style of the files was unified.

### Video and Audio

Actually backscrub retriewe a frame get the mask genarated from a previous
frame and send this to the video output device. This has the advantage that
the delay between sound and video not much modified. The disadvantage is
that movement will make the true background visible, this is not what we
expect.

If we trigger the mask processing as usual and then use the previously
generated mask, we will not have the effect decribed above. Since we send
an old video frame, the sound will arrive sooner as the video. This may be
annoying if the delay is to height.

According to the EBU Recommendation R37 the audio signal may not be send
more as 40 ms before the video signal. If the frame rate for the video signal
is about 33 ms, we will get a delay which may be a little bit greater, but
acceptable. With the default geometry 640x480 @ 30 FPS the delay may be
between 45 and 55 ms, so that the image and the sound may arrive a little
bit to soon, this appear not to be a problem. 

For testing purpose of backscrub the command line oprion --video-delayed
(or the shirt form -vd) was added. At this time, there are not optimization
for this part..

The timing values printed out are in nano seconds, readin of them is not
so easy. Printing them as ms with 4 digit after the decimal point is better.

## revision 1

### The major changes concern deepseg.c.

* Typos fixes from old code.
* Crash while using the the preview window (debug level 2).
* Use of size_t for some variables what not correct.
* Support for HDMI grabber (limiting of the frame rate).
* Command line parsing modified.
* Message output hopefully more user friendly.

### Added command line options:

* -help, --help
* --version
* -dt --debug-timing
* -dd: short form for -d -d
* --vg  may be: -vg --virtual-geometry
* --cg  may be: -cg --camera-geometry
* -mf --max-fps, limit the frame rate for the camera
* -rf --resize-first, actually big frame are processed as is and resized
  after the whole job is done.

Using an HDMI grabber may result to process frames with 1920x1080 @ 60Hz.
This implies a height CPU time and the results are not better as if
we resize first the input frame and then process the data.
 
--resize-first may be the normal and only case.

The visual results between post and pre resizing are similar. The major
difference is that the edges are very hard and shown the pixel nature
of the mask if we have post resizing. With pre-resizing the results
correspond mode to the known results (same input/output geometry).

### Performed tests

A lot of test was performed, some visually for the different delivered 
segmentation models and for backgrounds (with more files of different type).

The code run under Fedora 36 XFCE with the OpenCV 4.5.5 as well with
the own compiled OpenCV 4.6-dev, which included the missed support for
ffmpeg (the normal case on Fedora).

Backscrub what also compiled on a Linux Mint 20 which include OpenCV 4.2.
A short test shown that backscrub work well.



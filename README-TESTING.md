# Test version

## The major changes concern deepseg.c.

* Typos fixes from old code.
* Crash while using the the preview window (debug level 2).
* Use of size_t for some variables what not correct.
* Support for HDMI grabber (limiting of the frame rate).
* Command line parsing modified.
* Message output hopefully more user friendly.

## Added command line options:

* -help, --help
* --version
* -dt --debug-timing
* -dd: short form for -d -d
* --vg  may be: -vg --virtual-geometry
* --cg  may be: -cg --camera-geometry
* -mf --max-fps, limit the frame rate for the camera
* -rf --resize-first, actually big frame are processed as is and resized
  after the whole job is done.
  
Using an HDMIn grabber may result to process frames with 1920x1080@60Hz.
This implies a height CPU time and the results are not better as if
we resize first the input frame and then process the data.
 
--resize-first may be the normal and only case.

The visual results between post and pre resizing are similar. The major
difference is that the edges are very hard and shown the pixel nature
of the mask if we have post resizing. With pre-resizing the results
correspond mode to the known results (same input/output geometry).

## Performed tests

A lot of test was performed, some visually for the different delivered 
segmentation models and for backgrounds (with more files of different type).

The code run under Fedora 36 XFCE with the OpenCV 4.5.5 as well with
the own compiled OpenCV 4.6-dev, which included the missed support for
ffmpeg (the normal case on Fedora).

Backscrub what also compiled on a Linux Mint 20 which include OpenCV 4.2.
A short test shown that backscrub work well.


 

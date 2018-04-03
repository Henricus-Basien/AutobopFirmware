Cross Compile OpenCV for gnu-arm-linux-eabi
===========================================

[![Build Status](https://travis-ci.org/tudelft/opencv_bebop.png?branch=master)](https://travis-ci.org/tudelft/opencv_bebop) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)


 - OpenCV 3.2.0
 - For other version, look at the releases
 - On OSX, carlson-minot is automatically selected in the Makefile
 - Tested with:
   - /usr/bin/arm-linux-gnueabi-g++  (ver 4.7.3)
   - /opt/arm-2012.03/bin/arm-none-linux-gnueabi-g++  (ver 4.6.3)
   - /usr/local/carlson-minot/crosscompilers/bin/arm-none-linux-gnueabi-g++ (ver ??)
 - To use other crosscompilers, make a new bebop.toolchain.cmake.XXX and change the Makefile to call it
 - An XML document with the paparazzi link parameters is created in install/opencv.xml: **copy paste these parameters in your opencv module**

For more info: read the Makefile


To build it 100% automatic with defualt settings just use this command:
-------------------

  make



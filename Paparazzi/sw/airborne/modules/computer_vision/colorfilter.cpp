/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>



//+++++++++++++++++++++++++++++++++++++++++++
// OpenCV Test
//+++++++++++++++++++++++++++++++++++++++++++

//#include "modules/computer_vision/opencv_example.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

int opencv_example(char *img, int width, int height);
int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;
  // If you want a color image, uncomment this line
//   cvtColor(M, image, CV_YUV2BGR_Y422);
  // For a grayscale image, use this one
  cvtColor(M, image, CV_YUV2GRAY_Y422);

  // Blur it, because we can
//  blur(image, image, Size(5, 5));

  // Canny edges, only works with grayscale image
  int edgeThresh = 35;
  Canny(image, image, edgeThresh, edgeThresh * 3);

  // Convert back to YUV422, and put it in place of the original image
  grayscale_opencv_to_yuv422(image, img, width, height);
//  colorrgb_opencv_to_yuv422(image, img, width, height);

  return 0;
}

//+++++++++++++++++++++++++++++++++++++++++++
// 
//+++++++++++++++++++++++++++++++++++++++++++

#include "modules/computer_vision/lib/vision/image.h"

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result
int color_count = 0;

#include "subsystems/abi.h"


// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{


  opencv_example((char*)img->buf,img->w,img->h);


  printf("Test2...");
  // Filter
  color_count = image_yuv422_colorfilt(img, img,
                                       color_lum_min, color_lum_max,
                                       color_cb_min, color_cb_max,
                                       color_cr_min, color_cr_max
                                      );


  if (COLORFILTER_SEND_OBSTACLE) {
    if (color_count > 20)
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
    }
    else
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
    }
  }

  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}

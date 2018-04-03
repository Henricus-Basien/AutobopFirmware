/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/opencv_example.cpp"
 * @author C. De Wagter
 * A simple module showing what you can do with opencv on the bebop.
 */


#include "opencv_example.h"


int color_count;



using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

int opencv_example(char *raw_img_data, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat RAW(height, width, CV_8UC2, raw_img_data);
  Mat image;

  // Select ONE of the following to convert the RAW camera image: If you want a color image or grayscale
  cvtColor(RAW, image, CV_YUV2BGR_Y422);
  //cvtColor(M, image, CV_YUV2GRAY_Y422);




///////////////////////////////////////////////////////////////////////////////////////
// Start your code here

  Mat result;
  inRange(image, cv::Scalar(0, 0, 128), cv::Scalar(128, 128, 255), result);
  color_count = countNonZero(result);

//////////////////////////////////////////////////////////////////////////////////////




  // result will be sent to next module (e.g. video streaming)

  // Convert back to YUV422, and put it in place of the original image
  grayscale_opencv_to_yuv422(result, raw_img_data, width, height);
  //colorrgb_opencv_to_yuv422(image, raw_img_data, width, height);

  return 0;
}

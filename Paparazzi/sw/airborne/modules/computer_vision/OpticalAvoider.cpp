// Author: Henricus N. Basien
// Date:Tuesday 03.04.2018

#include "OpticalAvoider.h"

//+++++++++++++++++++++++++++++++++++++++++++
// Imports
//+++++++++++++++++++++++++++++++++++++++++++

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

#include <stdio.h>

int OpticalAvoider(char *raw_img_data, int width, int height){
  printf("Test OpenV Yeah\n");

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

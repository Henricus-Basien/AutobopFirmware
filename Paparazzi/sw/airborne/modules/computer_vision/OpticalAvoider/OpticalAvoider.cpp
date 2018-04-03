// Author: Henricus N. Basien
// Date:Tuesday 03.04.2018

#include "OpticalAvoider.h"

//****************************************************************************************
// Imports
//****************************************************************************************

//+++++++++++++++++++++++++++++++++++++++++++
// External
//+++++++++++++++++++++++++++++++++++++++++++

//-------------------------------------------
// stl
//-------------------------------------------

using namespace std;
#include <stdio.h>

//-------------------------------------------
// OpenCV
//-------------------------------------------

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "opencv_image_functions.h"

//...........................................
// Tracking
//...........................................

//#include "opencv2/core/cuda.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
//#include "opencv2/video/optflowgf.hpp"

//****************************************************************************************
// Test Function
//****************************************************************************************

Mat frame_old;
unsigned long frameNr = 0;

//unsigned long time = 0;

Mat TestRun(Mat frame);
Mat TestRun(Mat frame){

    printf("TestRun #%lu\n",frameNr);

    if (frameNr>0){

        double pyr_scale  = 0.5;
        int    levels     = 3;
        int    winsize    = 15;
        int    iterations = 3;
        int    poly_n     = 5;
        double poly_sigma = 1.2;

        int    flags      = 0; // cv::OPTFLOW_USE_INITIAL_FLOW

        Mat flow;
        printf("Running 'calcOpticalFlowFarneback'\n");
        //cv::calcOpticalFlowFarneback(frame_old, frame, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);

    }

    cv::circle(frame, cv::Point(200,100), 10, cv::Scalar(128, 0, 0));

    frame_old = frame;
    frameNr++;

    return frame;

}

//****************************************************************************************
// Main Function
//****************************************************************************************

int RunOpticalAvoider(char *raw_img_data, int width, int height){
    printf("Running OpticalAvoider (OpenCV)...\n");

    //+++++++++++++++++++++++++++++++++++++++++++
    // Get Raw Image
    //+++++++++++++++++++++++++++++++++++++++++++

    // Create a new image, using the original bebop image.
    Mat RAW(height, width, CV_8UC2, raw_img_data);
    Mat image;

    //-------------------------------------------
    // Convert ColorSpacce
    //-------------------------------------------

    // Select ONE of the following to convert the RAW camera image: If you want a color image or grayscale
    cvtColor(RAW, image, CV_YUV2BGR_Y422);
    //cvtColor(M, image, CV_YUV2GRAY_Y422);

    ///////////////////////////////////////////////////////////////////////////////////////
    // Start your code here

    Mat result;
    result = TestRun(image);
    //result = image;
    //inRange(image, cv::Scalar(0, 0, 128), cv::Scalar(128, 128, 255), result);
    //color_count = countNonZero(result);

    //////////////////////////////////////////////////////////////////////////////////////

    // result will be sent to next module (e.g. video streaming)

    //+++++++++++++++++++++++++++++++++++++++++++
    // Convert back to YUV422, and put it in place of the original image
    //+++++++++++++++++++++++++++++++++++++++++++

    //----//grayscale_opencv_to_yuv422(result, raw_img_data, width, height);

    colorrgb_opencv_to_yuv422(result, raw_img_data, width, height);

    return 0;
}

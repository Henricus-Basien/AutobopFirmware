// Author: Henricus N. Basien
// Date:Tuesday 03.04.2018

#include "OpticalAvoider.h"

//****************************************************************************************
// Imports
//****************************************************************************************

//#define UsePROFILER

//+++++++++++++++++++++++++++++++++++++++++++
// External
//+++++++++++++++++++++++++++++++++++++++++++

//-------------------------------------------
// stl
//-------------------------------------------

using namespace std;
#include <stdio.h>
#include <sys/time.h>
#include <chrono>
#include <iostream>

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
#include <opencv2/video/tracking.hpp>
//#include "opencv2/video/optflowgf.hpp"

//-------------------------------------------
// Mathematics
//-------------------------------------------

//#include <Eigen/Core>

//+++++++++++++++++++++++++++++++++++++++++++
// Internal
//+++++++++++++++++++++++++++++++++++++++++++
#ifdef UsePROFILER
    #include "Profiler.h"
#endif

//****************************************************************************************
// Settings
//****************************************************************************************

//-------------------------------------------
// Image
//-------------------------------------------

cv::Size res(90,195);//res(120,260);//res(240,520);//res(520,240);//res(200,150);//res(520,240);//res(150,200); //(200,150);

bool Rotate = false;

//-------------------------------------------
// Optical Flow
//-------------------------------------------

double pyr_scale  = 0.5;
int    levels     = 3;
int    winsize    = 15;
int    iterations = 3;
int    poly_n     = 5;
double poly_sigma = 1.2;

int    flags      = 0; // cv::OPTFLOW_USE_INITIAL_FLOW

//****************************************************************************************
// Detector
//****************************************************************************************

//+++++++++++++++++++++++++++++++++++++++++++
// Global Variables
//+++++++++++++++++++++++++++++++++++++++++++

Mat frame_new;
Mat frame_old;
unsigned long frameNr = 0;

cv::Size org_size;

//--- Time Keeping ---
unsigned long t_old = 0;

//+++++++++++++++++++++++++++++++++++++++++++
// Detector
//+++++++++++++++++++++++++++++++++++++++++++


Mat Detector(Mat frame);
Mat Detector(Mat frame){

    #ifdef UsePROFILER
        CPPP->ResetCheckPoints(true);
        CPPP->SetCheckPoint("Detector",0);
    #endif

    org_size = frame.size();
    frame_new = frame;

    //+++++++++++++++++++++++++++++++++++++++++++
    // Prepare Image
    //+++++++++++++++++++++++++++++++++++++++++++

    if (frame_new.size()!=res){ // Check correct Columns & Rows 
       printf("Converting from (%i,%i) to (%i,%i)\n",frame_new.size().height,frame_new.size().width,res.height,res.width);
       cv::resize(frame_new,frame_new, res);
    }

    //cv::circle(frame, cv::Point(200,100), 10, cv::Scalar(128, 0, 0));

    //+++++++++++++++++++++++++++++++++++++++++++
    // Track Time
    //+++++++++++++++++++++++++++++++++++++++++++
    
    unsigned long t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned long dt = t-t_old;
    t_old = t;
    float freq = 1000./float(dt);
    //t = time(NULL);

    printf("DetectorRun #%lu @t=%lu;dt=%lu,freq=%f Hz\n",frameNr,t,dt,freq);

    //+++++++++++++++++++++++++++++++++++++++++++
    // Optical Flow
    //+++++++++++++++++++++++++++++++++++++++++++

    if (frameNr>0){

        Mat frame_new_g;
        Mat frame_old_g;
        cvtColor(frame_new,frame_new_g, CV_BGR2GRAY);
        cvtColor(frame_old,frame_old_g, CV_BGR2GRAY);

        Mat flow;
        printf("Running 'calcOpticalFlowFarneback'\n");
        printf("Old Size (%i,%i|%i); New Size (%i,%i|%i)\n",frame_old.cols,frame_old.rows,frame_old.channels(),frame_new.cols,frame_new.rows,frame_new.channels());
        cv::calcOpticalFlowFarneback(frame_old_g, frame_new_g, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags);
        
        if (0){
            printf("Done Calculating OpticalFlow, showing Results\n");
            cout << "M = "<< endl << " "  << flow << endl << endl;
            // for(int i = 0; i < frame.cols; i++){
            //     for(int j = 0; j < frame.rows; j++){
            //         double val_x = frame.at<double>(i,j,0); //[frame.cols * j + i];
            //         double val_y = frame.at<double>(i,j,1); //[frame.cols * j + i + 1];
            //         printf("%i,%i: %f,%f\n",i,j,val_x,val_y);
            //     }
            // }
        }

        //-------------------------------------------
        // Get Mean
        //-------------------------------------------
        
        cv::Scalar mean = cv::mean(flow);   

        stringstream mean_ss;
        mean_ss << "AvgOF:" << " " << mean << endl;
        cout << mean_ss.str() << endl;
        //printf("Mean Flow %f",float(mean));

        if (0){
            double font_size = 0.5; //1.0;//#10.0;
            cv::putText(frame, mean_ss.str(), cv::Point(10,50), cv::FONT_HERSHEY_COMPLEX_SMALL, font_size, cv::Scalar(0,0,128));
        }

        //-------------------------------------------
        // Get Diff
        //-------------------------------------------
        
        cv::Mat diff;

        cv::absdiff(flow,mean, diff);

        if (0){
            cvtColor(diff,frame, CV_GRAY2BGR);
        }     

        printf("#Channels: flow=%i diff=%i",flow.channels(),diff.channels());

    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Addendum
    //+++++++++++++++++++++++++++++++++++++++++++

    #ifdef UsePROFILER
        CPPP->EndCheckPoints();
    #endif  

    //--- Store Old Frame --
    frame_old = frame_new;
    frameNr++;

    if (frame.size()!=org_size){ // Check correct Columns & Rows 
       printf("Converting from (%i,%i) to (%i,%i)\n",frame.size().height,frame_new.size().width,org_size.height,org_size.width);
       cv::resize(frame,frame, org_size);
    }
    

    //--- Return Frame --
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

    //-------------------------------------------
    // Rotate
    //-------------------------------------------
    
    if (Rotate){
        //cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        cv::transpose(image,image);
        cv::flip(image,image,0);
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    // Start your code here

    Mat result;
    result = Detector(image);
    //result = image;
    //inRange(image, cv::Scalar(0, 0, 128), cv::Scalar(128, 128, 255), result);
    //color_count = countNonZero(result);

    //////////////////////////////////////////////////////////////////////////////////////

    // result will be sent to next module (e.g. video streaming)

    //-------------------------------------------
    // Rotate
    //-------------------------------------------
    
    if (Rotate){
        //cv::rotate(image, image, cv::ROTATE_90_CLOCKWISE);
        cv::transpose(image,image);
        cv::flip(image,image,1);
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Convert back to YUV422, and put it in place of the original image
    //+++++++++++++++++++++++++++++++++++++++++++

    //----//grayscale_opencv_to_yuv422(result, raw_img_data, width, height);

    colorrgb_opencv_to_yuv422(result, raw_img_data, width, height);

    return 0;
}

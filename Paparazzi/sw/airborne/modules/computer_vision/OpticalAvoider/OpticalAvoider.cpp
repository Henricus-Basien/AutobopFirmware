// Author: Henricus N. Basien
// Date:Tuesday 03.04.2018

//****************************************************************************************
// Imports
//****************************************************************************************

//-------------------------------------------
// Debug
//-------------------------------------------

#define DEBUG
//#define UsePROFILER

#ifdef DEBUG
    #define TRACKTIME
    #define SHOWIMG
    #define SHOWDIR
    #define PRINTDEBUG
    #define FINDMAX
#else
    #define TRACKTIME
#endif

//-------------------------------------------
// Execution
//-------------------------------------------

#define UseAbsoluteMax
#define CropImage

//****************************************************************************************
// Imports
//****************************************************************************************

#include "OpticalAvoider.h"

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

//+++++++++++++++++++++++++++++++++++++++++++
// Image
//+++++++++++++++++++++++++++++++++++++++++++

cv::Size res(90,195);//res(120,260);//res(240,520);//res(520,240);//res(200,150);//res(520,240);//res(150,200); //(200,150);

bool Rotate = false;

//+++++++++++++++++++++++++++++++++++++++++++
// Optical Flow
//+++++++++++++++++++++++++++++++++++++++++++

double pyr_scale  = 0.5;
int    levels     = 3;
int    winsize    = 15;
int    iterations = 3;
int    poly_n     = 5;
double poly_sigma = 1.2;

int    flags      = 0; // cv::OPTFLOW_USE_INITIAL_FLOW

//+++++++++++++++++++++++++++++++++++++++++++
// Control
//+++++++++++++++++++++++++++++++++++++++++++

#ifdef CropImage
    float CropTop = 0.2; // [UnitInterval]
    float CropBot = 0.2; // [UnitInterval]
#endif

int NrColumns = 9;//5;
float K = 1.0; // [s]

//****************************************************************************************
// Detector
//****************************************************************************************

//+++++++++++++++++++++++++++++++++++++++++++
// Global Variables
//+++++++++++++++++++++++++++++++++++++++++++

Mat frame_new;
Mat frame_new_g;
Mat frame_old_g;
unsigned long frameNr = 0;

cv::Size org_size;

//--- Time Keeping ---
#ifdef TRACKTIME
    unsigned long t_old = 0;
#endif

//--- Optical Flow ---
cv::Mat diff;
#ifdef FINDMAX
    double MaxMean = 0;
#endif

//--- Optimal Directions ---
cv::Scalar OptPer = cv::Scalar(0.0,0.0);
cv::Scalar RecPer = cv::Scalar(0.0,0.0);
cv::Scalar RefAng = cv::Scalar(0.0,0.0);

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

    //-------------------------------------------
    // Crop
    //-------------------------------------------

    #ifdef CropImage
        int offset_y_top = int(CropTop*frame_new.size().width);
        int offset_y_bot = int(CropBot*frame_new.size().width);

        cv::Rect roi;
        roi.x = offset_y_bot;
        roi.y = 0;
        roi.width  = frame_new.size().width - (offset_y_bot+offset_y_top);
        roi.height = frame_new.size().height;

        /* Crop the original image to the defined ROI */
        //cv::Mat crop = frame_new(roi);
        frame_new = frame_new(roi);
    #endif

    //-------------------------------------------
    // Resize
    //-------------------------------------------

    if (frame_new.size()!=res){ // Check correct Columns & Rows 

        //--- Resize --
        #ifdef PRINTDEBUG
            printf("Converting from (%i,%i) to (%i,%i)\n",frame_new.size().height,frame_new.size().width,res.height,res.width);
        #endif
        cv::resize(frame_new,frame_new, res);
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Track Time
    //+++++++++++++++++++++++++++++++++++++++++++
    
    #ifdef TRACKTIME
        unsigned long t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        unsigned long dt = t-t_old;
        t_old = t;
        float freq = 1000./float(dt);
        //t = time(NULL);

        printf("DetectorRun #%lu @t=%lu;dt=%lu,freq=%f Hz\n",frameNr,t,dt,freq);
    #endif

    //+++++++++++++++++++++++++++++++++++++++++++
    // Optical Flow
    //+++++++++++++++++++++++++++++++++++++++++++

    cvtColor(frame_new,frame_new_g, CV_BGR2GRAY);

    if (frameNr>0){

        Mat flow;
        #ifdef PRINTDEBUG
            printf("Running 'calcOpticalFlowFarneback'\n");
            printf("Old grey Size (%i,%i|%i) | New Size (%i,%i|%i)\n",frame_old_g.cols,frame_old_g.rows,frame_old_g.channels(),frame_new.cols,frame_new.rows,frame_new.channels());
        #endif
        cv::calcOpticalFlowFarneback(frame_old_g, frame_new_g, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags); // returns matrix with flow in x and y
        
        //-------------------------------------------
        // Get Magnitude
        //-------------------------------------------

        Mat flow_split [2];
        cv::split(flow, flow_split); //splits flow x and y from eachother
        Mat flow_magnitude;
        cv::magnitude(flow_split[0],flow_split[1],flow_magnitude);

        #ifdef PRINTDEBUG
            if (0){
                printf("Done Calculating OpticalFlow, showing Magnitude\n");
                cout << "M = "<< endl << " "  << flow_magnitude << endl << endl;
            }
        #endif

        //................................
        // Normalize with dt
        //................................

        flow_magnitude*=1000.0/float(dt); // [px/s]

        //................................
        // Normalize with Resolution
        //................................

        flow_magnitude*=1.0/float(res.height);//width); // [Frame%/s]

        //-------------------------------------------
        // Get Mean
        //-------------------------------------------
        
        cv::Scalar mean = cv::mean(flow_magnitude);   

        #ifdef FINDMAX
            double Mean = sum(mean)[0];
            if (Mean>MaxMean){
                MaxMean = Mean;
            }
        #endif

        #ifdef PRINTDEBUG
            stringstream mean_ss;
            mean_ss << "AvgOF:"  << " " << mean << endl;
            #ifdef FINDMAX
                mean_ss << "MaxAvg:" << " " << MaxMean << endl;
            #endif
            cout << mean_ss.str() << endl;
        
            //printf("Mean Flow %f",float(mean));

            if (0){
                double font_size = 0.5; //1.0;//#10.0;
                cv::putText(frame, mean_ss.str(), cv::Point(10,50), cv::FONT_HERSHEY_COMPLEX_SMALL, font_size, cv::Scalar(0,0,128));
            }
        #endif

        //-------------------------------------------
        // Get Diff
        //-------------------------------------------
        
        //cv::Mat diff;
        cv::absdiff(flow_magnitude,mean, diff);

        #ifdef PRINTDEBUG
            printf("#Channels: flow=%i mag=%i diff=%i\n",flow.channels(),flow_magnitude.channels(),diff.channels());
        #endif

        //-------------------------------------------
        // Get Output Image
        //-------------------------------------------

        if (1){

            #ifdef UseAbsoluteMax
                double Min = 0;   // [frame/s]
                double Max = 0.8;//0.6;//1.0; // [frame/s]
            #else
                double Min,Max;
                cv::minMaxLoc(diff, &Min, &Max);
            #endif

            if (Min!=Max){ 
                diff -= Min;
                diff.convertTo(diff,CV_8U,255.0/(Max-Min));
            }


            #ifdef SHOWIMG
                if (0){
                    cvtColor(diff,frame, CV_GRAY2BGR);
                }
                else{
                    Mat R = diff;
                    Mat G = 255-diff;
                    Mat B;
                    B = Mat::zeros(diff.size(),CV_8U);


                    vector<Mat> channels;
                    channels.push_back(B);
                    channels.push_back(G);
                    channels.push_back(R);

                    Mat BGR;
                    cv::merge(channels,BGR);

                    cv::addWeighted(BGR,0.5,frame_new,0.5,1.0,frame);
                    printf("Converted to (%i,%i|%i)\n",frame.size().height,frame.size().width,frame.channels());
                }
            #endif
        }     
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Fix Frame Size
    //+++++++++++++++++++++++++++++++++++++++++++

    if (frame.size()!=org_size){ // Check correct Columns & Rows 
       printf("Converting from (%i,%i|%i) to (%i,%i)\n",frame.size().height,frame.size().width,frame.channels(),org_size.height,org_size.width);
       cv::resize(frame,frame, org_size);
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Control Direction
    //+++++++++++++++++++++++++++++++++++++++++++

    if (frameNr>0){
        //-------------------------------------------
        // Get Colums
        //-------------------------------------------

        cv::Size ColumnSize(1,NrColumns);
        Mat Columns;
        cv::resize(diff,Columns, ColumnSize);

        //-------------------------------------------
        // Get Direction
        //-------------------------------------------
            
        double minVal,maxVal;
        int minIdx,maxIdx;
        cv::minMaxIdx(Columns,&minVal,&maxVal,&minIdx,&maxIdx);

        int Idx = minIdx;
        OptPer[0] = 0.0;
        OptPer[1] = (float(Idx)+0.5)/float(NrColumns)-0.5;

        for (int i=0;i<2;i++){
            float Dif = OptPer[i]-RecPer[i];
            RecPer[i] +=Dif*float(dt)/1000.0/K;
        }

        //--- Get Angle ---
        float MaxAngle = 20;
        for (int i=0;i<2;i++){
            RefAng[i] = RecPer[i]*MaxAngle;
        }

        #ifdef PRINTDEBUG
            cout << "Recommended dHeading: "<< RefAng << endl;
        #endif

        //-------------------------------------------
        // Show Direction
        //-------------------------------------------

        #ifdef SHOWIMG
            #ifdef SHOWDIR

                cv::Point OptDir    = cv::Point((OptPer[0]+0.5)*org_size.width,(OptPer[1]+0.5)*org_size.height);
                cv::Point RecDir    = cv::Point((RecPer[0]+0.5)*org_size.width,(RecPer[1]+0.5)*org_size.height);

                cv::Scalar OptColor = cv::Scalar(128, 128, 0);
                cv::Scalar RecColor = cv::Scalar(0, 128, 0);
                int radius = 10;
                int thickness = 4;

                //--- Draw Arrows ---
                arrowedLine(frame, OptDir,RecDir, OptColor, thickness);

                //--- Draw Circles ---
                cv::circle(frame, OptDir, radius, OptColor,thickness);
                cv::circle(frame, RecDir, radius, RecColor,thickness);


            #endif
        #endif
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Addendum
    //+++++++++++++++++++++++++++++++++++++++++++

    //--- Store Old Frame --
    frame_old_g = frame_new_g.clone();
    frameNr++;

    #ifdef UsePROFILER
        CPPP->EndCheckPoints();
    #endif  

    //--- Return Frame --
    return frame;

}

//****************************************************************************************
// Main Function
//****************************************************************************************

int RunOpticalAvoider(char *raw_img_data, int width, int height){
    #ifdef PRINTDEBUG
        printf("Running OpticalAvoider (OpenCV)...\n");
    #endif

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
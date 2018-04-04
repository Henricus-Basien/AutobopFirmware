// Header ==> Function Declarations

#include <cstdlib>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <sys/time.h>

#include <iostream>

using namespace std;
using namespace cv;

#ifndef VIDEO_H
#define VIDEO_H

class VIDEO {
public: 
    
    //Default Constructor
    VIDEO();
    
    
    //Overload Constructor
    VIDEO(string);
    
    //Destructor
    ~VIDEO();
    
    //Accessor Functions
    
    string getName() const;
        // getName - returns name of patient
    
    void printName() const;
        // printName - prints name of patient on screen
    
    void Initt();
        //Initialization function
    

    
    
    //Mutator Functions

    void setName(string);
        // setName - sets the name of the video
    
    
    void loadFrame(Mat);
        // loads a single frame, can be used as alternative to loadVideo
    
    
    void setFrames(bool AddNew=true);
        
    void showFrame();
        // Show the frame currently stored in the frame variable
    
    vector<Point2f> findFeatures();
        // find the features in the current frame using GoodFeaturesToTrack
    
    void calcOptFlow();
        // insert the new frame, the previous frame is needed to 
        // The old points, new points, and difference vector is returned. 
    
    vector<Point2f> getOldPoints() const;
    
    vector<Point2f> getNewPoints() const;
    
    vector<Point2f> getFlowVectors() const;
    
private:
    //Member Variables
    long frameCounter = 0;
    string newName;
    Mat oldFrame;
    Mat newFrame;
    Mat *oldFrame_ptr;
    Mat *newFrame_ptr;
    Mat newFrame_gray;
    Mat *newFrame_gray_ptr;
    Mat oldFrame_gray;
    Mat *oldFrame_gray_ptr;
    vector<Point2f> corners;
    double qualityLevel = 0.005;
    double minDistance = 10;
    int blockSize = 1;
    bool useHarrisDetector = false;
    double k = 0.04;
    Mat copy;
    int maxCorners = 2000;
    int maxTrackbar = 500;
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> points[2];
    
    TermCriteria termcrit;
    Size subPixWinSize;
    Size winSize;
    
    vector<Point2f> oldPoints;
    vector<Point2f> newPoints;
    vector<Point2f> oldPoints_ptr;
    vector<Point2f> newPoints_ptr;
    
    vector<Point2f> flowVectors;
    Mat mflowVectors;

    int r = 4;
    
    
    
    
    
    
    
};

#endif

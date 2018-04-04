//Function Definitions

#include "VIDEO.h"

VIDEO::VIDEO() {
    VIDEO::Initt();
}



VIDEO::VIDEO(string name) {
    VIDEO::Initt();
    this->newName = name;

}


void VIDEO::Initt() {

    TermCriteria tempcrit(cv::TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    this->termcrit = tempcrit;
    Size tempsubPixWinSize(10,10);
    Size tempwinSize(31,31);
    this->subPixWinSize = tempsubPixWinSize;
    this->winSize = tempwinSize;
    
}

VIDEO::~VIDEO() {
    
}

string VIDEO::getName() const {
    return this->newName;
}

vector<Point2f> VIDEO::getOldPoints() const {
    return this->oldPoints;
}

vector<Point2f> VIDEO::getNewPoints() const {
    return this->newPoints;
}

vector<Point2f> VIDEO::getFlowVectors() const {
    return this->flowVectors;
}


void VIDEO::printName() const {
    cout << endl << "Video name: " << this->newName << endl;

}

void VIDEO::setName(string name) {
    this->newName = name;
}



void VIDEO::loadFrame(Mat VideoFrame) {
    if (this->frameCounter>1){

        oldFrame_gray = newFrame_gray.clone();
        
    }
    //return (*this->newFrame);
    
    newFrame = VideoFrame;
    cvtColor( this->newFrame, this->newFrame_gray, COLOR_BGR2GRAY );
    
    frameCounter++;
}



void VIDEO::showFrame() {
    /// Show what you got
    namedWindow( "hoi", WINDOW_AUTOSIZE );
    imshow( "hoi", this->newFrame );
    waitKey(0);
}

vector<Point2f> VIDEO::findFeatures() {
    



    /// Apply corner detection
    goodFeaturesToTrack( this->newFrame_gray,
                 this->oldPoints,
                 this->maxCorners,
                 this->qualityLevel,
                 this->minDistance,
                 Mat(),
                 this->blockSize,
                 this->useHarrisDetector,
                 this->k );



    return this->corners;
}


void VIDEO::calcOptFlow(){
    if (this->frameCounter>3){

        calcOpticalFlowPyrLK(this->oldFrame_gray, this->newFrame_gray, this->oldPoints, this->newPoints , this->status, this->err, this->winSize, 3, this->termcrit, 0, 0.001);
        this->mflowVectors = Mat(this->newPoints) - Mat(this->oldPoints);
        this->mflowVectors.copyTo(this->flowVectors);


        //cout << flowVectors << endl;
    }



  


}



 



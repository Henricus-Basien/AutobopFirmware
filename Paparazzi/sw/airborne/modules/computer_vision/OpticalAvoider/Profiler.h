#ifndef Profiler_H
#define Profiler_H

//********************************************************************************************
// Imports
//********************************************************************************************

//========================================================
// External
//========================================================

//#include <Arduino.h>
#include <stdio.h>
#include <string>

//========================================================
// Internal
//========================================================

#include "Singleton.h"

//********************************************************************************************
// Settings
//********************************************************************************************

#define HDB_uint uint32_t //uint64_t //uint32_t
#define CP_TimeType unsigned int
//#define ProfilingTimer millis
//#define ProfilingTimeScale 1.0
#define ProfilingTimer std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() //micros
#define ProfilingTimeScale 1.0//1000.0

typedef std::string String;
//typedef std::basic_string<char> String;
//#define String string

//-------------------------------------------
// Checkpoint Profillin
//-------------------------------------------

#ifdef ESP32
    #define MaxNrCheckpoints 500
#else
    #define MaxNrCheckpoints 150//100//80//65//50//35//30//20//30
#endif
#define MAX_CHECKPOINT_LEVEL 6//5//4//5//4 //3 // 5

//-------------------------------------------
// Relative Nesting
//-------------------------------------------

#define NO_RELATIVE_NESTING -1

//-------------------------------------------
// Seperators
//-------------------------------------------

#define MaxNrLevelSeperators 3

//********************************************************************************************
// Checkpoints
//********************************************************************************************

class CheckPoint{

    public:
        // Variables
        String Descriptor = "";
        unsigned int Level[MAX_CHECKPOINT_LEVEL] = {0};//{0,0,0,0,0};//{0,0,0};//{0,0,0,0,0};
        CP_TimeType dt = 0;//HDB_uint dt = 0;
        CP_TimeType AccumulatedTime = 0;//HDB_uint dt = 0;
        CP_TimeType HoldTime = 0;
        int CheckpointLevel = 0;

        CP_TimeType WaitTime = 0;

        // Functions
        void Reset();
        void AddWaitTime(unsigned long wt);

        String GetCheckText();
        void   SetCheckText();
        String CheckText = "";
};

//********************************************************************************************
// Profiler
//********************************************************************************************

class Profiler:public Singleton<Profiler>{

    //========================================================================================
    // Public
    //========================================================================================
    
    public:

        //+++++++++++++++++++++++++++++++++++++++++++
        // Variables
        //+++++++++++++++++++++++++++++++++++++++++++
        
        int CheckPointNr = 0;
        CheckPoint CheckPoints[MaxNrCheckpoints];

        //-------------------------------------------
        // Seperators
        //-------------------------------------------

        char* LevelSeperators[MaxNrLevelSeperators] = {"*","=","-"};//"*=-";//

        //-------------------------------------------
        // Settings
        //-------------------------------------------
        
        bool CalcAccumlatedTime = true;//false;//true;
        bool ProfilingActive = false;

        //-------------------------------------------
        // Timing
        //-------------------------------------------
        
        HDB_uint TotalWaitTime = 0;

        //+++++++++++++++++++++++++++++++++++++++++++
        // Functions
        //+++++++++++++++++++++++++++++++++++++++++++
        
        void ResetCheckPoints(bool ActivateProfilling=false);
        void SetCheckPoint(String Descriptor="Checkpoint Test",int Level=0);//,bool PrintSeperator=false);
        void EndCheckPoints();
        void PrintCheckPointProfilling();

        //-------------------------------------------
        // Relative Nesting
        //-------------------------------------------

        int CheckRelativeNesting(int Level);
        void ShiftRelativeNesting(int n);
        void IncrementRelativeNesting();
        void DecrementRelativeNesting();
        
    //========================================================================================
    // Private
    //========================================================================================

    private:

        //-------------------------------------------
        // Safety Checks
        //-------------------------------------------

        bool ReachedEnd = false;
        int OverflowCounter = 0;

        //-------------------------------------------
        // Timing
        //-------------------------------------------
        
        HDB_uint CheckPointTime = 0;
        HDB_uint CheckPointTime_0 = 0;
        HDB_uint CheckPointsTotalTime = 0;
        HDB_uint AccumulativeTime = 0;
        HDB_uint TotalHoldTime = 0;

        HDB_uint TotalTimeSpendInternal = 0;

        //-------------------------------------------
        // Relative Nesting
        //-------------------------------------------

        int Rel_NestedBaseLV = NO_RELATIVE_NESTING; 

        //-------------------------------------------
        // Printing
        //-------------------------------------------

        int MaxDescriptorLength = 0;

};

extern Profiler* CPPP;

#endif // Profiler_H

//********************************************************************************************
// Imports
//********************************************************************************************

#include "Profiler.h"

//#include "Serial_Connector.h"

//********************************************************************************************
// CheckPoints
//********************************************************************************************

void CheckPoint::Reset(){

    this->Descriptor = "";
    for (int i=0;i<CheckpointLevel+1;i++){
        this->Level[i] = 0;
    }
    this->dt = 0;
    this->AccumulatedTime = 0;

    this->CheckText = "";

    this->WaitTime = 0;

}

void CheckPoint::AddWaitTime(unsigned long wt){

    this->WaitTime+=wt;
    CPPP->TotalWaitTime+=wt;

}

String CheckPoint::GetCheckText(){

    return this->CheckText;

}

void CheckPoint::SetCheckText(){
    this->CheckText = "CP#";
    for (int level=0;level<this->CheckpointLevel+1;level++){
        if (level==this->CheckpointLevel){
            this->CheckText = this->CheckText+this->Level[level];
        }
        else{
            this->CheckText = this->CheckText+this->Level[level]+".";
        }
    }
    this->CheckText = this->CheckText+"|"+this->Descriptor;//+"...";
}




//********************************************************************************************
// Profiler
//********************************************************************************************

void Profiler::ResetCheckPoints(bool ActivateProfilling){

    this->ProfilingActive = ActivateProfilling;

    if (1){
        if (this->CheckPointNr>=MaxNrCheckpoints){
            this->CheckPointNr = MaxNrCheckpoints-1;
        }

        for (int i=0;i<=this->CheckPointNr;i++){
           this->CheckPoints[i].Reset();
//            this->CheckPointDescriptors[i] = "";
//            this->CheckPointTimesteps[i] = 0;
        }
    }

    this->CheckPointNr = 0;
    if (this->ProfilingActive){
        this->CheckPointTime = ProfilingTimer;//millis();
        this->CheckPointTime_0 = ProfilingTimer;//millis();
        this->AccumulativeTime = 0;
    }
    this->MaxDescriptorLength = 0;

    this->ReachedEnd = false;
    this->OverflowCounter = 0;

    this->TotalTimeSpendInternal = 0;

    this->TotalWaitTime = 0;

    // //+++++++++++++++++++++++++++++++++++++++++++
    // // Serial Connector Check Time
    // //+++++++++++++++++++++++++++++++++++++++++++
    
    // if (this->ProfilingActive){
    //     SC->CheckSendTime = true;
    // }
    // else{
    //     SC->CheckSendTime = false;
    // }
    // SC->ResetTotalSendTracker();

}



void Profiler::SetCheckPoint(String Descriptor,int Level){//,bool PrintSeperator){

    if (!this->ProfilingActive){
        return;
    }

    HDB_uint CPPP_t0 = micros();

    //+++++++++++++++++++++++++++++++++++++++++++
    // Get CheckPoints
    //+++++++++++++++++++++++++++++++++++++++++++
    
    CheckPoint* CP = &this->CheckPoints[this->CheckPointNr];
    CheckPoint* CP_prev;
    if (this->CheckPointNr>0){
        CP_prev = &this->CheckPoints[this->CheckPointNr-1];
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Checks
    //+++++++++++++++++++++++++++++++++++++++++++
    
    //-------------------------------------------
    // Check Relative Nesting
    //-------------------------------------------
    
    //printf("Level-Before: "+String(Level,DEC));
    Level = Profiler::CheckRelativeNesting(Level);
    //printf("Level-After: "+String(Level,DEC));

    if (Level>=MAX_CHECKPOINT_LEVEL){
        //SC->Warning("Maximum Checkpoint Nesting Depth ("+String(Level+1,DEC)+">"+String(MAX_CHECKPOINT_LEVEL,DEC)+") reached!");
        Level = MAX_CHECKPOINT_LEVEL-1;
    }

    //-------------------------------------------
    // Check Overflow
    //-------------------------------------------

    //    if (this->CheckPointNr>=MaxNrCheckpoints-1){
    //        return;
    //    }
    
    if (this->ReachedEnd){
        this->OverflowCounter++;
        return;
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Set CheckPoint
    //+++++++++++++++++++++++++++++++++++++++++++

    if (this->CheckPointNr==0 || Level==0){
        if (this->CheckPointNr==0){
            CP->Level[0] = this->CheckPointNr;
        }
        else{
            CP->Level[0] = CP_prev->Level[0]+1;
        }
        CP->CheckpointLevel = Level;
    }
    else{
        CP->CheckpointLevel = Level;
        for (int level=Level;level>=0;level--){
            if (Level==level){
                CP->Level[level] = CP_prev->Level[level]+1;
            }
            else{
                CP->Level[level] = CP_prev->Level[level];
            }
        }
    }

    //-------------------------------------------
    // Set Descriptor
    //-------------------------------------------

    CP->Descriptor = Descriptor;//this->CheckPointDescriptors[this->CheckPointNr] = Descriptor;
    CP->SetCheckText();

    if ((CP->GetCheckText()).length()>this->MaxDescriptorLength){
        this->MaxDescriptorLength = (CP->GetCheckText()).length();
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Set Nested Timing
    //+++++++++++++++++++++++++++++++++++++++++++

    if (this->CheckPointNr>0){
        CP_prev->dt = ProfilingTimer-this->CheckPointTime;//millis()-this->CheckPointTime;//this->CheckPointTimesteps[this->CheckPointNr-1] = millis()-this->CheckPointTime;
        this->AccumulativeTime+=CP_prev->dt;

        if (this->CalcAccumlatedTime){
            CP_prev->AccumulatedTime+=CP_prev->dt;

            // Accumulated Time
            if (CP_prev->CheckpointLevel>0 && this->CheckPointNr>1){
                for (int CPNr=this->CheckPointNr-2;CPNr>=0;CPNr--){
                    if (0){
                        if (this->CheckPoints[CPNr].Level[CP_prev->CheckpointLevel-1]<CP_prev->Level[CP_prev->CheckpointLevel-1]){//if (this->CheckPoints[CPNr].Level[0]<CP_prev->Level[0]){
                            break;
                        }
                        if (this->CheckPoints[CPNr].CheckpointLevel<CP_prev->CheckpointLevel){// && this->CheckPoints[CPNr].Level[0]==CP_prev->Level[0]){
                            this->CheckPoints[CPNr].AccumulatedTime+=CP_prev->dt;
                        }
                     }
                    else{
                        if (this->CheckPoints[CPNr].Level[0]<CP_prev->Level[0]){
                            break;
                        }
                        bool AddT = true;
                        for (int LV=this->CheckPoints[CPNr].CheckpointLevel;LV>0;LV--){//for (int LV=CP_prev->CheckpointLevel;LV>=this->CheckPoints[CPNr].CheckpointLevel;LV--){
                            if (CP_prev->Level[LV] != this->CheckPoints[CPNr].Level[LV]){
                                AddT=false;
                                //printf("FalseLV@"+String(LV,DEC));
                                break;
                            }
                        }
                        //printf(this->CheckPoints[CPNr].GetCheckText()+"|"+CP_prev->GetCheckText());
                        if (AddT){
                            this->CheckPoints[CPNr].AccumulatedTime+=CP_prev->dt;
                            //printf("Added!");
                        }
                    }
                }
            }
        }
    }

    this->CheckPointTime = ProfilingTimer;//millis();



    if (this->CheckPointNr>=MaxNrCheckpoints-1){
        //CP->Descriptor += " - !Warning Max CheckPoint Reached!";
        this->ReachedEnd = true;
        //this->CheckPointNr++;
    }
//    else{
//        this->CheckPointNr++;
//    }

    this->CheckPointNr++;

    this->TotalTimeSpendInternal += micros()-CPPP_t0;

}

void Profiler::EndCheckPoints(){

    HDB_uint CPPP_t0 = micros();

    CheckPoint* CP_prev = &this->CheckPoints[this->CheckPointNr-1];

    if (this->ProfilingActive){
        CP_prev->dt = ProfilingTimer-this->CheckPointTime;//millis()-this->CheckPointTime;
        CP_prev->AccumulatedTime+=CP_prev->dt;

        if (this->CalcAccumlatedTime){
            this->AccumulativeTime+=CP_prev->dt;
            // Accumulated Time
            if (CP_prev->CheckpointLevel>0 && this->CheckPointNr>1){
                for (int CPNr=this->CheckPointNr-2;CPNr>=0;CPNr--){
                    if (this->CheckPoints[CPNr].Level[0]<CP_prev->Level[0]){
                        break;
                    }
                    bool AddT = true;
                    for (int LV=this->CheckPoints[CPNr].CheckpointLevel;LV>0;LV--){//for (int LV=CP_prev->CheckpointLevel;LV>=this->CheckPoints[CPNr].CheckpointLevel;LV--){
                        if (CP_prev->Level[LV] != this->CheckPoints[CPNr].Level[LV]){
                            AddT=false;
                            break;
                        }
                    }
                    if (AddT){
                        this->CheckPoints[CPNr].AccumulatedTime+=CP_prev->dt;
                    }
                }
            }
        }

        this->CheckPointsTotalTime = ProfilingTimer-this->CheckPointTime_0;//millis()-this->CheckPointTime_0;

        Profiler::PrintCheckPointProfilling();

        this->ProfilingActive = false;
    }

    this->TotalTimeSpendInternal += micros()-CPPP_t0;
}

//========================================================================================
// Check Relative Nesting
//========================================================================================

int Profiler::CheckRelativeNesting(int Level){

    if (Level<0){

        //-------------------------------------------
        // Nesting Active?
        //-------------------------------------------
        
        if (this->Rel_NestedBaseLV == NO_RELATIVE_NESTING){
            if (this->CheckPointNr==0){
                this->Rel_NestedBaseLV = 0;
            }
            else{
                CheckPoint* CP_prev = &this->CheckPoints[this->CheckPointNr-1];
                this->Rel_NestedBaseLV = CP_prev->CheckpointLevel;
                //printf(">>> Previous Checkpoint: "+CP_prev->Descriptor);
            }
        }
        Level = this->Rel_NestedBaseLV-Level;
        //printf("Base: "+String(this->Rel_NestedBaseLV,DEC)+" | LV: "+String(Level));
    }
    else if(this->Rel_NestedBaseLV != NO_RELATIVE_NESTING){
        this->Rel_NestedBaseLV = NO_RELATIVE_NESTING;
        //printf("Reset BaseLV");
    }
    return Level;
}

void Profiler::ShiftRelativeNesting(int n){
    if (this->Rel_NestedBaseLV != NO_RELATIVE_NESTING){
        this->Rel_NestedBaseLV += n;
    }
}

void Profiler::IncrementRelativeNesting(){
    Profiler::ShiftRelativeNesting(1);
}
void Profiler::DecrementRelativeNesting(){
    Profiler::ShiftRelativeNesting(-1);
}

//========================================================================================
// Print Results
//========================================================================================

void Profiler::PrintCheckPointProfilling(){

    unsigned long ProfPrint_t0 = micros();

    //SC->PrintHeader("Checkpoint Profiling Results",100,"*");
    printf("Checkpoint Profiling Results")

    if (this->ReachedEnd){
        printf("The Profiler Ran out of Checkpoints|Max="+String(MaxNrCheckpoints,DEC)+"|Overflow: "+String(this->OverflowCounter,DEC)+" ("+String(MaxNrCheckpoints+this->OverflowCounter,DEC)+")");
    }

    //+++++++++++++++++++++++++++++++++++++++++++
    // Summary
    //+++++++++++++++++++++++++++++++++++++++++++
    
    String ProfilingSummary = "Total Time: ";
    ProfilingSummary = ProfilingSummary + float(this->CheckPointsTotalTime)/ProfilingTimeScale + " ms"+"|"+ProfilingTimeScale*1000.0/this->CheckPointsTotalTime+" Hz";//+"|-|"+ float(this->AccumulativeTime)/ProfilingTimeScale + " ms"+"|"+ProfilingTimeScale*1000.0/this->AccumulativeTime+" Hz"+"|-|"+100.0*float(this->AccumulativeTime)/this->CheckPointsTotalTime+"%"+"|Dif: "+float(this->CheckPointsTotalTime-this->AccumulativeTime)/ProfilingTimeScale +"ms";
    if (this->TotalWaitTime>0){
        ProfilingSummary = ProfilingSummary + " // {Total WaitTime: "+(float(this->TotalWaitTime)/ProfilingTimeScale)+" ms - > Total-Wait="+(float(this->CheckPointsTotalTime-this->TotalWaitTime)/ProfilingTimeScale)+"}";
    }

    printf(ProfilingSummary);
    ProfilingSummary = "Accumulated Time: ";
    ProfilingSummary = ProfilingSummary + float(this->AccumulativeTime)/ProfilingTimeScale + " ms"+"|"+ProfilingTimeScale*1000.0/this->AccumulativeTime+" Hz";
    printf(ProfilingSummary);
    ProfilingSummary = "SCDif: ";
    ProfilingSummary = ProfilingSummary + 100.0*float(this->AccumulativeTime)/this->CheckPointsTotalTime+"%"+"|"+float(this->CheckPointsTotalTime-this->AccumulativeTime)/ProfilingTimeScale +"ms";
    printf(ProfilingSummary);

    //+++++++++++++++++++++++++++++++++++++++++++
    // Description
    //+++++++++++++++++++++++++++++++++++++++++++
    
    String Description = ">>>CP#//CP_LV|Description: dt,%dt/Total,%dt/AccumulatedTotal";
    if (this->CalcAccumlatedTime){
        Description+="||Accumulated_dt,%Accumulated_dt/Total,%Accumulated_dt/AccumulatedTotal";
    }
    printf(Description);

    //+++++++++++++++++++++++++++++++++++++++++++
    // Data
    //+++++++++++++++++++++++++++++++++++++++++++
    
    for (int i=0;i<this->CheckPointNr;i++){
        CheckPoint* CP = &this->CheckPoints[i];
        //String CheckpointString = "Checkpoint #";
        //CheckpointString = CheckpointString+i+": "+this->CheckPointDescriptors[i]+" - "+this->CheckPointTimesteps[i]+" ms"+" - "+100.0*this->CheckPointTimesteps[i]/this->CheckPointsTotalTime+"%";
        String CheckpointString = "#";//cd /
        CheckpointString = CheckpointString + i+"//"+CP->GetCheckText()+": ";

        // Add WhiteSpaces
        int NrSpaces = this->MaxDescriptorLength-(CP->GetCheckText()).length()+2;
        if (i>=10){
            NrSpaces-=1;
        }
        CheckpointString = CheckpointString +MultiplySeq(" ",NrSpaces);
        // Add Data
        CheckpointString = CheckpointString +String(float(CP->dt)/ProfilingTimeScale,3)+" ms"+" - "+100.0*float(CP->dt)/float(this->AccumulativeTime)+"%"+" - "+100.0*float(CP->dt)/float(this->CheckPointsTotalTime)+"%";
        if (this->CalcAccumlatedTime && (CP->AccumulatedTime-CP->dt)>1.0){
            CheckpointString = CheckpointString +" ||"+" [LV#"+CP->CheckpointLevel+": "+String(float(CP->AccumulatedTime)/ProfilingTimeScale,3)+" ms"+" - "+100.0*float(CP->AccumulatedTime)/float(this->AccumulativeTime)+"%"+" - "+100.0*float(CP->AccumulatedTime)/float(this->CheckPointsTotalTime)+"%"+"]";
        }
        if (CP->WaitTime>0){
            CheckpointString = CheckpointString +" ||"+" {WaitT: "+(float(CP->WaitTime)/1000.)+" ms}";
        }

        //-------------------------------------------
        // Check Child Levels
        //-------------------------------------------

        int n = i+1;
        int MaxLV = CP->CheckpointLevel;

        while (!n<this->CheckPointNr){
            CheckPoint* CP_child = &this->CheckPoints[n];

            //printf("CP: "+CP->Descriptor+" ("+String(CP->CheckpointLevel,DEC)+")"+"|""CP_Child: "+CP_child->Descriptor+" ("+String(CP_child->CheckpointLevel,DEC)+")"+"|"+"MaxLV: "+String(MaxLV,DEC));

            if (CP_child->CheckpointLevel<=CP->CheckpointLevel){
                break;
            }

            if (MaxLV<CP_child->CheckpointLevel){
                MaxLV = CP_child->CheckpointLevel;
            }
            
            n++;
        }

        int HeaderWrite_LV_dif = 1;//2;

        //-------------------------------------------
        // Write Seperators
        //-------------------------------------------

        if (CP->CheckpointLevel==0 || (MaxLV-CP->CheckpointLevel>=HeaderWrite_LV_dif && CP->CheckpointLevel<MaxNrLevelSeperators)){
            printf(MultiplySeq(this->LevelSeperators[CP->CheckpointLevel],CheckpointString.length()));//CheckpointString = CheckpointString +"\n"+CheckpointString;
        }

        //-------------------------------------------
        // Print Entry
        //-------------------------------------------

        printf(CheckpointString);

        //AccumulativeTime+=CP->dt;
    }

    unsigned long ProfPrint_dt = micros()-ProfPrint_t0;
    printf(">Total Time Spend Internal: "+String(double(this->TotalTimeSpendInternal)/ProfilingTimeScale,2)+" ms|Max Loop Freq without Printing: "+String(1000.*ProfilingTimeScale/double(this->TotalTimeSpendInternal),2)+" Hz");
    printf(">Total Time Spend Printing Profiling Results: "+String(double(ProfPrint_dt)/ProfilingTimeScale,2)+" ms|Max Loop Freq with Profiling on: "+String(1000.*ProfilingTimeScale/double(this->TotalTimeSpendInternal+ProfPrint_dt),2)+" Hz");
    printf(">Total Time Spend Printing ALL: "+String(double(SC->TotalSendTime)/1000,2)+" ms"+"|"+"#Msg Send: "+String(SC->TotalSendCounter,DEC)+" ("+String(float(SC->TotalSendTime)/SC->TotalSendCounter,2)+"us/msg)");
    printf("Estimated Raw Loop Freq: "+String(1000.*ProfilingTimeScale/(float(this->CheckPointsTotalTime)-float(this->TotalTimeSpendInternal-SC->TotalSendTime)),2)+" Hz");


//    String AccumulatedTimeSummary = "Accumulated Time: ";
//    AccumulatedTimeSummary = AccumulatedTimeSummary + float(this->AccumulativeTime)/ProfilingTimeScale + " ms"+"|"+100.0*float(this->AccumulativeTime)/this->CheckPointsTotalTime+"%"+"|Dif: "+float(this->CheckPointsTotalTime-this->AccumulativeTime)/ProfilingTimeScale +"ms";
//    printf(AccumulatedTimeSummary);

}


//==================================================================================
// Define Profiler Object Singleton
//==================================================================================

Profiler* CPPP = Profiler::GetInstance();

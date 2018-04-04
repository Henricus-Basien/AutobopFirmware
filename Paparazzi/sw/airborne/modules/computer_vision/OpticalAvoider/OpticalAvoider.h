// Author: Henricus N. Basien
// Date:Tuesday 03.04.2018

#ifndef OpticalAvoider_H
#define OpticalAvoider_H

#ifdef __cplusplus
extern "C" {
#endif

//+++++++++++++++++++++++++++++++++++++++++++
// Global Variables
//+++++++++++++++++++++++++++++++++++++++++++

int color_count;

extern float MaxAngle;
extern float RefAng[2];

//+++++++++++++++++++++++++++++++++++++++++++
// Global Functions
//+++++++++++++++++++++++++++++++++++++++++++

int RunOpticalAvoider(char *img, int width, int height);

#ifdef __cplusplus
}
#endif

#endif

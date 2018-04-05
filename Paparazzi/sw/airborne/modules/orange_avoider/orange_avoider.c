/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/orange_avoider/orange_avoider.h"
//#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "modules/computer_vision/cv_opencvdemo.h"
#include "modules/computer_vision/OpticalAvoider/OpticalAvoider.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

// #ifndef ORANGE_AVOIDER_LUM_MIN
// #define ORANGE_AVOIDER_LUM_MIN 41
// #endif

// #ifndef ORANGE_AVOIDER_LUM_MAX
// #define ORANGE_AVOIDER_LUM_MAX 183
// #endif

// #ifndef ORANGE_AVOIDER_CB_MIN
// #define ORANGE_AVOIDER_CB_MIN 53
// #endif

// #ifndef ORANGE_AVOIDER_CB_MAX
// #define ORANGE_AVOIDER_CB_MAX 121
// #endif

// #ifndef ORANGE_AVOIDER_CR_MIN
// #define ORANGE_AVOIDER_CR_MIN 134
// #endif

// #ifndef ORANGE_AVOIDER_CR_MAX
// #define ORANGE_AVOIDER_CR_MAX 249
// #endif


uint8_t safeToGoForwards        = false;
//int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
//float maxDistance               = 2.25;
float moveDist 					= 1.5;

// initalize state variables
bool states_upd;
float hdg_chg;
float rngsq;
//struct EnuCoor_i bbp;
int32_t crs;
float sog;
float dx;
float dy;
int obs[5];

float orient = 0.60284; // Cyberzoo is oriented 34.54 deg (0.60284 rad) relative to N
float wallTreshold = 2.7;
float k_wall = 0.15;
float k_obst = 0*0.075;
// heading change limit
float capLimit = (float) M_PI/9;
float bias = 0.1;

uint8_t doingBigEvasiveTurn = false;
uint8_t HaveComputedTurnHeadingGoalForBigEvasiveTurn = false;
float BigEvasiveTurnHeadingGoal = 0.0;
float BigEvasiveTurnAmount = 0.0;
float BigEvasiveTurnAmountToGo = 0.0;
float BigEvasiveTurnCommand = 0.0;

float k_refang = -0.1/10.;
float dHDG_optflow = 0.0;


/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void orange_avoider_init()
{
  // Initialise the variables of the colorfilter to accept orange
  // color_lum_min = ORANGE_AVOIDER_LUM_MIN;
  // color_lum_max = ORANGE_AVOIDER_LUM_MAX;
  // color_cb_min  = ORANGE_AVOIDER_CB_MIN;
  // color_cb_max  = ORANGE_AVOIDER_CB_MAX;
  // color_cr_min  = ORANGE_AVOIDER_CR_MIN;
  // // color_cr_max  = ORANGE_AVOIDER_CR_MAX;
  // // Initialise random values
  // srand(time(NULL));
  // chooseRandomIncrementAvoidance();

  // obs[0] = WP_OBS1;
  // obs[1] = WP_OBS2;
  // obs[2] = WP_OBS3;
  // obs[3] = WP_OBS4;
  // obs[4] = WP_OBS5;
  //VERBOSE_PRINT("Waypoint 1 x: %f, y: %f \n", waypoint_get_x(obs[0]), waypoint_get_y(obs[0]));
}

void orange_avoider_periodic()
{  
  states_upd = false;
  hdg_chg = 0.0;
  struct EnuCoor_i *bbp = stateGetPositionEnu_i(); // Get your current position
  float bbp_x =  POS_FLOAT_OF_BFP(bbp->x);
  float bbp_y = POS_FLOAT_OF_BFP(bbp->y);

  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  float heading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);

  sog           = stateGetHorizontalSpeedNorm_f();



  if(doingBigEvasiveTurn) 
  {
    // okay, so we need to do a Big Turn
    if(!HaveComputedTurnHeadingGoalForBigEvasiveTurn) {
      // first things first: stay here untill turn is completed!
      waypoint_set_here_2d(WP_GOAL);
      waypoint_set_here_2d(WP_TRAJECTORY);
      // first compute our turn command
      FLOAT_ANGLE_NORMALIZE(BigEvasiveTurnAmount);
      BigEvasiveTurnHeadingGoal = heading + BigEvasiveTurnAmount;
      BigEvasiveTurnAmountToGo = BigEvasiveTurnAmount;
      FLOAT_ANGLE_NORMALIZE(BigEvasiveTurnHeadingGoal);
      HaveComputedTurnHeadingGoalForBigEvasiveTurn = true; 

      VERBOSE_PRINT("Starting Big Evasive turn: Amount to go = %f (goal = %f) \n", DegOfRad(BigEvasiveTurnAmount), DegOfRad(BigEvasiveTurnHeadingGoal));
    }
    else {
      BigEvasiveTurnAmountToGo = BigEvasiveTurnHeadingGoal - heading;
      FLOAT_ANGLE_NORMALIZE(BigEvasiveTurnAmountToGo);
    }
    // have we already completed it maybe?
    if(fabs(BigEvasiveTurnAmountToGo) < capLimit/2.0)
    {
      VERBOSE_PRINT("Stopping turn: Amount to go = %f (limit = %f) \n", DegOfRad(BigEvasiveTurnAmountToGo), DegOfRad(capLimit/2.0));
      doingBigEvasiveTurn = false;
      HaveComputedTurnHeadingGoalForBigEvasiveTurn = false;
      BigEvasiveTurnHeadingGoal = 0.0;
      BigEvasiveTurnAmount = 0.0;
      BigEvasiveTurnAmountToGo = 0.0;
      BigEvasiveTurnCommand = 0.0;

      // we have reset everything, go to the next periodic iteration
      return;
    }
    // make sure the heading command change isn't too big
    capHeadingChange(&BigEvasiveTurnAmountToGo);
    


     increase_nav_heading(&nav_heading, incrementForAvoidance);
    return; // stop executing of the rest
  }

  // for (int i = 0; i<5; i++){
  // 	dx = waypoint_get_x(obs[i]) - bbp_x;
  //   dy = waypoint_get_y(obs[i]) - bbp_y;
  // 	rngsq = dx*dx+dy*dy;
  // 	if ((rngsq < (float) 6) && (sog > 0.1)){
  // 		if (states_upd == false){
  // 			crs           = stateGetHorizontalSpeedDir_i();
  // 			states_upd     = true;
  // 		}
  // 		hdg_chg += compute_dHDG_obst();
  // 	}
  // }
  hdg_chg += compute_dHDG_wall(&bbp_x, &bbp_y) ;//* ((float) dt)/1000.0*4.0;


  // optical flow hdg change
  dHDG_optflow = 1./k_refang * ((float) dt) /1000. * RadOfDeg(RefAng[1]);
  VERBOSE_PRINT("Heading change due to wall, obst: %f, %f degrees \n", DegOfRad(hdg_chg), DegOfRad(dHDG_optflow));
  hdg_chg += dHDG_optflow;
  capHeadingChange(&hdg_chg);
  //VERBOSE_PRINT("Heading change: %f degrees \n", 57.3*hdg_chg);
  moveWaypointToAvoid(WP_GOAL, moveDist, &hdg_chg);
  moveWaypointToAvoid(WP_TRAJECTORY, 1.25 * moveDist, &hdg_chg);
  nav_set_heading_towards_waypoint(WP_GOAL);
  return;
}
// headingOffset in degrees
void initiate_bigEvasiveTurn(float headingOffset)
{
  doingBigEvasiveTurn = true;
  BigEvasiveTurnAmount = RadOfDeg(headingOffset);
  VERBOSE_PRINT("BigEvasiveTurn initiated: %f degrees \n", headingOffset);  
}

extern bool doingBigEvasiveTurnCheck() {
  VERBOSE_PRINT("BigEvasiveTurn: %d / %d \n", (bool) doingBigEvasiveTurn, doingBigEvasiveTurn);  
  return (bool) doingBigEvasiveTurn;
}

 float compute_dHDG_obst()
{
  float dCRSobs;
  float turnDir;


  int32_t mbrg = ANGLE_BFP_OF_REAL(atan2f(dx,dy));
  int32_t vbrg = mbrg-crs;
  INT32_ANGLE_NORMALIZE(vbrg);

  float VMG = sog * cosf(ANGLE_FLOAT_OF_BFP(vbrg));
  if(VMG>0.4*sog) {
    turnDir = -1*signOfFloat(vbrg);
    dCRSobs =  turnDir * k_obst *M_PI/(rngsq);
  }
  else {
    dCRSobs = 0;
    turnDir=1;
  }
  //VERBOSE_PRINT("VMG: %f >  %f, dCRS = %f = %f*%f/%f \n", VMG, 0.4*sog, 57*dCRSobs, turnDir, 0.025*M_PI, rngsq);
  return dCRSobs;
}

 float signOfFloat(float variable)
{
	float sign = (float) ((variable > 0) - (variable < 0));
	return sign;
}

 void capHeadingChange(float *dHDG)
{
  
	if(*dHDG > capLimit)
	{
		*dHDG = capLimit;
	}
	else if(*dHDG < -capLimit)
	{
		*dHDG = -capLimit;
	}
	return;
}

 float compute_dHDG_wall(float *posx, float *posy)
{
	float xZoo = *posx * cosf(orient) + *posy * sinf(orient);
	float yZoo = *posy * cosf(orient) - *posx * sinf(orient);
	 
	float v_xZoo = sog*sinf(ANGLE_FLOAT_OF_BFP(crs)+orient);
	float v_yZoo = sog*cosf(ANGLE_FLOAT_OF_BFP(crs)+orient);

	float turnDir = 0;
	float dCRSx = 0;
	float dCRSy = 0;
  uint8_t noprint = true;
  
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the new heading the drone is supposed to keep

  float heading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);
  float hdg_xZoo = (heading+orient);
  float hdg_yZoo = (heading+orient)+M_PI/2;
  
  FLOAT_ANGLE_NORMALIZE(hdg_xZoo);
  FLOAT_ANGLE_NORMALIZE(hdg_yZoo);
	if((fabs(xZoo)>wallTreshold)&&(xZoo*hdg_xZoo > 0))
	{
		float vbrg_xZoo = M_PI/2-(hdg_xZoo);
    FLOAT_ANGLE_NORMALIZE(vbrg_xZoo);
		turnDir = -1 * signOfFloat(vbrg_xZoo) * signOfFloat(xZoo);

    VERBOSE_PRINT("hdg_xZoo = %f, vbrg_xZoo = %f, v_xZoo = %f \n", DegOfRad(hdg_xZoo), DegOfRad(vbrg_xZoo),v_xZoo);
		float rng_x = 3.5-fabs(xZoo);
		dCRSx = turnDir*k_wall*M_PI/(rng_x*rng_x)*(fabs(v_xZoo/sog)+bias);
	  VERBOSE_PRINT("xZoo = %f, yZoo = %f, dCRSx = %f deg: rng_x = %f \n", xZoo, yZoo, 57.3*dCRSx, rng_x);
    noprint = false;
  }

	if((fabs(yZoo)>wallTreshold)&&(yZoo*hdg_yZoo > 0))
	{
		if (turnDir == 0)
		{
			float vbrg_yZoo = -(hdg_xZoo);
      FLOAT_ANGLE_NORMALIZE(vbrg_yZoo);
			turnDir = -1 * signOfFloat(vbrg_yZoo) * signOfFloat(yZoo);
      VERBOSE_PRINT("vbrg_yZoo = %f, v_yZoo = %f \n", DegOfRad(vbrg_yZoo),v_yZoo);
		}

		float rng_y = 3.2-fabs(yZoo);
		dCRSy = turnDir*k_wall*M_PI/(rng_y*rng_y)*(fabs(v_yZoo/sog)+bias);
    VERBOSE_PRINT("xZoo = %f, yZoo = %f, dCRSy = %f deg; rng_y = %f \n", xZoo, yZoo, 57.3*dCRSy, rng_y);
    noprint = false;
	}

  if(noprint)
  {
    VERBOSE_PRINT("xZoo, yZoo = %f, %f \n", xZoo, yZoo);
  }
	return dCRSx+dCRSy;
}



/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
// void orange_avoider_periodic()
// {
//   // Check the amount of orange. If this is above a threshold
//   // you want to turn a certain amount of degrees
//   safeToGoForwards = color_count < tresholdColorCount;
//   VERBOSE_PRINT("Color_count: %d  threshold: %d safe: %d \n", color_count, tresholdColorCount, safeToGoForwards);
//   float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
//   if (safeToGoForwards) {
//     moveWaypointForward(WP_GOAL, moveDistance);
//     moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
//     nav_set_heading_towards_waypoint(WP_GOAL);
//     chooseRandomIncrementAvoidance();
//     trajectoryConfidence += 1;
//   } else {
//     waypoint_set_here_2d(WP_GOAL);
//     waypoint_set_here_2d(WP_TRAJECTORY);
//     increase_nav_heading(&nav_heading, incrementForAvoidance);
//     if (trajectoryConfidence > 5) {
//       trajectoryConfidence -= 4;
//     } else {
//       trajectoryConfidence = 1;
//     }
//   }
//   return;
// }

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  //VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
  //              POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
  //              DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)) );
  return false;
}

static uint8_t calculateAvoidCoord(struct EnuCoor_i *new_coor, float distMeters, float *angleDiversion)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the new heading the drone is supposed to keep

  float newHeading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + *angleDiversion;
  
  FLOAT_ANGLE_NORMALIZE(newHeading);
  VERBOSE_PRINT("Diversion command: %f degrees; newHeading: %f \n", DegOfRad(*angleDiversion), DegOfRad( newHeading+orient));

  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_newheading                 = sinf(newHeading);
  float cos_newheading                 = cosf(newHeading);
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_newheading * (distMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_newheading * (distMeters));
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
  //              POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

uint8_t moveWaypointToAvoid(uint8_t waypoint, float distanceMeters, float *angleDiversion)
{
  struct EnuCoor_i new_coor;
  calculateAvoidCoord(&new_coor, distanceMeters, angleDiversion);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = 10.0;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -10.0;
    //VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}


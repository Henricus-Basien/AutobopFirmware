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
 *
 * Edited by Group 3 (course 2018): Salomon Voorhoeve, Casper Vertregt, Henricus Basien, 
 * 					  				Jari Lubberding, Luuk van Litsenburg & Sukrit Gupta
 * Script now changes the Bebop's heading based on optical flow visual information and
 * location based wall avoidance. 
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

/*
 * =========================================================
 * == DISUSED GLOBAL VARIABLES OF ORIGINAL ORANGE AVOIDER ==
 * =========================================================
 */
uint8_t safeToGoForwards        = false;
//int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
//float maxDistance               = 2.25;
/*
 * ==============
 * == \DISUSED ==
 * ==============
 */

// initalize state variables
int32_t crs;		// Vehicle COURSE (CRS), direction of movement relative to North
float sog;			// Vehicle SPEED OVER GROUND (SOG)

// initialize avoid algorithm variables and gains
// general variables
float hdg_chg; 						// Heading change command given to Bebop each iteration step
float moveDist = 1.5;				// Setting the reference Waypoint distance, determining the forward speed

// Optical flow obstacle avoider variables
float dHDG_optflow = 0.0;			// Heading change command resultng from optical flow obstacle avoidance
float k_refang = -0.1/10.;			// Gain to tune heading deviation resulting from optical flow
float k_obst = 0*0.075; 			// Gain to tune heading deviation resulting from optical flow

// Wall avoider variables
float orient = 0.60284; 			// Cyberzoo is oriented 34.54 deg (0.60284 rad) relative to North
float wallTreshold = 2.7;			// Parameter 
float k_wall = 0.15;				// Gain to tune heading deviation resulting from wall distance
float capLimit = (float) M_PI/9;	// Heading deviation limit in 1 timestep
float bias = 0.1;					// Bias to slowly move away from walls when flying parallel. 

/* ============================================================================================================
 * DISUSED FEATURE
 * Parameters for an EVASIVE MANEUVER, which worked in the simulator but not in the Zoo, so was not implemented
 * ============================================================================================================
 */ 
uint8_t doingBigEvasiveTurn = false;	
uint8_t HaveComputedTurnHeadingGoalForBigEvasiveTurn = false;
float BigEvasiveTurnHeadingGoal = 0.0;
float BigEvasiveTurnAmount = 0.0;
float BigEvasiveTurnAmountToGo = 0.0;
float BigEvasiveTurnCommand = 0.0;


// The name orange_avoider is still 
void orange_avoider_periodic()
{  
  // reset heading change command
  hdg_chg = 0.0;

  // Get your current state
  //position
  struct EnuCoor_i *bbp = stateGetPositionEnu_i(); 
  float bbp_x =  POS_FLOAT_OF_BFP(bbp->x);
  float bbp_y = POS_FLOAT_OF_BFP(bbp->y);
  // attitude
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  float heading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);
  // speed
  sog           = stateGetHorizontalSpeedNorm_f();

  /* ==========================================================================================================
   * DISUSED FEATURE
   * Execution of an EVASIVE MANEUVER, which worked in the simulator but not in the Zoo, so was not implemented
   * ==========================================================================================================
   */ 
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
  /*
   * ==============
   * == \DISUSED ==
   * ==============
   */

  // compute heading change due to walls and add to total heading change
  hdg_chg += compute_dHDG_wall(&bbp_x, &bbp_y) ;//* ((float) dt)/1000.0*4.0;

  // compute optical flow heading change, print both contributions and add to total
  dHDG_optflow = 1./k_refang * ((float) dt) /1000. * RadOfDeg(RefAng[1]);
  VERBOSE_PRINT("Heading change due to wall, obst: %f, %f degrees \n", DegOfRad(hdg_chg), DegOfRad(dHDG_optflow));
  hdg_chg += dHDG_optflow;

  // cap heading change to avoid giving too big commands in a single time step
  capHeadingChange(&hdg_chg);

  // execute command by moving waypoint in the correct direction and pointing the vehicle towards it
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

/*
 * computes the sign of a float varible (returns +1 or -1 float)
 */
float signOfFloat(float variable)
{
	float sign = (float) ((variable > 0) - (variable < 0));
	return sign;
}

/*
 * Caps the commanded heding change to set maximum commanded change
 */
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

/*
 * Calculates the commanded heading change resulting from proximity to the walls. 
 * It chooses the easiest direction to turn away from the wall, depending on current heading in combination with the approached wall (determined by XY position)
 */
float compute_dHDG_wall(float *posx, float *posy)
{
  // Resetting heading deviation variables 
  float turnDir = 0;
  float dHDGx = 0;
  float dHDGy = 0;
  uint8_t noprint = true;

  // coordinate transformation from ENU-coordinates to ZOO coordinates (Y pointing outward from control table, X to the right)
  float xZoo = *posx * cosf(orient) + *posy * sinf(orient);
  float yZoo = *posy * cosf(orient) - *posx * sinf(orient);
  float v_xZoo = sog*sinf(ANGLE_FLOAT_OF_BFP(crs)+orient);
  float v_yZoo = sog*cosf(ANGLE_FLOAT_OF_BFP(crs)+orient); 
  
  // heading transformations
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i(); 
  float heading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);
  // get heading relative to ZOO positive Y axis, dividing the zoo into positive hdg_x and negative hdg_x
  float hdg_xZoo = (heading+orient);
  // get heading relative to ZOO positive X axis, dividing the zoo into positive hdg_y and negative hdg_y
  float hdg_yZoo = (heading+orient)+M_PI/2;
  FLOAT_ANGLE_NORMALIZE(hdg_xZoo);
  FLOAT_ANGLE_NORMALIZE(hdg_yZoo);

  // Calculate the new heading the drone is supposed to keep 

  // HEADING DEVIATION DUE TO SIDE WALLS (X)
  // Only compute deviation if wall is closer than threshold and if vehicle points in direction of the closest wall
  if((fabs(xZoo)>wallTreshold)&&(xZoo*hdg_xZoo > 0))
  {
  	// get bearing of the wall relative to the direction of the nose of the vehicle
  	float rbrg_xZoo = M_PI/2-(hdg_xZoo);
    FLOAT_ANGLE_NORMALIZE(rbrg_xZoo);

    // determine turn direction that makes the easiest avoidance of the wall
  	turnDir = -1 * signOfFloat(rbrg_xZoo) * signOfFloat(xZoo);  
    VERBOSE_PRINT("hdg_xZoo = %f, rbrg_xZoo = %f, v_xZoo = %f \n", DegOfRad(hdg_xZoo), DegOfRad(rbrg_xZoo),v_xZoo);

    // compute range to wall and commanded heading change
  	float rng_x = 3.5-fabs(xZoo);
  	dHDGx = turnDir*k_wall*M_PI/(rng_x*rng_x)*(fabs(v_xZoo/sog)+bias);

  	// print result for debug purposes
    VERBOSE_PRINT("xZoo = %f, yZoo = %f, dHDGx = %f deg: rng_x = %f \n", xZoo, yZoo, 57.3*dHDGx, rng_x);
    noprint = false;
  }

  // HEADING DEVIATION DUE TO TOP AND BOTTOM WALLS (Y)
  // Only compute deviation if wall is closer than threshold and if vehicle points in direction of the closest wall
  if((fabs(yZoo)>wallTreshold)&&(yZoo*hdg_yZoo > 0))
  {
  	// If approach a corner, contributions of x and y should't cancel each other. 
  	// Therefore, turn direction as provided by x is given priority
  	if (turnDir == 0)
  	{
  	  // get bearing of the wall relative to the direction of the nose of the vehicle
  	  float rbrg_yZoo = -(hdg_xZoo);
      FLOAT_ANGLE_NORMALIZE(rbrg_yZoo);

      // determine turn direction that makes the easiest avoidance of the wall
  	  turnDir = -1 * signOfFloat(rbrg_yZoo) * signOfFloat(yZoo);
      VERBOSE_PRINT("rbrg_yZoo = %f, v_yZoo = %f \n", DegOfRad(rbrg_yZoo),v_yZoo);
  	}

  	// compute range to wall and commanded heading change
  	float rng_y = 3.2-fabs(yZoo);
  	dHDGy = turnDir*k_wall*M_PI/(rng_y*rng_y)*(fabs(v_yZoo/sog)+bias);

  	// print result for debug purposes
    VERBOSE_PRINT("xZoo = %f, yZoo = %f, dHDGy = %f deg; rng_y = %f \n", xZoo, yZoo, 57.3*dHDGy, rng_y);
    noprint = false;
  }  

  // include print statement in case no wall is avoided for debug purposes
  if(noprint)
  {
    VERBOSE_PRINT("xZoo, yZoo = %f, %f \n", xZoo, yZoo);
  }

  // return summed heading deviation
  return dHDGx+dHDGy;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  //VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
  return false;
}

/*
 * Computes new coordinates of Goal Waypoint based on current position and commanded heading change
 */
static uint8_t calculateAvoidCoord(struct EnuCoor_i *new_coor, float distMeters, float *angleDiversion)
{
  // get current state
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); 
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  
  // Calculate the new heading the drone is supposed to keep
  float newHeading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi) + *angleDiversion;
  FLOAT_ANGLE_NORMALIZE(newHeading);
  VERBOSE_PRINT("Diversion command: %f degrees; newHeading: %f \n", DegOfRad(*angleDiversion), DegOfRad( newHeading+orient));

  // Calculate the sine and cosine of the heading the drone is to keep
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
 * Calculates coordinates of distance forward with computed angle diversion and sets waypoint 'Waypoint' to those coordinates
 */
uint8_t moveWaypointToAvoid(uint8_t waypoint, float distanceMeters, float *angleDiversion)
{
  struct EnuCoor_i new_coor;
  calculateAvoidCoord(&new_coor, distanceMeters, angleDiversion);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * ================================================
 * == DISUSED FUNCTIONS OF ORIGINAL ORANGE_AVOID ==
 * ================================================
 */

/*
 * DISUSED - Initialisation function, setting the colour filter, random seed and incrementForAvoidance
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


// Testfunction to try out wall avoidance function with preprogrammed obtascles 
 float compute_dHDG_obst()
{
  float dCRSobs = 0;
  // float turnDir;


  // int32_t mbrg = ANGLE_BFP_OF_REAL(atan2f(dx,dy));
  // int32_t vbrg = mbrg-crs;
  // INT32_ANGLE_NORMALIZE(vbrg);

  // float VMG = sog * cosf(ANGLE_FLOAT_OF_BFP(vbrg));
  // if(VMG>0.4*sog) {
  //   turnDir = -1*signOfFloat(vbrg);
  //   dCRSobs =  turnDir * k_obst *M_PI/(rngsq);
  // }
  // else {
  //   dCRSobs = 0;
  //   turnDir=1;
  // }
  // //VERBOSE_PRINT("VMG: %f >  %f, dCRS = %f = %f*%f/%f \n", VMG, 0.4*sog, 57*dCRSobs, turnDir, 0.025*M_PI, rngsq);
  return dCRSobs;
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

/*
 * ==============
 * == \DISUSED ==
 * ==============
 */





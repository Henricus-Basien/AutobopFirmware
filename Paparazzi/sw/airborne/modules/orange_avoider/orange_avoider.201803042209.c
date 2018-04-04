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
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
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

#ifndef ORANGE_AVOIDER_LUM_MIN
#define ORANGE_AVOIDER_LUM_MIN 41
#endif

#ifndef ORANGE_AVOIDER_LUM_MAX
#define ORANGE_AVOIDER_LUM_MAX 183
#endif

#ifndef ORANGE_AVOIDER_CB_MIN
#define ORANGE_AVOIDER_CB_MIN 53
#endif

#ifndef ORANGE_AVOIDER_CB_MAX
#define ORANGE_AVOIDER_CB_MAX 121
#endif

#ifndef ORANGE_AVOIDER_CR_MIN
#define ORANGE_AVOIDER_CR_MIN 134
#endif

#ifndef ORANGE_AVOIDER_CR_MAX
#define ORANGE_AVOIDER_CR_MAX 249
#endif




uint8_t safeToGoForwards        = false;
int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 1.5;
float moveDist          = 1.5;

// initalize state variables
bool states_upd;
float rngsq;
//struct EnuCoor_i bbp;
int32_t crs;
float sog;
float dx;
float dy;
int obs[5];

float getinsideobstaclezone_angle = 20.0;

static inline bool salomon_InsideObstacleZone(float _x, float _y) {
  if (_y <= 0.8) {
    if (_y <= -0.8) {
      if (_y <= -5.5) {
        return FALSE;
      } else {
        float dy = _y - -0.8;
        return (-4.6+dy*-0.685215<= _x && _x <= 5.6+dy*1.453782);
      }
    } else {
      float dy = _y - 0.8;
      return (-5.6+dy*-0.685215<= _x && _x <= 4.6+dy*-0.679873);
    }
  } else {
    if (_y <= 5.5) {
      float dy = _y - 5.5;
      return (1.3+dy*1.451883<= _x && _x <= 1.3+dy*-0.679873);
    } else {
      return FALSE;
    }
  }
}


/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void orange_avoider_init()
{
  // Initialise the variables of the colorfilter to accept orange
  color_lum_min = ORANGE_AVOIDER_LUM_MIN;
  color_lum_max = ORANGE_AVOIDER_LUM_MAX;
  color_cb_min  = ORANGE_AVOIDER_CB_MIN;
  color_cb_max  = ORANGE_AVOIDER_CB_MAX;
  color_cr_min  = ORANGE_AVOIDER_CR_MIN;
  color_cr_max  = ORANGE_AVOIDER_CR_MAX;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  obs[0] = WP_OBS1;
  obs[1] = WP_OBS2;
  obs[2] = WP_OBS3;
  obs[3] = WP_OBS4;
  obs[4] = WP_OBS5;
  VERBOSE_PRINT("Waypoint 1 x: %f, y: %f \n", waypoint_get_x(obs[0]), waypoint_get_y(obs[0]));
}

void orange_avoider_periodic()
{  
  states_upd = false;
  sog           = stateGetHorizontalSpeedNorm_f();  
  struct EnuCoor_i *bbp = stateGetPositionEnu_i(); // Get your current position
  int32_t hdg_chg = 0;
  float hdg_chgf = 0.0;
  
  for (int i = 0; i<5; i++){
    dx = waypoint_get_x(obs[i]) - POS_FLOAT_OF_BFP(bbp->x);
    dy = waypoint_get_y(obs[i]) - POS_FLOAT_OF_BFP(bbp->y);
    rngsq = dx*dx+dy*dy;
    if ((rngsq < (float) 6) && (sog > 0.1)){
      if (states_upd == false){
        crs           = stateGetHorizontalSpeedDir_i();
        states_upd     = true;
        waypoint_set_here_2d(WP_GOAL);
        waypoint_set_here_2d(WP_TRAJECTORY);
      }
      hdg_chgf +=  casper_compt_hdg_chg();
    }
  }
  hdg_chg = ANGLE_BFP_OF_REAL(hdg_chgf);
  INT32_ANGLE_NORMALIZE(hdg_chg);
  VERBOSE_PRINT("Heading change: %f degrees \n", 57.3*ANGLE_FLOAT_OF_BFP(hdg_chg));
  moveWaypointToAvoid(WP_GOAL, moveDist, hdg_chg);
  moveWaypointToAvoid(WP_TRAJECTORY, 1.25 * moveDist, hdg_chg);
  nav_set_heading_towards_waypoint(WP_GOAL);
  return;
}

float casper_compt_hdg_chg()
{
  float dCRSobs;
  float turnDir;

  int32_t mbrg = ANGLE_BFP_OF_REAL(atan2f(dx,dy));
  int32_t vbrg = mbrg-crs;
  INT32_ANGLE_NORMALIZE(vbrg);

  float VMG = sog * cosf(ANGLE_FLOAT_OF_BFP(vbrg));
  if(VMG>0.4*sog) {
    turnDir = (float) -((vbrg > 0) - (vbrg < 0));
    dCRSobs =  turnDir*0.025*M_PI/(rngsq);
  }
  else {
    dCRSobs = 0;
    turnDir = 1;
  }

  VERBOSE_PRINT("VMG: %f >  %f, dCRS = %f = %f*%f/%f \n", VMG, 0.4*sog, dCRSobs, turnDir, 0.025*M_PI, rngsq);

  return dCRSobs;
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
  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
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
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,  
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
                DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)) );
  return false;
}

static uint8_t calculateAvoidCoord(struct EnuCoor_i *new_coor, float distMeters, uint32_t angleDiversion, uint32_t changed_times)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the new heading the drone is supposed to keep
  int32_t newHeading = eulerAngles->psi + angleDiversion;
  INT32_ANGLE_NORMALIZE(newHeading);
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_newheading                 = sinf(ANGLE_FLOAT_OF_BFP(newHeading));
  float cos_newheading                 = cosf(ANGLE_FLOAT_OF_BFP(newHeading));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_newheading * (distMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_newheading * (distMeters));

  // now see if this new position makes sense
  if(!salomon_InsideObstacleZone(POS_FLOAT_OF_BFP(new_coor->x),POS_FLOAT_OF_BFP(new_coor->y)) && changed_times < 7)
  {
    // oops, it isn't... let's see where we need to go now
    float new_anglediversion;
    changed_times++;
    if(changed_times == 1) { // first time
      new_anglediversion = getinsideobstaclezone_angle;
    }
    else {
      new_anglediversion = powf(-1.0, (float) changed_times)*(changed_times*getinsideobstaclezone_angle);
    }
    VERBOSE_PRINT("Changing angle by %f degrees, changed_times = %d; x,y: %f,%f \n", new_anglediversion, changed_times, POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y));
    calculateAvoidCoord(pos, distMeters, ANGLE_BFP_OF_REAL(RadOfDeg(new_anglediversion)), changed_times);

  }
  return false;
}
/*
uint8_t get_target_in_obstaclezone(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}
*/
/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
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

uint8_t moveWaypointToAvoid(uint8_t waypoint, float distanceMeters, uint32_t angleDiversion)
{
  struct EnuCoor_i new_coor;
  calculateAvoidCoord(&new_coor, distanceMeters, angleDiversion, 0);
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
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}


/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;
//extern int allowupdateheading;
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
//extern uint8_t calculateAvoidCoord(struct EnuCoor_i *new_coor, float distMeters, uint32_t angleDiversion);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern uint8_t moveWaypointToAvoid(uint8_t waypoint, float distanceMeters, uint32_t angleDiversion);

extern float casper_compt_hdg_chg(void);
#endif


/* 
Header file for all functions allowing abstraction of communications between 
the strategy-processor and the movement-processor
Those functions are only meant to be used by the strategy processor to give orders to the 
movement processor
 */
#ifndef COMMMVT_H
#define COMMMVT_H
#include "commFrame.h"

void moveXY(const float X, const float Y, commFrame &frame);
void moveRT(const float R, const float theta, commFrame &frame);
void servo(const float angle, const float servo, commFrame &frame);
void stop(commFrame &frame);

void speedConf(const float speed, commFrame &frame);
void accelConf(const float accel, commFrame &frame);

#endif
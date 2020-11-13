#include "commMvt.h"
#include "commKeywords.h"
#include <Arduino.h>

void moveXY(const float X, const float Y, commFrame &frame){
    frame.setId(ID_goTo_XY);
    frame.setValues(0,Y,X);
    sendFrame(frame);
}

void moveRT(const float R, const float theta, commFrame &frame){
    frame.setId(ID_goTo_Rtheta);
    frame.setValues(0,theta,R);
    sendFrame(frame);
}

void servo(const float angle, const float servo, commFrame &frame){
    frame.setId(ID_SERVO);
    frame.setValues(0,angle, servo);
    sendFrame(frame);
}

void stop(commFrame &frame){
    frame.setId(ID_STOP);
    frame.setValues(0,0,0);
    sendFrame(frame);
}

void speedConf(const float speed, commFrame &frame){
    frame.setId(ID_SPEED_CONF);
    frame.setValues(0,0,speed);
    sendFrame(frame);
}

void accelConf(const float accel, commFrame &frame){
    frame.setId(ID_ACCEL_CONF);
    frame.setValues(0,0,accel);
    sendFrame(frame);
}



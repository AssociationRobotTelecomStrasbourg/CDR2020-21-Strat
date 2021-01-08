#include <Arduino.h>
#include "pins_strt.h"
#include "strat.h"

void ledsInit(){
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
}

void ledsWrite(const bool led1, const bool led2){
    digitalWrite(LED1, led1);
    digitalWrite(LED2, led2);
}

void ledsBlinkSeq(){
    ledsWrite(1,1);
    delay(500);
    ledsWrite(0,1);
    delay(500);
    ledsWrite(1,0);
    delay(500);
    ledsWrite(0,0);
}

float readVbatt(uint16_t &V_raw){
    V_raw = analogRead(VBATT);
    return (map(V_raw,0,1023,0,3.3)*4.0)*1.0842; //1.0842 : facteur de calibration
}

void LidarSetSpeed(const uint8_t speed){
    analogWrite(PWM_LIDAR,speed);
}

void LidarSpeedTestSequence(){
    LidarSetSpeed(255);
    delay(700);
    LidarSetSpeed(125);
    delay(700);
    LidarSetSpeed(0);
    delay(700);
}

uint8_t lidarNetworkInit(VL53Network &network, const int timeout, const int interval){
    u_int8_t* list = new u_int8_t[4];
    list[0] = XSHUTa;
    list[1] = XSHUTb;
    list[2] = XSHUTc;
    list[3] = XSHUTd;
    network.setTimeout(timeout);
    network.set_xshut(list);
    network.initNetwork();
    network.startContinuous(interval);
    return network.getN_OK(); //Contient le nombre de capteurs initialises avec succes
}
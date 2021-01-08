/* HEADER FILE - Fonctions propres au processeur strat */
#ifndef STRAT_H
#define STRAT_H

#include <Arduino.h>
#include "VL53Network.h"

void ledsInit();
void ledsWrite(const bool, const bool);
void ledsBlinkSeq();

float readVbatt(uint16_t &V_raw);

void LidarSetSpeed(const uint8_t speed);
void LidarSpeedTestSequence();
uint8_t lidarNetworkInit(VL53Network &network, const int timeout, const int interval);
#endif

//Fichier de configuration des pins pour la Teensy 3.5 Stratégie
#ifndef PINS_STRT_H
#define PINS_STRT_H

//UART 
//  Lidar
#define SerialLidar Serial1
//  IHM
#define SerialIHM Serial3
// Teensy Mouvement
#define Movt Serial2

//XSHUT Lidars fixes
#define XSHUTa 25
#define XSHUTb 26
#define XSHUTc 27
#define XSHUTd 28


#define ReflecSens A9 //Pin analog capteur de reflectance

//Leds
#define LED1 20
#define LED2 21

//PWM LIDAR
#define PWM_LIDAR 14

#define GO_SENSOR A22 //Pin capteur de départ

#define VBATT A21//Tension batterie

#endif
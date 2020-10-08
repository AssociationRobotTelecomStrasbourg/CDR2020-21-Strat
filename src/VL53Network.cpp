#include <VL53L0X.h>
#include "VL53Network.h"

VL53Network::VL53Network(const int N_new){

    N_ok = 0;
    VL53Network::N = N_new;

    VL53L0X* network_new = new VL53L0X[N];
    network = network_new;

    u_int8_t* xshut_list_new = new u_int8_t[N];
    xshut_list = xshut_list_new;
}

VL53Network::VL53Network(const int N_new, VL53L0X* VL53list_new,  u_int8_t* xshutlist_new){
    N = N_new;
    network = VL53list_new;
    xshut_list = xshutlist_new;
}

void VL53Network::addSensor(VL53L0X &sensor,u_int8_t xshut_gpio){
    VL53Network::N = N+1;
    VL53L0X* new_list = new VL53L0X[N];
    u_int8_t* new_list_xshut = new u_int8_t[N];
    for(int i = 0;i<N-1;i++){
        new_list[i] = network[i];
        new_list_xshut[i] = xshut_list[i];
    }
    new_list[N-1] = sensor;
    new_list_xshut[N-1] = xshut_gpio;

    network = new_list;
    xshut_list = new_list_xshut;
}

void VL53Network::initNetwork(){
    //Shutdown all sensors in network
    Serial.println("STARTING INIT...");
    Serial.println(N);
    for(int i = 0;i<N;i++){
        Serial.print("Setting XSHUT gpio ");
        Serial.println(xshut_list[i]);
        pinMode(xshut_list[i],OUTPUT);
        digitalWrite(xshut_list[i],LOW);
    }
    for(int i = 0;i<N;i++){
        digitalWrite(xshut_list[i],HIGH);
        if (!network[i].init())
        {
            Serial.print("Failed to detect and initialize sensor ");
            Serial.println(i);
            Serial.println("Skipping to next sensor");
        }
        else{
            Serial.print("Succeed to detect and initialize sensor ");
            N_ok++;
            Serial.println(i);
        network[i].setAddress(0x50+i);
        Serial.println("Setting ADDR: ");
        Serial.print(0x50+i,HEX);
        Serial.print(network[i].getAddress(),HEX);
        Serial.println(" ");
        network[i].setTimeout(timeout);
        }
    }
}

void VL53Network::setTimeout(int timeout_new){
    timeout = timeout_new;
}

void VL53Network::startContinuous(int interval){
    for(int i = 0;i<N;i++){
        network[i].startContinuous(interval);
    }
}

void VL53Network::set_xshut(u_int8_t *list_new){
    xshut_list = list_new;
}

uint16_t* VL53Network::readRangeContinuousMillimeters(){
    uint16_t* measures = new uint16_t[N];
    for(int i = 0;i<N;i++){
        measures[i] = network[i].readRangeContinuousMillimeters();
    }
    return measures;
}

uint16_t* VL53Network::readRangeSingleMillimeters(){
    uint16_t* measures = new uint16_t[N];
    for(int i = 0;i<N;i++){
        measures[i] = network[i].readRangeSingleMillimeters();
    }
    return measures;
}

uint16_t VL53Network::readRangeContinuousMillimeters(int sensor_nb){
    return network[sensor_nb].readRangeContinuousMillimeters();
}

uint16_t VL53Network::readRangeSingleMillimeters(int sensor_nb){
    return network[sensor_nb].readRangeSingleMillimeters();
}

uint16_t VL53Network::readMinimumRangeSingleMillimeters(int &sensor_min){
    uint16_t* measures = VL53Network::readRangeSingleMillimeters();
    uint16_t minimum = measures[0];
    sensor_min = 0;
    for(int i = 1;i<N;i++){
        if(minimum>measures[i]){
            minimum = measures[i];
            sensor_min = i;
        }
    }
    return minimum;
}

uint8_t VL53Network::getN_OK(){
    return N_ok;
}
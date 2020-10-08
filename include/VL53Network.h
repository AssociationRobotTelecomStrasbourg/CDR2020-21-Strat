#ifndef VL53NETWORK_H
#define VL53NETWORK_H
#include <VL53L0X.h>

#define DEFAULT_TIMEOUT 500

class VL53Network
{
private:
    /* data */
    VL53L0X* network;
    u_int8_t* xshut_list;
    int N;
    int N_ok;
    int timeout = DEFAULT_TIMEOUT;
public:
    
    
    VL53Network(const int N_new);
    VL53Network(const int N_new, VL53L0X* VL53list_new, u_int8_t* xshutlist_new);

    void addSensor(VL53L0X &sensor, u_int8_t xshut_gpio);
    void initNetwork();
    void setTimeout(int timeout_new);
    void set_xshut(u_int8_t *list_new);

    void startContinuous(int interval);

    uint16_t* readRangeContinuousMillimeters(); //returns N-array of continuous measures from all sensors in network 
    uint16_t* readRangeSingleMillimeters(); //returns N-array of single measures from all sensors in network 
    uint16_t readRangeContinuousMillimeters(int sensor_nb); //returns continous measures from specified sensor
    uint16_t readRangeSingleMillimeters(int sensor_nb); //returns single measures from specified sensor

    uint16_t readMinimumRangeSingleMillimeters(int &sensor_min);

    uint8_t getN_OK();
};



#endif
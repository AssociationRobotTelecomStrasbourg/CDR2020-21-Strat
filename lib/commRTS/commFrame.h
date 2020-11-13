#ifndef COMMFRAME_H
#define COMMFRAME_H
#include <Arduino.h>

class commFrame //Class allowing easier handling of communication frames 
{
private:
    uint8_t _framestarter;
    uint8_t _id;
    float _value2,_value1,_value0;

public:
    commFrame(const uint8_t framestarter);

    void setStarter(const uint8_t framestarter); //allows to change framestarter value
    void setId(const uint8_t id); //allows to write new id value
    void setValues(const float value2, const float value1, const float value0); //allows to write new data values

    uint8_t getStarter();
    uint8_t getId();
    void getValues(float &value2, float &value1, float &value0);
};

void divideFloat(const float value, uint8_t &byte3, uint8_t &byte2, uint8_t &byte1, uint8_t &byte0); //divides 32bits float into 4*8bits unsigned int
void formFloat(float &value, const uint8_t &byte3, const uint8_t &byte2, const uint8_t &byte1, const uint8_t &byte0); //recreates 32bits float with 4*8bits unsigned int
void sendFrame(commFrame frame);
void readFrame(uint8_t &ID, float &value2, float &value1, float &value0);
void readFrame(commFrame &frame);
#endif
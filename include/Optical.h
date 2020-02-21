#pragma once
#ifndef OPTICAL_H
#define OPTICAL_H

#define MBED

#ifdef MBED
#include <mbed.h>
typedef PinName m_PinName;
#endif

#ifdef ARDUINO
#include<Arduino.h>
typedef uint8_t m_PinName;
#endif

#define SHIFT_SPEED 2

enum OpticalSensErrorState {
    OKAY,
    COMMUNICATION_ERROR,
    OVERFLOW_ERROR
};

class OpticalSens
{
private:
    DigitalOut clock;
    DigitalInOut data;   
    int dt_x = 0;
    int dt_y = 0;
    int res; // resolutiun in dpi
    OpticalSensErrorState errorState = OKAY;

public:
    //Basic comunication
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t data);

    void setup(); // setup an config the sensor;
    void update(); // get position updates from the sensor

    void get_raw_xy(int &x, int &y); // get accumulated raw values
    void get_mm_xy(float &x, float & y); // get accumulated values in mm
    void reset();  // reset the acummulator;

    int image_quality(); // gets the current image qualitiy (bad)0-255(good)
    

    //FrameWork Dependent:
    OpticalSens(m_PinName clk, m_PinName sdio);
private:
    void writeByte(uint8_t val);
    uint8_t readByte();
    void resync();
};



#endif // OPTICAL_H
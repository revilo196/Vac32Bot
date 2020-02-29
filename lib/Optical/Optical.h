#pragma once
#ifndef OPTICAL_H
#define OPTICAL_H

#include <mbed.h>

//delay in us beween clock cycles
#define SHIFT_SPEED 2

enum OpticalSensErrorState {
    OKAY,
    COMMUNICATION_ERROR,
    OVERFLOW_ERROR
};

/**
 * @brief object for comunicating with a Pix_Art optical tracking sensor
 * PMW3605
 */
class OpticalSens
{
private:
    DigitalOut clock;
    DigitalInOut data;   
    int dt_x = 0;
    int dt_y = 0;
    int res; // resolutiun in dpi
    OpticalSensErrorState errorState = OKAY;
    uint64_t last_time_X;
    uint64_t last_time_Y;
    float radius;
    float angular_velocity;
    float velocity;
    float angle_estimate;

public:
    //Basic comunication
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t data);

    void setup(); // setup an config the sensor;
    void update(); // get position updates from the sensor

    float get_yaw_estimate(){return angle_estimate;}
    void get_raw_xy(int &x, int &y); // get accumulated raw values
    void get_mm_xy(float &x, float & y); // get accumulated values in mm
    void reset();  // reset the acummulator;
    inline float get_angular_velocity() {return angular_velocity;}
    inline float get_velocity(){return velocity;}
    int image_quality(); // gets the current image qualitiy (bad)0-255(good)
    

    OpticalSens(PinName clk, PinName sdio, float radius);
private:
    void writeByte(uint8_t val);
    uint8_t readByte();
    void resync();
};



#endif // OPTICAL_H
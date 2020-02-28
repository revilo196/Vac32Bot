#pragma once
#ifndef SONAR_H
#define SONAR_H
#include <mbed.h>

#define SEN_CNT 3
#define RATE 50000 //50ms
#define MAP_SIZE 4096
#define DEBUG_SONAR

class CompactBufferLogger;

class Sonar
{
private:
    CompactBufferLogger * logger;
    Serial * s;
    I2C * c;
    uint32_t raw_values[SEN_CNT];
    float sensor_angle[SEN_CNT];
    volatile uint32_t transfer_buffer[SEN_CNT];
    volatile bool i2cTransfer;
    volatile bool updated;
    uint64_t last_time;
    volatile bool value_changed[SEN_CNT];
    void i2cTransferCallback(int event);
    void i2cUpdateRequest();
    int16_t map_x[MAP_SIZE];
    int16_t map_y[MAP_SIZE];
    uint16_t map_pos;
    

public:
    Sonar(Serial * ser, I2C * i2c, CompactBufferLogger * logger);
    ~Sonar();
    void update();
    void updateMap(int16_t g_x, int16_t g_y, float g_yaw);
};


#endif //SONAR_
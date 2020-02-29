#pragma once
#ifndef IMU_H
#define IMU_H
 
#include "mbed.h"
#include "MPU9250.h"
#include "logger.h"

#define PI 3.14159265358979323846

class IMU
{
private:
    CompactBufferLogger* logger;
    Serial * pc;
    MPU9250 mpu;
    void updateRotation();
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
    float q[4];
    float delta_v[3];
    float delta_p[3];
    float deltat;
    float beta; 
    float zeta;
    
    float last_acc[3];
    float jerk[3];
    float jerk_sq[3];
    float l_pitch, l_yaw, l_roll;
    float pitch, yaw, roll;
    float d_pitch, d_yaw, d_roll;
    unsigned long  last_time;
    
    int16_t accData[3];
    int16_t gyroData[3];
    float accUserBias_PREC[3];
    float gyrUserBias_PREC[3];
    int16_t accUserBias[3];
    int16_t gyrUserBias[3];
    float accStddevData[3];
    float gyrStddevData[3];
    int covariance[6][6];
    float corelation[6][6];

    bool bump_flag = false;

public:
    
    IMU(Serial * console, CompactBufferLogger* logger);
    ~IMU();
    
    int setup();
    int update();
    int adv_calibration();
    int re_calibrate();
    inline float getPitch() const {return pitch;}
    inline float getYaw() const {return yaw;}
    inline float getRoll() const {return roll;}
    inline float getGyroZ() const {return ((gyroData[2] - gyrUserBias_PREC[2] )*mpu.gRes /  180.0 * PI);}

    inline float getDeltaPitch() const {return d_pitch;}
    inline float getDeltaYaw() const {return d_yaw;}
    inline float getDeltaRoll() const {return d_roll;}

    inline void resetDelta() {d_pitch=0.0f; d_yaw=0.0f; d_roll=0.0f;}

    inline bool isBump() const {return bump_flag;}
    inline void resetBump() {bump_flag = false;}
};




#endif //IMU_H
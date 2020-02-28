#ifndef DRIVE_H
#define DRIVE_H

#define ENCODER_RES 40
#define PI 3.14159265358979323846

#include <mbed.h>
#include "logger.h"
#include "odometry.h"
#include "INA219.hpp"


#define DRIVE_DESTINATION_REACHED_EVENT 1
#define DRIVE_DESTINATION_ERROR_EVENT 2
#define DRIVE_OVERCURREN_EVENT 4
#define DEBUG_DRIVE

#define PID_Kp 1.0
#define PID_Ki 0.0
#define PID_Kd 0.0
#define RATE 0.01

enum TargetState {
    POSITION,
    ROTATION,
    NONE,
};

class Drive
{
private:
    PwmOut mR1;
    PwmOut mR2;
    PwmOut mL1;
    PwmOut mL2;

    INA219 ina219_R;
    INA219 ina219_L;

    InterruptIn encR;
    InterruptIn encL;

    int counter_R;
    int counter_L;
    float speed_R = 0.0;
    float speed_L = 0.0; 
    float angular_velocity;
    float velocity = 0.0;
    unsigned long last_time_R = 0;
    unsigned long last_time_L = 0;

    float current_R;
    float current_L;

    int dir_R;
    int dir_L;

    int m_cal_cnt;
    float estimated_x;
    float estimated_y;
    float estimated_yaw;
    float estimated_vel;
    float estimated_rot_v;
    unsigned long last_time;
    float target_x;
    float target_y;
    float target_yaw;
    float error_integral;
    float last_err;
    TargetState state;
    event_callback_t destinationCallback;

    Odometry mOdometry;
    float wheel_diameter;
    float wheel_base;

    inline void rightWheelForward(float f) {mR1 = 0; mR2 = f; dir_R = 1;};
    inline void leftWheelForward(float f) {mL1 = 0; mL2 = f; dir_L = 1;};
    inline void rightWheel(float f) {
        if (f > 0) {
            mR1 = 0; mR2 = f; dir_R = 1;
        } else {
            mR1 = -f; mR2 = 0; dir_R = -1;
        }
    }
    inline void leftWheel(float f) {
        if (f > 0) {
            mL1 = 0; mL2 = f; dir_L = 1;;
        } else {
            mL1 = -f; mL2 = 0; dir_L = -1;
        }
    }
    inline void rightWheelBackward(float f) {mR1 = f; mR2 = 0; dir_R = -1;};
    inline void leftWheelBackward(float f) {mL1 = f; mL2 = 0; dir_L = -1;};
    inline void rightWheelStop() {mR1 = 0; mR2 = 0; dir_R = 0;};
    inline void leftWheelStop() {mL1 = 0; mL2 = 0;dir_L = 0; };
    void setForward(float f = 1.0f);
    void setBackward(float f = 1.0f);
    void setStop();
    void setRotateLeft(float f = 1.0f);
    void setRotateRight(float f = 1.0f);
    CompactBufferLogger* logger;

public:
    void IR_encoderLeftSide();
    void IR_encoderRightSide();
    Drive(PinName pin_mL1, PinName pin_mL2, PinName pin_mR1, PinName pin_mR2, PinName pin_encL, PinName pin_encR, 
          float wheel_base, float wheel_diameter, CompactBufferLogger* _logger);
    ~Drive();
    void setDestinationCallback(event_callback_t call) {this-> destinationCallback = call;}
    void setDestination(float x_rel, float y_rel);
    void setDestinationPolar(float alpha, float radius);
    void setDestinationRel(float x_rel, float y_rel);
    void setDestinationRotation(float yaw);
    const Odometry * getOdometry() const {return &mOdometry;}; 
    void update(float x, float y, float yaw){} // TODO Update function without self calculated odometry
    void update(); //TODO Change calculate odometry and then use to use  ^^  update(float x, float y, float yaw)
    void setup();
    void getEstimated(float & g_x,float & g_y,float & g_yaw) {g_x =  estimated_x; g_y = estimated_y; g_yaw = estimated_yaw;};
};


#endif // DRIVE_H
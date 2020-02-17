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

enum TargetState {
    POSITION,
    ROTATION,
    NONE,
};

class Drive
{
private:
    DigitalOut mR1;
    DigitalOut mR2;
    DigitalOut mL1;
    DigitalOut mL2;

    INA219 ina219_R;
    INA219 ina219_L;

    InterruptIn encR;
    InterruptIn encL;

    int counter_R;
    int counter_L;

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
    TargetState state;
    event_callback_t destinationCallback;

    Odometry mOdometry;
    float wheel_diameter;
    float wheel_base;

    inline void rightWheelForward() {mR1 = 0; mR2 = 1; dir_R = 1;};
    inline void leftWheelForward() {mL1 = 0; mL2 = 1; dir_L = 1;};
    inline void rightWheelBackward() {mR1 = 1; mR2 = 0; dir_R = -1;};
    inline void leftWheelBackward() {mL1 = 1; mL2 = 0; dir_L = -1;};
    inline void rightWheelStop() {mR1 = 0; mR2 = 0; dir_R = 0;};
    inline void leftWheelStop() {mL1 = 0; mL2 = 0;dir_L = 0; };
    void setForward();
    void setBackward();
    void setStop();
    void setRotateLeft();
    void setRotateRight();
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
    void update();
    void setup();
};


#endif // DRIVE_H
#pragma once
#ifndef ODOMETRY_H
#define ODOMETRY_H

void odometry_function(float w1_travel, float w2_travel,float wb, float * dx, float * dy, float * d_yaw);


float normalizeRadiansPiToMinusPi(float rad);

class Odometry
{
private:
    float wheel_base;  //[mm]

    float x;  //[mm]
    float y;  //[mm]
    float yaw; //[rad]

    unsigned long last_time;   //[us]

    float avg_acceleration; //[mm/s^2]
    float avg_speed; //[mm/s]
    float speed_x; //[mm/s]
    float speed_y; //[mm/s]
    float speed_rotation;  //[rads/s]

public:
    Odometry(float wheel_base);
    ~Odometry();
    void move(float w1l, float w2r, unsigned long current_time_us);
    inline void getPosition(float &rx, float &ry, float &ryaw) const {rx = x; ry = y; ryaw = yaw;}
    inline void reset(float &rx, float &ry, float &ryaw) {x = 0; y = 0; yaw = 0;}
    inline float getAvgSpeed() const {return avg_speed;}
    inline float getAvgAcceleration() const {return avg_acceleration;}
    inline float getRotationSpeed() const {return speed_rotation;}

};


#endif
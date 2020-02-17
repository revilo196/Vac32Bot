#include <cmath>
#include "odometry.h"


// Bring the 'difference' between two angles into [-pi; pi].

float normalizeRadiansPiToMinusPi(float rad) {
  // Copy the sign of the value in radians to the value of pi.
  float signed_pi = std::copysign(M_PI,rad);
  // Set the value of difference to the appropriate signed value between pi and -pi.
  rad = std::fmod(rad + signed_pi,(2 * M_PI)) - signed_pi;
  return rad;
}

const float beta = 0.5;

/**
 * odometry_function
 * 
 * the x axis is the driving directon of the vehicle
 *  
 * :param w1_travel, distance traveld of the left wheel
 * :param w2_travel, distance traveld of the right wheel 
 * :param wb,  Wheel base distance
 * 
 * :return dx delta traveld in x direction
 * 
 * :return dy delta traveld in y direction
 * :return d_yaw delta rotation
 * */
void odometry_function(float w1_travel, float w2_travel, float wb,  float * dx, float * dy, float * d_yaw)  {
    
    float alpha = (w2_travel - w1_travel) / wb;

    if (alpha > 0.0000000001 || alpha < -0.0000000001) {
        float r1 = w1_travel / alpha;
        float r2 = w2_travel / alpha;
        float r_mid = (r1 + r2) / 2;
        *dx = r_mid * (sin(alpha));  
        *dy = -(r_mid * (cos(alpha) - 1));
        *d_yaw = alpha;
    } else {
        *dx =  (w1_travel + w2_travel) / 2;
        *dy = 0.0;
        *d_yaw = 0.0;
    }
}


Odometry::Odometry(float wheel_base_distance)
{
    wheel_base = wheel_base_distance;
    x = 0;
    y = 0;
    yaw= 0;

    last_time = 0;

    avg_acceleration = 0;
    avg_speed = 0;
    speed_x = 0;
    speed_y = 0;
    speed_rotation = 0;
}

Odometry::~Odometry()
{
}

void Odometry::move(float w1_travel, float w2_travel,unsigned long current_time_us) {
    float dx, dy, d_yaw;
    odometry_function(w1_travel, w2_travel, wheel_base, &dx, &dy, &d_yaw);

    //rotate around the current rotation 
    float dr_x = dx * cos(yaw) - dy * sin(yaw);
    float dr_y = dx * sin(yaw) + dy * cos(yaw);

    x += dr_x;  //[mm]
    y += dr_y;  //[mm]
    yaw += d_yaw;  //[rad]
    yaw = normalizeRadiansPiToMinusPi(yaw);
    float deltat = (current_time_us - last_time) / 1e+6; // [s]
    last_time = current_time_us;

    speed_x = dx / deltat;  // [mm/s]
    speed_y = dy / deltat;  // [mm/s]

    //fist calculate and lowpass filfer the speed
    float new_speed_rotation = d_yaw / deltat;  //[rad/s]
    float new_speed =  ((w1_travel + w2_travel) / 2.0) / deltat;
    speed_rotation = new_speed_rotation * beta + (speed_rotation*(1.0-beta));
    new_speed = new_speed * beta + (avg_speed*(1.0-beta)); // [mm/s]

    //then caclulate and low pas filter the acceleration
    float new_avg_acceleration = (new_speed - avg_speed) / deltat; 
    avg_acceleration = new_avg_acceleration * beta + (avg_acceleration*(1.0-beta));
    avg_speed = new_speed;
}
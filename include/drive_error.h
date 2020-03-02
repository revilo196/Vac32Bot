#pragma onec
#ifndef DRIVE_ERROR_H
#define DRIVE_ERROR_H

#include <drive.h>
#include <Optical.h>
#include <logger.h>
#include <mbed.h>


class DriveError
{
private:
    const Drive * drive;
    const OpticalSens * opti;
    CompactBufferLogger * logger;
    event_callback_t driveErrorCallback;
    double last_error;
public:
    DriveError(const Drive * dr, const OpticalSens * op, CompactBufferLogger * logger);
    void setDriveErrorCallback( event_callback_t call) {driveErrorCallback = call;};
    void update();
};

DriveError::DriveError(const Drive * dr, const OpticalSens * op, CompactBufferLogger * log)
{
    drive = dr;
    opti = op;
    logger = log;
    last_error = 0;
}


void DriveError::update() {
    double n_diff_a = (drive->getRotateVelocity() - opti->get_angular_velocity());
    double n_diff_v = (drive->getVelocity() - opti->get_velocity()) / 100; 
    double sq_err = n_diff_a*n_diff_a +  n_diff_v*n_diff_v;

    last_error =  last_error*0.99 + sq_err*0.01;

    if(last_error > 0.58) {
         if(driveErrorCallback) {
             driveErrorCallback(1);
         }
    } 
    
    logger->begin("err", 8);
    logger->log("dav", drive->getRotateVelocity());
    logger->log("oav", opti->get_angular_velocity());
    logger->log("dvv", drive->getVelocity());
    logger->log("ovv", opti->get_velocity());
    logger->log("da", (float)n_diff_a);
    logger->log("dv", (float)n_diff_v);
    logger->log("se", (float)sq_err);
    logger->log("le", (float)last_error);
    logger->submit();

}


#endif
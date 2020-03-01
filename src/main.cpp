#include <mbed.h>
#define M_PI 3.14159265358979323846
#include <cmath>
#include "IMU.h"
#include "drive.h"
#include "logger.h"
#include "Optical.h"
#include "sonar.h"


Serial pc(USBTX, USBRX, 2000000); //tx, rx


DigitalOut led1(LED1);
PwmOut led2(PB_1_ALT0);
PwmOut led3(PB_2);


OpticalSens sens(PB_6, PB_5,101.0f);

DigitalOut br1(PC_0);
DigitalOut br2(PC_1);

Serial sonar_ser(PC_10,PC_5,9600);
I2C i2c_sonar(PC_12,PB_10);




CompactBufferLogger logger(&pc);

Drive drive(PA_10,PA_11,PA_8,PA_9,PC_2,PC_3,235,52.5, &logger);
IMU imu(&pc, &logger);
Sonar sonar(&sonar_ser,&i2c_sonar,&logger);


int programm_cnt = 0;

void drive_programm_callback(int event ) {
  if(event == DRIVE_DESTINATION_REACHED_EVENT) {

    programm_cnt++;
    int programm = (programm_cnt + 1) % 7;
    switch (programm)
    {
    case 0:
        drive.setDestination(0,0);
      break;

    case 1:
      	drive.setDestination(400,0);
      break;
    case 2:
        drive.setDestinationRotation(PI/2.0);
    break;

    case 3:
        drive.setDestination(400,400);
      break;
    case 4:
        drive.setDestinationRotation(PI);
      break;

    case 5:
        drive.setDestination(0,400);
      break;
    case 6:
        drive.setDestinationRotation(PI+ (PI/2.0));
      break;
    
    default:
        drive.setDestination(0,0);
      break;
    }


  }

}

void sonarError(SonarEvent s) {
    programm_cnt = 999;
   drive.setStop();
}

void driveError(int event) {
  programm_cnt = 999;
  drive.setStop();
}

int main(){

    imu.setup();
    drive.setup();
    sens.setup();
    drive.setDestinationCallback(&drive_programm_callback);
    drive.setNavigationInterrupt(&driveError);
    sonar.setSonarCallback(&sonarError);
    HAL_Delay(1000);


    int counter = 0 ;

    HAL_Delay(5000);
    imu.adv_calibration();
    HAL_Delay(1000);

    drive.adv_calibration();
    led2 = 0.5;

    HAL_Delay(1000);

    drive.setDestination(0,0);


    imu.resetDelta();
    while (programm_cnt <= 13 )
    {
        counter++;    


        //int err = i2c_sonar.read(0x2a,(char*) dist, sizeof(uint32_t)*3);
        //look at transferfor non blocking i2c
        led2 = fabs(sin(counter/20.0))/4.0; 
        led3 = fabs(cos(counter/20.0))/4.0; 

        //pc.printf("err: %i  1:  %ld  \t 2:  %ld  \t  3:  %ld mm \n" ,err,  dist[0], dist[1], dist[2]);
        
        imu.update(); //should be run at least every ~6ms
        drive.update(); // should be run ~20ms
        sens.update();
        sonar.update();

        float x,y, yaw;
        float yaw2 = imu.getDeltaYaw();
        drive.getEstimated(x,y,yaw);
        sonar.updateMap(x,y,(yaw2/180.0)*M_PI);
        


        logger.begin("sen",6);
        logger.log("imu",(float)((yaw2/180.0)*M_PI));
        logger.log("dri",yaw);
        logger.log("opt", sens.get_yaw_estimate());
        logger.log("vimu", imu.getGyroZ());
        logger.log("vdir", drive.getRotateVelocity());
        logger.log("vopt", sens.get_angular_velocity());
        logger.submit();

        /*
        int sx,sy;
        sens.get_raw_xy(sx,sy);
        logger.begin("sen",4);
        logger.log("x",(int16_t)sx);
        logger.log("y",(int16_t)sy);
        logger.log("vel",sens.get_velocity());
        logger.log("avel",sens.get_angular_velocity());
        logger.submit();*/

        

    }
  drive.setStop();
  for (int i = 0; i < 128; i++) {
    logger.begin("logger",1);
    logger.log("wait_for_flush_counter1", (int64_t)logger.wait_for_flush_counter);
    logger.submit();
  }
  while (1){}
  
}

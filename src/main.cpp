#include <mbed.h>
#define M_PI 3.14159265358979323846
#include <cmath>
#include "IMU.h"
#include "drive.h"
#include "logger.h"
#include "Optical.h"
//#include "INA219.hpp"



Serial pc(USBTX, USBRX, 2000000); //tx, rx
Serial sonar(PC_10,PC_5,9600);

DigitalOut led1(LED1);
PwmOut led2(PB_1_ALT0);
PwmOut led3(PB_2);

//DigitalOut sens_clk(PB_6); 
//DigitalInOut sens_sdio(PB_5);

OpticalSens sens(PB_6, PB_5,101.0f);

DigitalOut br1(PC_0);
DigitalOut br2(PC_1);

I2C i2c_sonar(PC_12,PB_10);

CompactBufferLogger logger(&pc);

Drive drive(PA_10,PA_11,PA_8,PA_9,PC_2,PC_3,235,52.5, &logger);
IMU imu(&pc, &logger);


volatile bool i2cTransfer = false;
void respondedCallback( int event ) {
    i2cTransfer = false;
}

int programm_cnt = 0;

void drive_programm_callback(int event ) {
  if(event == DRIVE_DESTINATION_REACHED_EVENT) {
    
    programm_cnt = (programm_cnt + 1) % 9; 
    switch (programm_cnt)
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


int main(){

    imu.setup();
    drive.setup();
    sens.setup();

    HAL_Delay(1000);
    uint32_t dist[3];

    int counter = 0 ;

    dist[0] = 0;
    dist[1] = 0; 
    dist[2] = 0;

    HAL_Delay(5000);
    imu.adv_calibration();
    HAL_Delay(1000);
    

    while (programm_cnt <= 7 )
    {
        counter++;    

        if(i2cTransfer == false) {
          //should be run every ~50ms? runs for 10ms
          logger.begin("sonar",3);
          logger.log("x",dist[0]);
          logger.log("y",dist[1]);
          logger.log("z",dist[2]);
          logger.submit();
          const char* cmd = "\x00";
          i2cTransfer = true;
          i2c_sonar.transfer(0x2a, cmd, 1, (char*)dist , sizeof(uint32_t) * 3, callback(respondedCallback), I2C_EVENT_ALL);
        }
        //int err = i2c_sonar.read(0x2a,(char*) dist, sizeof(uint32_t)*3);
        //look at transferfor non blocking i2c
        led2 = fabs(sin(counter/20.0))/4.0; 
        led3 = fabs(cos(counter/20.0))/4.0; 

        //pc.printf("err: %i  1:  %ld  \t 2:  %ld  \t  3:  %ld mm \n" ,err,  dist[0], dist[1], dist[2]);
        
        imu.update(); //should be run at least every ~6ms
        drive.update(); // should be run ~20ms
        sens.update();
        int sx,sy;
        sens.get_raw_xy(sx,sy);

        logger.begin("sen",4);
        logger.log("x",(int16_t)sx);
        logger.log("y",(int16_t)sy);
        logger.log("vel",sens.get_velocity());
        logger.log("avel",sens.get_angular_velocity());
        logger.submit();
    }

  for (int i = 0; i < 64; i++) {
    logger.begin("logger",1);
    logger.log("wait_for_flush_counter1", (int64_t)logger.wait_for_flush_counter);
    logger.submit();
  }
  while (1){}
  
}

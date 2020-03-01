
#include "sonar.h"
#include "math.h"
#include "logger.h"

#define M_PI 3.14159265358979323846

Sonar::Sonar(Serial * ser, I2C * i2c, CompactBufferLogger * l)
{
    this->s = ser;
    this->c = i2c;
    this->logger = l;

    i2cTransfer = false;
    updated = false;
    last_time = 0;
    map_pos = 0;

    sensor_angle[0] = 0;
    sensor_angle[1] =  -M_PI/4;
    sensor_angle[2] =   M_PI/4;

    for(int i = 0; i < SEN_CNT; i++) {
        raw_values[i] = 0;
        transfer_buffer[i] = 0;
        obsticalCount[i] = 0;
    }
}

Sonar::~Sonar()
{
}


void Sonar::i2cTransferCallback(int event) {
    switch (event)
    {
    case I2C_EVENT_TRANSFER_COMPLETE:
        this->i2cTransfer = false;
        
        for(int i = 0; i < SEN_CNT; i++) {
            if(raw_values[i]/1000 != transfer_buffer[i]/1000) {
                raw_values[i] = transfer_buffer[i];
                value_changed[i] = true;
                updated = true;
            }
        }
        break;
    
    default:
        break;
    }
}



void Sonar::i2cUpdateRequest(){
    const char* cmd = "\x00";
    i2cTransfer = true;
    c->transfer(0x2a, cmd, 1, (char*)transfer_buffer , sizeof(uint32_t) * 3, 
                            callback(this,&Sonar::i2cTransferCallback), I2C_EVENT_ALL);
        
}



void Sonar::update(){

    unsigned long current_time = us_ticker_read();
    if (!i2cTransfer && current_time - last_time > SON_RATE) {
        last_time = current_time; 
        i2cUpdateRequest();
    }

    for(int i = 0; i < SEN_CNT; i++) {
        if (value_changed[i]) {
            if (raw_values[i]/1000.0f < 120) /*100mm*/   {
                obsticalCount[i]++;
                if (obsticalCount[i] > 3) { // only emit signal if the obstical was recoginzed 4 times in a row
                    if(sonarEventCalback) {
                        SonarEvent s = {sensor_angle[i], raw_values[i] / 1000};
                        sonarEventCalback(s);
                    }
                }

            } else {
                obsticalCount[i] = 0;
            }
        }
    }

}


void Sonar::updateMap(int16_t g_x, int16_t g_y, float g_yaw){ 
    for(int i = 0; i < SEN_CNT; i++) {
        if (value_changed[i]) {
            float sen_yaw = g_yaw + sensor_angle[i];
            int16_t x = g_x + ((cos(sen_yaw) * (raw_values[i])/1000.0f));
            int16_t y = g_y + ((sin(sen_yaw) * (raw_values[i])/1000.0f));

            map_x[map_pos] = x;
            map_y[map_pos] = y;
            map_pos = (map_pos + 1) % MAP_SIZE;
            value_changed[i] = false;
            #ifdef DEBUG_SONAR
                logger->begin("SO",7);
                logger->log("s", (uint16_t)i);
                logger->log("x", (int16_t)x);
                logger->log("y", (int16_t)y);
                logger->log("gx", (int16_t)g_x);
                logger->log("gy", (int16_t)g_y);
                logger->log("gyaw", g_yaw);
                logger->log("raw", (uint16_t)(raw_values[i]/1000));
                logger->submit();
            #endif
        }
    }
}

#include "Optical.h"

    #define HIGH 1
    #define LOW 0

    #define INPUT 1
    #define OUTPUT 0

    /** Arduino emulation function */
    inline void delayMicroseconds(uint32_t us) {
        wait_us(us);
    }
    /** Arduino emulation function */
    inline void delay(uint32_t ms) {
        wait_us(ms * 1000);
    }
    /** Arduino emulation function */
    inline void digitalWrite(DigitalOut out, int val) {
        out = val;
    }
    /** Arduino emulation function  */
    inline void digitalWrite(DigitalInOut out, int val) {
        out.write(val);
    }
    /** Arduino emulation function */
    inline void pinMode(DigitalInOut pin, int mode) {
        if(mode == OUTPUT) {
            pin.output();
        } else {
            pin.input();
        }
    }
    /** Arduino emulation function */
    inline int digitalRead(DigitalInOut pin) {
        return pin.read();
    }


    /**
     * @brief Construct a new Optical Sens:: Optical Sens object
     * 
     * @param clk clock Pin
     * @param sdio serial data pin 
     */
    OpticalSens::OpticalSens(PinName clk, PinName sdio, float radius) : clock(clk), data(sdio)  {
        this->res = 1;
        this->radius = radius;
        last_time_X = 0;
        last_time_Y = 0;

    }

    /**
     * @brief shift out a byte of data to the Optical sensor
     * this action takes minimal SHIFT_SPEED*19us
     * @param val the byte that should be tranmitted
     */
    void OpticalSens::writeByte(uint8_t val){
        //set the pin to output
        pinMode(data,OUTPUT);
        //wait for the pin to settle
        delayMicroseconds(SHIFT_SPEED*3);
    
        uint8_t i;
        for (i = 0; i < 8; i++)  {
            digitalWrite(data, !!(val & (1 << (7 - i))));
            digitalWrite(clock, LOW);
            delayMicroseconds(SHIFT_SPEED);
            digitalWrite(clock, HIGH);
            delayMicroseconds(SHIFT_SPEED);
	    }
    }

    /**
     * @brief read one serial byte from SPI-like comunication
     * this action takes minimal SHIFT_SPEED*19us
     * @return uint8_t the recived byte
     */
    uint8_t OpticalSens::readByte(){
        //set the pin low before switching to output
        digitalWrite(data,LOW);
        pinMode(data,INPUT);
        //wait for the pin to settle
        delayMicroseconds(SHIFT_SPEED*3);


        uint8_t value = 0;
        uint8_t i;
        for (i = 0; i < 8; ++i) {
            digitalWrite(clock, LOW);
            delayMicroseconds(SHIFT_SPEED/2);
            value |= digitalRead(data) << (7 - i);
            delayMicroseconds(SHIFT_SPEED/2);
            digitalWrite(clock, HIGH);
            delayMicroseconds(SHIFT_SPEED);
        }
        return value;
    }

    /**
     * @brief resync the sensor comunication
     * EXPERIMANTAL
     * BLOCKS FOR 3.5ms!
     */
    void OpticalSens::resync(){
        delayMicroseconds(500);
        pinMode(data,OUTPUT);
        digitalWrite(data,LOW);
        digitalWrite(clock,LOW);
        delayMicroseconds(5);
        digitalWrite(clock,HIGH);
        delay(3);
    }

    /**
     * @brief read a register of the sensor
     * this action takes minimal SHIFT_SPEED*38us
     * @param reg addres of the register that should be read
     * @return uint8_t the returned value of the register
     */
    uint8_t OpticalSens::readRegister(uint8_t reg){
        reg = reg & 0x7F; //bitmask for write bit 8 has to be 0
        this->writeByte(reg);
        return readByte();
    }

    /**
     * @brief write to a register of the sensor
     * this action takes minimal SHIFT_SPEED*38us
     * @param reg  address of the register to write to 
     * @param data value to write in the register
     */
    void OpticalSens::writeRegister(uint8_t reg, uint8_t data){
        reg = reg | 0x80; //bitmask for read bit 8 has to be 1
        this->writeByte(reg);
        this->writeByte(data);
    }

    /**
     * @brief setup and configure the sensor
     */
    void OpticalSens::setup(){
        //pinMode(clock, OUTPUT);
        //wait for sensor
        for (int i = 0;i < 100; i++) {
            delay(10);
            uint8_t id = readRegister(0x00);
            writeRegister(0x05, 0x00); // set no sleep
            if(id == 0x31) break;
            delay(10);
        }
    } 

    /**
     * @brief read updates from the sensor
     */
    void OpticalSens::update(){
        unsigned long current_time = us_ticker_read();
        float deltat_X = (current_time - last_time_X) / 1e+6; // [s]
        float deltat_Y = (current_time - last_time_Y) / 1e+6; // [s]
        if (deltat_X > 0.005) {velocity = 0; last_time_X = current_time;}
        if (deltat_Y > 0.005) {angular_velocity = 0; last_time_Y = current_time;}

        uint8_t inByte[4];
        inByte[0] = readRegister(0x02); // read if a change occured
                                        // 0bMXXO ORRR  -> M change
                                        // X -> Reserved
                                        // O_1 -> Y-Overflow
                                        // O_2 -> X-Overflow
                                        // RRR -> Resulution 

        if ((inByte[0] & 0x80) > 0) {

            //read the 
            uint8_t b_res = inByte[0] & 0b00000111;
            switch (b_res)
            {
            case 0:
                res = 600;
                break;
            case 1:
                res = 600;
                break;
            case 2:
                res = 725;
                break;
            case 3:
                res = 800;
                break;
            case 4:
                res = 1000;
                break;
            case 5:
                res = 1200;
                break;
            case 6:
                res = 1200;
                break;
            default:
                res = 1000;
                break;
            }

            inByte[1] = readRegister(0x03); // deltaX int8 -128 -- 127
            inByte[2] = readRegister(0x04); // deltaY int8 -128 -- 127
            inByte[3] = readRegister(0x00); // check id == 31 allways

            if (inByte[3] == 0x31) {

                if(inByte[1] > 0) {
                    this->dt_x += (int8_t)inByte[1];
                    float _velocity = -(((int8_t)inByte[1] / (float)res) * 25.4f) / deltat_X;
                    this->velocity = (velocity*0.25f) + (_velocity * 0.75f);
                    last_time_X = current_time;
                }
                if(inByte[2] > 0) {
                    this->dt_y += (int8_t)inByte[2];
                    float _angular_velocity = ((((int8_t)inByte[2] / (float)res) * 25.4f) / radius) / deltat_Y;
                    this->angular_velocity = (angular_velocity*0.25f) + (_angular_velocity * 0.75f);
                    last_time_Y = current_time;
                }
                
            } else {
                errorState = OpticalSensErrorState::COMMUNICATION_ERROR;
                resync();
            }
        } else {
            inByte[3] = readRegister(0x00); // check id == 31 allways
            if (inByte[3] != 0x31) {
                errorState = OpticalSensErrorState::COMMUNICATION_ERROR;
                resync();
            }
        } 

    } 

    /**
     * @brief get the accumulated values in x and y direction
     * 
     * @param x movement in x direction in steps
     * @param y movement in y direction in steps
     */
    void OpticalSens::get_raw_xy(int &x, int &y) {
        x = this->dt_x;
        y = this->dt_y;
    } 

        /**
     * @brief get the accumulated values in x and y direction convertet to mm
     * 
     * @param x movement in x direction in mm
     * @param y movement in y direction in mm
     */
    void OpticalSens::get_mm_xy(float &x, float & y){
        x = (this->dt_x / (float)res) * 25.4;
        y = (this->dt_y / (float)res) * 25.4;
    } 

    /**
     * @brief reset the movement accumulator
     */
    void OpticalSens::reset(){
        dt_x = 0;
        dt_y = 0;
    } 

    /**
     * @brief read the current image quality
     * 
     * @return int image qualita (bad)0-255(good)
     */
    int OpticalSens::image_quality(){
        return readRegister(0x07);
    } 
    


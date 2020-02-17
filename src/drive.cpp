#include "drive.h"
#include "odometry.h"



void Drive::IR_encoderLeftSide()
{
    counter_L += dir_L;
}
void Drive::IR_encoderRightSide()
{
    counter_R += dir_R;
}

Drive::Drive(PinName pin_mL1, PinName pin_mL2, PinName pin_mR1, PinName pin_mR2, 
             PinName pin_encL, PinName pin_encR,float wheel_b, float wheel_d, CompactBufferLogger* _logger) : 
             mR1(pin_mR1), mR2(pin_mR2), encR(pin_encR), mL1(pin_mL1), mL2(pin_mL2), encL(pin_encL),
             mOdometry(wheel_b), ina219_R(PB_7, PB_8, 0x41, 400000, RES_12BITS),
             ina219_L(PB_7, PB_8, 0x40, 400000, RES_12BITS)
             
{
    encR.rise(callback(this,&Drive::IR_encoderRightSide));
    encR.fall(callback(this,&Drive::IR_encoderRightSide));
    encL.rise(callback(this,&Drive::IR_encoderLeftSide));
    encL.fall(callback(this,&Drive::IR_encoderLeftSide));
    
    counter_R = 0;
    counter_L = 0;
    current_R = 0.0f;
    current_L = 0.0f;
    dir_R = 0;
    dir_L = 0;

    wheel_diameter = wheel_d;
    wheel_base = wheel_b;

    logger = _logger;

    estimated_x = 0;
    estimated_y = 0;
    estimated_yaw  = 0;
    estimated_vel = 0;
    estimated_rot_v = 0;

    m_cal_cnt = 0;
    last_time = 0;
    state = TargetState::NONE;
}

Drive::~Drive()
{

}

void Drive::setup()
{
    ina219_R.calibrate_16v_400mA();
    ina219_L.calibrate_16v_400mA();
}

void  Drive::update() {

    //low pass current filter
    current_R =  (current_R  * 0.7f) +  (ina219_R.read_current_mA() * 0.3f);    
    current_L =  (current_L  * 0.7f) +  (-ina219_L.read_current_mA()* 0.3f);

    { // counter and odomertry update
    if (abs(counter_L) + abs(counter_R) >= 5) {
        float right = ((float)counter_R / ENCODER_RES) * (PI * wheel_diameter);
        float left = ((float)counter_L / ENCODER_RES) * (PI * wheel_diameter);
        
        mOdometry.move( left,right, us_ticker_read());
        counter_L = 0;
        counter_R = 0;

        float x,y,yaw;
        mOdometry.getPosition(x,y,yaw);
        float speed = mOdometry.getAvgSpeed();
        float rot_speed = mOdometry.getRotationSpeed();
        float acceleration = mOdometry.getAvgAcceleration();

        estimated_x = x;
        estimated_y = y;
        estimated_yaw = yaw;

        estimated_vel =  (estimated_vel  * 0.8f) +  (speed * 0.2f);    // [mm/s]
        estimated_rot_v = (estimated_rot_v  * 0.8f) +  (rot_speed * 0.2f);   // [rad/s]

        #ifdef DEBUG_DRIVE
            logger->begin("dri", 7);
            logger->log("ts",us_ticker_read());
            logger->log("curR", (int16_t)current_R);
            logger->log("curL", (int16_t)current_L);
            logger->log("x_y_yaw", x,y,yaw);
            logger->log("vel", speed);
            logger->log("avel", rot_speed);
            logger->log("acc", acceleration);
            logger->submit();
        #endif //DEBUG_DRIVE
        last_time = us_ticker_read();
    } else {
        unsigned long current_time = us_ticker_read();
        float deltat = (current_time - last_time) / 1e+6; // [s]
        last_time = current_time;

        estimated_x += cos(estimated_yaw) * estimated_vel * deltat;
        estimated_y += sin(estimated_yaw) * estimated_vel * deltat;
        estimated_yaw += estimated_rot_v * deltat;
    }
    }
    m_cal_cnt = (m_cal_cnt + 1) % 128;


    if(state == TargetState::ROTATION) {
        float yaw_diff = target_yaw - estimated_yaw;
            yaw_diff = normalizeRadiansPiToMinusPi(yaw_diff);

        setStop();
        if(abs(yaw_diff) > 0.045){
            if(yaw_diff > 0) {
                if( ((yaw_diff)*512.0f*PI) > m_cal_cnt){
                    leftWheelBackward();
                    rightWheelForward();
                }
            } else {
                if( ((-yaw_diff)*512.0f*PI) > m_cal_cnt) {
                    rightWheelBackward();
                    leftWheelForward();
                }
            }
        } else {
            state = TargetState::NONE;
            setStop();
            if(destinationCallback) {
                destinationCallback.call(DRIVE_DESTINATION_REACHED_EVENT);
            }
        }
        #ifdef DEBUG_DRIVE    
            logger->begin("rot",4) ;
            logger->log("c", (uint16_t)m_cal_cnt);
            logger->log("d",yaw_diff);
            //logger->log("r", mR2 == 1);
            //logger->log("l", mL2 == 1);
            logger->submit();      
        #endif
    }

    if(state == TargetState::POSITION) {

    float x_diff = target_x - estimated_x;
    float y_diff = target_y - estimated_y;
    float l_target_yaw = atan2f(y_diff,x_diff);
    float yaw_diff = (l_target_yaw - estimated_yaw);
    yaw_diff = normalizeRadiansPiToMinusPi(yaw_diff);

    if(abs(yaw_diff) > (PI / 16.0f)) {
        setStop();
        if(yaw_diff > 0) {
                if( ((yaw_diff)*512.0f*PI) > m_cal_cnt){
                    leftWheelBackward();
                    rightWheelForward();
                }
            } else {
                if( ((-yaw_diff)*512.0f*PI) > m_cal_cnt) {
                    rightWheelBackward();
                    leftWheelForward();
                }
            }

    } else {
        setForward();
        if(yaw_diff > 0) {
            if( ((yaw_diff)*128.0f) > m_cal_cnt) {
                leftWheelStop();
            }
        } else {
            if( ((-yaw_diff)*128.0f) > m_cal_cnt) {
                rightWheelStop();
            }
        }

       
    }
    #ifdef DEBUG_DRIVE    
        logger->begin("mpos",4) ;
            logger->log("c", (uint16_t)m_cal_cnt);
            logger->log("x",x_diff);
            logger->log("y",y_diff);
            logger->log("d",yaw_diff);
            //logger->log("r", mR2 == 1);
            //logger->log("l", mL2 == 1);
        logger->submit();
    #endif   

    if(sqrt(x_diff*x_diff + y_diff*y_diff) < 10) {
        //callback destiation reached;
        state = TargetState::NONE;
        setStop();
        if(destinationCallback) {
            destinationCallback.call(DRIVE_DESTINATION_REACHED_EVENT);
        }
    }


    }
}

void Drive::setForward()
{
    rightWheelForward();
    leftWheelForward();
}
void Drive::setBackward()
{
    rightWheelBackward();
    leftWheelBackward();

}
void Drive::setRotateLeft()
{
    rightWheelForward();
    leftWheelBackward();

}
void Drive::setRotateRight()
{
    rightWheelBackward();
    leftWheelForward();
}

void Drive::setStop()
{
    rightWheelStop();
    leftWheelStop();
}

void Drive::setDestinationPolar(float a, float r)
{
    float x,y,yaw;

    mOdometry.getPosition(x,y,yaw);

    target_x = (cos(a) * r) + x;
    target_y = (sin(a) * r) + y;
    state = TargetState::POSITION;
}


void Drive::setDestination(float x, float y)
{
    target_x = x;
    target_y = y;
    state = TargetState::POSITION;
}


void Drive::setDestinationRel(float x_rel, float y_rel)
{
    float x,y,yaw;
    mOdometry.getPosition(x,y,yaw);

    float x_rot = x_rel * cos(yaw)  -  y_rel* sin(yaw);
    float y_rot = x_rel * sin(yaw)  +  y_rel* cos(yaw);

    target_x = x + x_rot; 
    target_y = y + y_rot;
    state = TargetState::POSITION;

}


void  Drive::setDestinationRotation(float yaw) {

    target_yaw = yaw;
    state = TargetState::ROTATION;
}
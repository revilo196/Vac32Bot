#include "drive.h"
#include "odometry.h"



float adiff(float a1, float a2) {
    float a = a1 - a2;
    if (a > PI) {a -= 2.0*PI;}
    if (a < PI) {a += 2.0*PI;} 
    return a;
}

void Drive::IR_encoderLeftSide()
{
    unsigned long current = us_ticker_read();
    float deltat = (current - last_time_L) * 1e-6;
    if (deltat < 0.0001f) {return;}
    counter_L += dir_L;

    last_time_L = current;
    speed_L = speed_L*0.05 + max(min(((wheel_diameter/ENCODER_RES) * dir_L / deltat)*0.95f, 40.0f),-40.0f);; //distance travedl after one tick with low pass filter
    angular_velocity = (speed_R - speed_L) / wheel_base; 
    velocity = (speed_R + speed_L) / 2;
}
void Drive::IR_encoderRightSide()
{   
    unsigned long  current =  us_ticker_read();
    float deltat = (current - last_time_R) * 1e-6;
    if (deltat < 0.0001f) {return;}
    counter_R += dir_R;
    last_time_R = current;
    speed_R = speed_R*0.05  + max(min(((wheel_diameter/ENCODER_RES) * dir_R / deltat)*0.95f, 40.0f),-40.0f); //distance travedl after one tick with low pass filter
    angular_velocity = (speed_R - speed_L) / wheel_base; 
    velocity = (speed_R + speed_L) / 2;
}

Drive::Drive(PinName pin_mL1, PinName pin_mL2, PinName pin_mR1, PinName pin_mR2,
             PinName pin_encL, PinName pin_encR,float wheel_b, float wheel_d, CompactBufferLogger* _logger) : 
             mR1(pin_mR1), mR2(pin_mR2), mL1(pin_mL1), mL2(pin_mL2),
             encR(pin_encR), encL(pin_encL), mOdometry(wheel_b), 
             ina219_R(PB_7, PB_8, 0x41, 400000, RES_12BITS),
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
    
    unsigned long current_time = us_ticker_read();
    float deltat = (current_time - last_time) / 1e+6; // [s]
    last_time = current_time;
    if (current_time - last_time_L >  300000)  {//300ms
        speed_L = 0;
    }
    if (current_time - last_time_R >  300000)  {//300ms
        speed_R = 0;
    }
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

        estimated_vel =  (estimated_vel  * 0.15f) +  (speed * 0.85f);    // [mm/s]
        estimated_rot_v = (estimated_rot_v  * 0.15f) +  (rot_speed * 0.85f);   // [rad/s]

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


        estimated_x += cos(estimated_yaw) * velocity * deltat;
        estimated_y += sin(estimated_yaw) * velocity * deltat;
        estimated_yaw += angular_velocity * deltat;
        estimated_yaw = normalizeRadiansPiToMinusPi(estimated_yaw);
    }
    }

    const float rot_Kp = 2;
    const float for_Kp = 4.9;

    if(state == TargetState::ROTATION) {
        float yaw_diff = adiff(target_yaw, estimated_yaw);
            yaw_diff = normalizeRadiansPiToMinusPi(yaw_diff);

        if(abs(yaw_diff) > 0.025){
            if(yaw_diff > 0) {
                {
                    leftWheelBackward(fabs(yaw_diff*rot_Kp));
                    rightWheelForward(fabs(yaw_diff*rot_Kp));
                }
            } else {
                {
                    rightWheelBackward(fabs(yaw_diff*rot_Kp));
                    leftWheelForward(fabs(yaw_diff*rot_Kp));
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
            logger->begin("rot",1) ;
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
    error_integral += yaw_diff * deltat;
    error_integral = max(min(error_integral, 2.0f),-2.0f);
    float err_diff =    yaw_diff -last_err;
    last_err = yaw_diff;
    float er = (yaw_diff*for_Kp + error_integral*1.9 + err_diff*0.7);
    float edist = (sqrt(x_diff*x_diff + y_diff*y_diff) / 60) +0.2;
    
        if(er > 0) {
            {   
                rightWheelForward(1.0f * edist);
                leftWheel((1.0f - fabs(er)) * edist);
            }
        } else {
            {
                rightWheel((1.0f - fabs(er)) * edist);
                leftWheelForward(1.0f*  edist);
            }
        }

       
   // }
    #ifdef DEBUG_DRIVE    
            logger->begin("mpos",11) ;
            logger->log("dx",x_diff);
            logger->log("dy",y_diff);
            logger->log("target_a", l_target_yaw);
            logger->log("est_yaw", estimated_yaw);
            logger->log("pid", er);
            logger->log("err",yaw_diff);
            logger->log("err_integral",error_integral);
            logger->log("vel", velocity);
            logger->log("avel", angular_velocity);
            logger->log("vr", speed_R);
            logger->log("vl", speed_L);
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

void Drive::setForward(float f)
{
    rightWheelForward(f);
    leftWheelForward(f);
}
void Drive::setBackward(float f)
{
    rightWheelBackward(f);
    leftWheelBackward(f);

}
void Drive::setRotateLeft(float f)
{
    rightWheelForward(f);
    leftWheelBackward(f);

}
void Drive::setRotateRight(float f)
{
    rightWheelBackward(f);
    leftWheelForward(f);
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
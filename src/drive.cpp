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
    speed_L = max(min((((float)(wheel_diameter*PI)/(float)ENCODER_RES) * dir_L / deltat), 100.0f),-100.0f);; //distance travedl after one tick with low pass filter
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
    speed_R = max(min((((float)(wheel_diameter*PI)/(float)ENCODER_RES) * dir_R / deltat), 100.0f),-100.0f); //distance travedl after one tick with low pass filter
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
    mR1.period_us(500);
    mR2.period_us(500);
    mL1.period_us(500);
    mL2.period_us(500);

    encR.rise(callback(this,&Drive::IR_encoderRightSide));
    encR.fall(callback(this,&Drive::IR_encoderRightSide));
    encL.rise(callback(this,&Drive::IR_encoderLeftSide));
    encL.fall(callback(this,&Drive::IR_encoderLeftSide));
    
    counter_R = 0;
    counter_L = 0;
    current_R = 0.0f;
    current_L = 0.0f;
    overR = 0;
    overL = 0;
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

    current_sensing();

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



#define D_CAL_SIZE 512
#define D_CAL_RATE_MS 2
#define DSQ_MOVING_AVERGE 30
#define DSQ_SELLTE_PRZ 0.12
void Drive::adv_calibration() {
    float curL[D_CAL_SIZE];
    float curR[D_CAL_SIZE];
    int wp = 0;
    setStop();

    //Aquire data from start, driving and  stop
    wait_us(500*1000); //200ms
    setForward(1.0f);
    for(;wp < D_CAL_SIZE/2; wp++) {
            curL[wp] = abs(ina219_R.read_current_mA());    
            curR[wp] = abs(ina219_L.read_current_mA());
            wait_us(D_CAL_RATE_MS*1000);
    }
    setBackward(1.0f);
    for(;wp < D_CAL_SIZE; wp++) {
            curL[wp] = abs(ina219_R.read_current_mA());    
            curR[wp] = abs(ina219_L.read_current_mA());
            wait_us(D_CAL_RATE_MS*1000);
    }
    setStop();
    
    /** Optional calculations
    //double avgR = curR[0] /(double)D_CAL_SIZE;
    //double avgL = curL[0] /(double)D_CAL_SIZE;
    //double maxR = curR[0];
    //double maxL = curL[0];
    //double avgDSQ_R = 0;
    //double avgDSQ_L = 0;
    **/
    //do calculation with aquired data
    // ---- Go over the the data and find the biggest rise in the current over smoothed data (Moving_Average)
    float maxDSQ_R = 0;
    float maxDSQ_L = 0;
    for(int i = 1; i < D_CAL_SIZE; i++) {
        //avgL += curL[i]/(double)D_CAL_SIZE;
        //avgR += curR[i]/(double)D_CAL_SIZE;

        if(i >= DSQ_MOVING_AVERGE) {
            float diff_L_SQ = 0;
            float diff_R_SQ = 0; 
            for(int j = i; j >= i - DSQ_MOVING_AVERGE  && j >= 0; j--) {
                float diff_L = curL[j-1] - curL[j];
                float diff_R = curR[j-1] - curR[j];
                diff_L_SQ += diff_L /(float)DSQ_MOVING_AVERGE;
                diff_R_SQ += diff_R /(float)DSQ_MOVING_AVERGE;
            }
            diff_L_SQ = diff_L_SQ*diff_L_SQ;
            diff_R_SQ = diff_R_SQ*diff_R_SQ; 
            //avgDSQ_R += diff_R_SQ / (float)(D_CAL_SIZE-DSQ_MOVING_AVERGE);
            //avgDSQ_L += diff_L_SQ / (float)(D_CAL_SIZE-DSQ_MOVING_AVERGE);

            maxDSQ_R = max(diff_L_SQ, maxDSQ_R);
            maxDSQ_L = max(diff_R_SQ, maxDSQ_L);
        }
        //maxR = max((float)curR[i], maxR);
        //maxL = max((float)curL[i], maxL);
    }
    // calculate the rise value (settel point) when the current goes back to normal
    float settle_DSQ_R = pow(sqrt(maxDSQ_R) * DSQ_SELLTE_PRZ ,2) ;
    float settle_DSQ_L = pow(sqrt(maxDSQ_L) * DSQ_SELLTE_PRZ ,2);  

    //--- go over the the data a 2nd time and calculate the average current in settled region (when the current is not changing)
    float settle_avgR = 0; int settle_avgR_cnt = 0;
    float settle_avgL = 0; int settle_avgL_cnt = 0;
    float settle_maxR = 0;
    float settle_maxL = 0;
    //float settle_minR = maxR;
    //float settle_minL = maxL;
    for(int i = DSQ_MOVING_AVERGE; i < D_CAL_SIZE; i++) {
        float diff_L_SQ = 0;
        float diff_R_SQ = 0; 
        for(int j = i; j >= i - DSQ_MOVING_AVERGE  && j >= 0; j--) {
            float diff_L = curL[j-1] - curL[j];
            float diff_R = curR[j-1] - curR[j];
            diff_L_SQ += diff_L /(float)DSQ_MOVING_AVERGE;
            diff_R_SQ += diff_R /(float)DSQ_MOVING_AVERGE;
        }
        diff_L_SQ = diff_L_SQ*diff_L_SQ;
        diff_R_SQ = diff_R_SQ*diff_R_SQ; 

        if (diff_R_SQ < settle_DSQ_R) {
            settle_avgR += curR[i];
            settle_avgR_cnt++;
            settle_maxR = max(settle_maxR, (float)curR[i]);
            //settle_minR = min(settle_minR, (float)curR[i]);
        }

        if (diff_L_SQ < settle_DSQ_L) {
            settle_avgL+= curL[i];
            settle_avgL_cnt++;
            settle_maxL = max(settle_maxL, (float)curL[i]);
            //settle_minL = min(settle_minL, (float)curL[i]);
        }
    }
    settle_avgL /= (float)settle_avgL_cnt; //calculate sum to average
    settle_avgR /= (float)settle_avgR_cnt; //calculate sum to average

    #ifdef DEBUG_DRIVE
        logger->begin("dri_cal", 3);
        logger->log("set_DSQ", settle_DSQ_R);
        logger->log("set_avg", settle_avgR);
        logger->log("set_max", settle_maxR);
        //logger->log("set_min", settle_minR);
        logger->log("max_DSQ", maxDSQ_R);
        //logger->log("avg_DSQ", avgDSQ_R);
        //logger->log("max", maxR);
        //logger->log("avg", avgR);
        logger->submit();

        logger->begin("dri_cal", 3);
        logger->log("set_DSQ", settle_DSQ_L);
        logger->log("set_avg", settle_avgL);
        logger->log("set_max", settle_maxL);
        //logger->log("set_min", settle_minL);
        logger->log("max_DSQ", maxDSQ_L);
        //logger->log("avg_DSQ", avgDSQ_L);
        //logger->log("max", maxL);
        //logger->log("avg", avgL);
        logger->submit();

        for(int i = 0; i < D_CAL_SIZE; i++) {
            logger->begin("dri_data", 2);
            logger->log("R", curR[i]);
            logger->log("L", curL[i]);
            logger->submit();
        }
    #endif


    //output to class values;
    set_DSQR = settle_DSQ_R;
    set_DSQL = settle_DSQ_L;
    set_avgR = settle_avgR;
    set_avgL = settle_avgL;
}

void Drive::current_sensing() {

    float curL = abs(ina219_R.read_current_mA());    
    float curR = abs(ina219_L.read_current_mA());

    float diff_L = current_L  - curL;
    float diff_R = current_R  - curR;

    current_L = current_L*0.6 + curL*0.4;
    current_R = current_R*0.6 + curR*0.4;

    diffBuffer_L[pDiffBuffer] = diff_L;
    diffBuffer_R[pDiffBuffer] = diff_R;
    pDiffBuffer = (pDiffBuffer + 1) % D_DIFF_BUFF;

    float DSQ_L = 0;
    float DSQ_R = 0;
    for (int i = 0 ; i < D_DIFF_BUFF; i++) {
        DSQ_L += diffBuffer_L[i] / (float)D_DIFF_BUFF;
        DSQ_R += diffBuffer_R[i] / (float)D_DIFF_BUFF;
    }
    DSQ_L = DSQ_L*DSQ_L;
    DSQ_R = DSQ_R*DSQ_R;

    DSQ_L *= (1/set_DSQL);
    DSQ_R *= (1/set_DSQR);
    float DET_L = min(pow(DSQ_L,8), 1.0);
    float DET_R = min(pow(DSQ_R,8), 1.0);

    float oL = ((current_L * (1 - DET_L)) / set_avgL ) - 1.0;
    float oR = ((current_R * (1 - DET_R)) / set_avgR ) - 1.0;

    overL = (overL*0.9) + (oL * 0.1); 
    overR = (overR*0.9) + (oR * 0.1); 

    if (overL > 0.1) // 10% over average
        overCntL++;
    else 
        overCntL = 0;

    if (overR > 0.10) // 10% over average
        overCntR++;
    else 
        overCntR = 0;

    if (overCntL > 10) {
        if(navigationAbortInterrupt) {
            navigationAbortInterrupt(0);
        }
    }

    if (overCntR > 10) {
        if(navigationAbortInterrupt) {
            navigationAbortInterrupt(0);
        }
    }


    logger->begin("c", 10);
    logger->log("r", current_R);
    logger->log("l", current_L);
    logger->log("dr", DSQ_R);
    logger->log("dl", DSQ_L);
    logger->log("tr", DET_R);
    logger->log("tl", DET_L);
    logger->log("or", overR);
    logger->log("ol", overL);
    logger->log("cr", (uint16_t) overCntR);
    logger->log("cl", (uint16_t) overCntL);
    logger->submit();


}

/*
TODO
sensor input
-> diff from prev
    -> mean of last (30) diffs
    -> square this mean to get DSQ 
    -> scale by (1/settle_DSQ)
    -> X^8 for noise filter  -> s_dectector

-> current input * (1- s_dectector)
  -> low pass filter or mean(15)
  -> substact  settle_avgR
  -> if this is bigger than a error margin !!!
*/
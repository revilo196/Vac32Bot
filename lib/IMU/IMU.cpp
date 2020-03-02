#include "IMU.h"



IMU::IMU(Serial * console,CompactBufferLogger* _logger) : mpu()
{
    logger = _logger;
    pc = console;
    q[0] = 1; q[1] = 1; q[2] = 1; q[2] = 1; q[3] = 1; 
    deltat = 0;
    float GyroMeasError = PI * (3.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 10 deg/s), then reduce after ~10 s to 3
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; 
    last_time = 0;
    d_pitch = 0;
    d_yaw = 0;
    d_roll = 0;

    l_roll = 0;
    l_pitch = 0;
    l_yaw = 0;

    for(int i = 0; i < 3; i++) {
        accStddevData[i] =  0.0f;
        gyrStddevData[i] =  0.0f;
        accUserBias[i] = 0;
        gyrUserBias[i] = 0;
        accUserBias_PREC[i] = 0.0f;
        gyrUserBias_PREC[i] = 0.0f;
        delta_v[i] =  0.0f;
        delta_p[i] =  0.0f;
        last_acc[i] =  0.0f;
        jerk[i] =  0.0f;
        jerk_sq[i] =  0.0f;
    }

    for(int i = 0 ;i < 6*6; i++){
        covariance[i/6][i%6] = 0;
        corelation[i/6][i%6] = 0;
    }
}

IMU::~IMU()
{
}

int IMU::setup() {


    uint8_t whoami = mpu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    #ifdef DEBUG_IMU
        pc->printf("I AM 0x%x\n\r", whoami);
        pc->printf("I SHOULD BE 0x71\n\r");
    #endif 

    if (whoami != 0x71) { // WHO_AM_I should always be 0x68
        #ifdef DEBUG_IMU
            pc->printf("Could not connect to MPU9250: \n\r");
            pc->printf("%#x \n",  whoami);
        #endif 
        return -1;
    }  

    #ifdef DEBUG_IMU
        pc->printf("MPU9250 is online...\n\r");
    #endif 
    wait_us(1000*1000);
 
    mpu.resetMPU9250(); // Reset registers to default in preparation for device calibration
    mpu.calibrateMPU9250(mpu.gyroBias, mpu.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    #ifdef DEBUG_IMU
        pc->printf("x gyro bias = %f\n\r", mpu.gyroBias[0]);
        pc->printf("y gyro bias = %f\n\r", mpu.gyroBias[1]);
        pc->printf("z gyro bias = %f\n\r", mpu.gyroBias[2]);
        pc->printf("x accel bias = %f\n\r", mpu.accelBias[0]);
        pc->printf("y accel bias = %f\n\r", mpu.accelBias[1]);
        pc->printf("z accel bias = %f\n\r", mpu.accelBias[2]);
    #endif 

    wait_us(1000*1000);
    mpu.initMPU9250();

    #ifdef DEBUG_IMU
        pc->printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        pc->printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<mpu.Ascale));
        pc->printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<mpu.Gscale));
    #endif 


    wait_us(1000*1000);

    mpu.getAres(); // Get accelerometer sensitivity
    mpu.getGres(); // Get gyro sensitivity
    #ifdef DEBUG_IMU
        pc->printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/mpu.aRes);
        pc->printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/mpu.gRes);
    #endif 

    wait_us(1000*1000);



    last_time = us_ticker_read();
    return 0;
}


// should be called apoxmatly every 1 ms to get the 1000Hz update from the sensor
int IMU::update() {
    mpu.readAccelData(accData);
    float ax = (accData[0]*mpu.aRes);
    float ay = (accData[1]*mpu.aRes);
    float az = (accData[2]*mpu.aRes);

    mpu.readGyroData(gyroData);
    float gx = ((gyroData[0] - gyrUserBias_PREC[0] )*mpu.gRes  / 180.0 * PI);
    float gy = ((gyroData[1] - gyrUserBias_PREC[1] )*mpu.gRes /  180.0 * PI);
    float gz = ((gyroData[2] - gyrUserBias_PREC[2] )*mpu.gRes /  180.0 * PI);

    unsigned long current_time = us_ticker_read();
    deltat = (current_time - last_time) / 1e+6;
    last_time = current_time;

    jerk[0] = (ax - last_acc[0]) / deltat;
    jerk[1] = (ay - last_acc[1]) / deltat;
    jerk[2] = (az - last_acc[2]) / deltat;

    jerk_sq[0] = jerk[0] * jerk[0];
    jerk_sq[1] = jerk[1] * jerk[1];
    jerk_sq[2] = jerk[2] * jerk[2];

    last_acc[0] = 0;
    last_acc[1] = 0;
    last_acc[2] = 0;

    MadgwickQuaternionUpdate(ax,ay,az,gx,gy,gz);
    updateRotation();

    if ( sqrt(jerk_sq[0] + jerk_sq[1]) > 150) {
        if (bumpNavigationSignal) {
            bumpNavigationSignal(1);
        }
    }
    
    //TODO integrade accelerometer data to velocity
    // and velocity to position

    //Update delta calculation and delta sum calculation 
    d_roll += roll - l_roll;
    l_roll  = roll;

    d_pitch += pitch - l_pitch;
    l_pitch  = pitch;

    d_yaw += yaw - l_yaw;
    l_yaw  = yaw; 

    #ifdef DEBUG_IMU
        logger->begin("IMU",6);
        logger->log("ts", current_time);
        logger->log("dt", deltat);
        logger->log("jerk", sqrt(jerk_sq[0] + jerk_sq[1]));
        logger->log("acc", sqrt(ax*ax + ay*ay));
        logger->log("yaw", yaw);
        logger->submit();
    #endif 

    return 0;
}

#define CALSIZE 4096
int IMU::adv_calibration() {

    int16_t accCalData[CALSIZE][3];
    int16_t gyrCalData[CALSIZE][3];
    int64_t accSumData[3];
    int64_t gyrSumData[3];
    int64_t accVarianceData[3];
    int64_t gyrVarianceData[3];


    for(int j = 0; j < 3; j++) {
        accSumData[j] = 0;
        gyrSumData[j] = 0;
        accVarianceData[j] = 0;
        gyrVarianceData[j] = 0;
    }
    for(int j = 0; j < 6; j++) {
        for(int k = 0; k < 6; k++) {
            covariance[j][k] =  0;
            corelation[j][k] = 0;
        }
    }
    for(int i = 0; i < CALSIZE; i++) {
        mpu.readAccelData(accCalData[i]);
        mpu.readGyroData(gyrCalData[i]);


        for(int j = 0; j < 3; j++) {
            accCalData[i][j] -= accUserBias[j];
            gyrCalData[i][j] -= gyrUserBias[j];

            accSumData[j] += accCalData[i][j];
            gyrSumData[j] += gyrCalData[i][j];
        }
        logger->begin("cal_data", 6);
        logger->log("aX", accCalData[i][0]);
        logger->log("aY", accCalData[i][1]);
        logger->log("aZ", accCalData[i][2]);
        logger->log("gX", gyrCalData[i][0]);
        logger->log("gY", gyrCalData[i][1]);
        logger->log("gZ", gyrCalData[i][2]);
        logger->submit();

        wait_us(1000);
    }

    for(int j = 0; j < 3; j++) {
        accUserBias[j] += accSumData[j] / CALSIZE;
        gyrUserBias[j] += gyrSumData[j] / CALSIZE;
        accUserBias_PREC[j] += (double)accSumData[j] / (double)CALSIZE;
        gyrUserBias_PREC[j] += (double)gyrSumData[j] / (double)CALSIZE;
    }

    for(int i = 0; i < CALSIZE; i++) {
        int32_t de_mean[6];
        for(int j = 0; j < 3; j++) {
            accVarianceData[j] += (accCalData[i][j]-accUserBias[j])* (accCalData[i][j]-accUserBias[j]);
            de_mean[j] =  (accCalData[i][j]-accUserBias[j]);
            gyrVarianceData[j] += (gyrCalData[i][j]-gyrUserBias[j])*(gyrCalData[i][j]-gyrUserBias[j]);
            de_mean[j+3] =  (gyrCalData[i][j]-gyrUserBias[j]);
        }   
        for(int j = 0; j < 6; j++) {
            for(int k = 0; k < 6; k++) {
                    covariance[j][k] +=  de_mean[j] * de_mean[k];
            }
        }
    }
    
    for(int j = 0; j < 6; j++) {
        for(int k = 0; k < 6; k++) {
            covariance[j][k] =   covariance[j][k]  / (CALSIZE -1);
        }
    }

    for(int j = 0; j < 3; j++) {
        accVarianceData[j] =  (accVarianceData[j]) / CALSIZE;
        gyrVarianceData[j] =  (gyrVarianceData[j]) / CALSIZE;
        accStddevData[j] =  sqrt(accVarianceData[j]);
        gyrStddevData[j] =  sqrt(gyrVarianceData[j]);
    }
    float st_devs[6];
    for(int j = 0; j < 3; j++) {
        st_devs[j] = accStddevData[j];
        st_devs[j+3] = gyrStddevData[j];
    }

    for(int j = 0; j < 6; j++) {
        for(int k = 0; k < 6; k++) {
            corelation[j][k] =   covariance[j][k]  / st_devs[j] / st_devs[k];
        }
    }

    logger->begin("covar", 1);
    logger->log("cov", (int*)covariance, 6, 6);
    logger->submit();
    
    logger->begin("correlation", 1);
    logger->log("cor", (float*)corelation, 6, 6);
    logger->submit();


    //compute_covariance_matrix(data, cov);
    logger->begin("calibrate_cov", 4);
    logger->log("uax", accUserBias[0]);
    logger->log("uay", accUserBias[1]);
    logger->log("uaz", accUserBias[2]);
    logger->log("ugx", gyrUserBias[0]);
    logger->log("ugy", gyrUserBias[1]);
    logger->log("ugz", gyrUserBias[2]);
    logger->log("std_acc", accStddevData[0], accStddevData[1], accStddevData[2]);
    logger->log("std_gyr", gyrStddevData[0], gyrStddevData[1], gyrStddevData[2]);
    logger->submit();
    return 0;
}

#define RE_CALSIZE 512
int IMU::re_calibrate() {

    int16_t accCalData[3];
    int16_t gyrCalData[3];
    int32_t accSumData[3];
    int32_t gyrSumData[3];
    double accSumDataPREC[3];
    double gyrSumDataPREC[3];

    for(int i = 0; i < 3; i++) {
        accCalData[i] = 0.0;
        gyrCalData[i] = 0.0;
        accSumData[i] = 0.0;
        gyrSumData[i] = 0.0;
        accSumDataPREC[i] = 0.0;
        gyrSumDataPREC[i] = 0.0;
    }


     for(int i = 0; i < RE_CALSIZE; i++) {
        mpu.readAccelData(accCalData);
        mpu.readGyroData(gyrCalData);
        
        for(int j = 0; j < 3; j++) {
            accSumDataPREC[j] += accCalData[j] - accUserBias_PREC[j];
            gyrSumDataPREC[j] += gyrCalData[j] - gyrUserBias_PREC[j];

            accCalData[j] -= accUserBias[j];
            gyrCalData[j] -= gyrUserBias[j];
            
            accSumData[j] += accCalData[j];
            gyrSumData[j] += gyrCalData[j];
        }
        logger->begin("re_cal_data", 6);
        logger->log("aX", accCalData[0]);
        logger->log("aY", accCalData[1]);
        logger->log("aZ", accCalData[2]);
        logger->log("gX", gyrCalData[0]);
        logger->log("gY", gyrCalData[1]);
        logger->log("gZ", gyrCalData[2]);
        logger->submit();

        wait_us(1000);
    }

    for(int j = 0; j < 3; j++) {
        accUserBias[j] += accSumData[j] / CALSIZE;
        gyrUserBias[j] += gyrSumData[j] / CALSIZE;
        accUserBias_PREC[j] += accSumDataPREC[j] / (double)CALSIZE;
        gyrUserBias_PREC[j] += gyrSumDataPREC[j] / (double)CALSIZE;
    }
    return 0;
}

void IMU::updateRotation() 
{
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
}


void IMU::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
            float norm;                                               // vector norm
            float f1, f2, f3;                                         // objective funcyion elements
            float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
            float qDot1, qDot2, qDot3, qDot4;
            float hatDot1, hatDot2, hatDot3, hatDot4;
            float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;  // gyro bias error

            // Auxiliary variables to avoid repeated arithmetic
            float _halfq1 = 0.5f * q1;
            float _halfq2 = 0.5f * q2;
            float _halfq3 = 0.5f * q3;
            float _halfq4 = 0.5f * q4;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;


            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;
            
            // Compute the objective function and Jacobian
            f1 = _2q2 * q4 - _2q1 * q3 - ax;
            f2 = _2q1 * q2 + _2q3 * q4 - ay;
            f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
            J_11or24 = _2q3;
            J_12or23 = _2q4;
            J_13or22 = _2q1;
            J_14or21 = _2q2;
            J_32 = 2.0f * J_14or21;
            J_33 = 2.0f * J_11or24;
          
            // Compute the gradient (matrix multiplication)
            hatDot1 = J_14or21 * f2 - J_11or24 * f1;
            hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
            hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
            hatDot4 = J_14or21 * f1 + J_11or24 * f2;
            
            // Normalize the gradient
            norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
            hatDot1 /= norm;
            hatDot2 /= norm;
            hatDot3 /= norm;
            hatDot4 /= norm;
            
            // Compute estimated gyroscope biases
            gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
            gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
            gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
            
            // Compute and remove gyroscope biases
            gbiasx += gerrx * deltat * zeta;
            gbiasy += gerry * deltat * zeta;
            gbiasz += gerrz * deltat * zeta;
            
            // Compute the quaternion derivative
            qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
            qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
            qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
            qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

            // Compute then integrate estimated quaternion derivative
            q1 += (qDot1 -(beta * hatDot1)) * deltat;
            q2 += (qDot2 -(beta * hatDot2)) * deltat;
            q3 += (qDot3 -(beta * hatDot3)) * deltat;
            q4 += (qDot4 -(beta * hatDot4)) * deltat;

            // Normalize the quaternion
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;
            
}

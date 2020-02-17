#include "IMU.h"

#define DEBUG_IMU

IMU::IMU(Serial * console,CompactBufferLogger* _logger) : mpu()
{
    logger = _logger;
    pc = console;
    q[0] = 1; q[1] = 1; q[2] = 1; q[2] = 1; q[3] = 1; 
    deltat = 0;
    float GyroMeasError = PI * (10.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 10 deg/s), then reduce after ~10 s to 3
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

    delta_v[0] = 0;
    delta_v[1] = 0;
    delta_v[2] = 0;

    delta_p[0] = 0;
    delta_p[1] = 0;
    delta_p[2] = 0;

    last_acc[0] = 0;
    last_acc[1] = 0;
    last_acc[2] = 0;

    jerk[0] = 0;
    jerk[1] = 0;
    jerk[2] = 0;

    jerk_sq[0] = 0;
    jerk_sq[1] = 0;
    jerk_sq[2] = 0;
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
    float gx = (gyroData[0]*mpu.gRes  / 180.0 * PI);
    float gy = (gyroData[1]*mpu.gRes /  180.0 * PI);
    float gz = (gyroData[2]*mpu.gRes /  180.0 * PI);

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
        bump_flag = true;
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
        logger->log("bump", bump_flag);
        logger->log("yaw", yaw);
        logger->submit();
    #endif 

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

#include <arduino.h>
#include <limits.h>
#include "MPU9250_DMP_enh.hpp"

// #define min(a,b)    ((a>b)?b:a)
// #define max(a,b)    ((a>b)?a:b)

MPU9250_DMP_enh::MPU9250_DMP_enh(){
    newData = false;
    sumCount = 0;
    tapDir = tapCnt = 0;
    memset(fmpuD, 0, sizeof(fmpuD));
    memset(magBias, 0, sizeof(magBias));
    memset(magScale, 0, sizeof(magScale));
}

short MPU9250_DMP_enh::handleIRQ(){
    short res = 0;

    short status;
    mpu_get_int_status(&status);
    if(status & 0x001){   // data available
        readMPU();
        res = 1;
    }
    return res;
}

short MPU9250_DMP_enh::handleFIFOIRQ(){
    short res = 0;
    // Check for new data in the FIFO
    if ( fifoAvailable() )
    {
        res |= 0x02;
        // DMP FIFO must be updated in order to update tap data
        dmpUpdateFifo();
        // Check for new tap data by polling tapAvailable
        if ( tapAvailable() )
        {
            // If a new tap happened, get the direction and count
            // by reading getTapDir and getTapCount
            tapDir = getTapDir();
            tapCnt = getTapCount();
            res |= 0x04;
        }
    }
    return res;
}

void MPU9250_DMP_enh::updateTime(){
    now = micros();

    // Set integration time by time elapsed since last filter update
    deltat = ((now - lastUpdate) / 1000000.0f);
    lastUpdate = now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
}

short MPU9250_DMP_enh::calAccelGyro(float *accelB, float *gyroB){
    begin();                // reset devices
    setLPF(188);            // set low-pass filter to 188 Hz
    setSampleRate(1000);    // set sample rate to 1kHz
    setGyroFSR(250);        // set gyro full scale range to 250 deg/setClock
    setAccelFSR(2);         // set accel full scale range to 2 G

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    resetFifo();
    configureFifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
    int16_t bytes, cnt = 0;
    do{
        delay(40);
        bytes = fifoAvailable();
        cnt++;
    }while(bytes < 40*12 && cnt < 20);

    configureFifo(0);       // disable fifo
    int16_t packetCnt = bytes/12;  // 3*2 + 3*2 bytes / package

    long accelBias[3] = {0,0,0};
    long gyroBias[3] = {0,0,0};

    int16_t j = 0;
    for( int16_t i = j = 0; i < packetCnt; i++){
        if(updateFifo() == INV_SUCCESS){
            accelBias[0] += ax;
            accelBias[1] += ay;
            accelBias[2] += az;
            gyroBias[0] += gx;
            gyroBias[1] += gy;
            gyroBias[2] += gz;
            j++;
        }
    }
    if(j <= 0){
        accelB[0] = bytes;
        accelB[1] = j;
        accelB[1] = cnt;
        return INV_ERROR;
    }

    accelBias[0] /= (int32_t)bytes;
    accelBias[1] /= (int32_t)bytes;
    accelBias[2] /= (int32_t)bytes;
    gyroBias[0] /= (int32_t)bytes;
    gyroBias[1] /= (int32_t)bytes;
    gyroBias[2] /= (int32_t)bytes;

    if(accelBias[2] > 0L) {accelBias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accelBias[2] += (int32_t) accelsensitivity;}

    for(uint16_t i = 0; i < 3; i++){
        *accelB++ = (float)accelBias[i] * (float)accelsensitivity;
    }
    for(uint16_t i = 0; i < 3; i++){
        *gyroB++ = (float)gyroBias[i] * (float)gyrosensitivity;
    }

    gyroBias[0] /= -4;      // divide by 4 for 1000g full scale range (we measured with 256 g fsr)
    gyroBias[1] /= -4;      // and change sign because it has to be subtracted!
    gyroBias[2] /= -4;
    mpu_set_gyro_bias_reg(gyroBias);

    for(uint16_t i = 0; i < 3; i++){
        accelBias[i] /= -8; // divide by 8 for 16g fsr and apply sign
    }
    mpu_set_accel_bias_6500_reg(accelBias);

    return INV_SUCCESS;
}

void MPU9250_DMP_enh::readMPU(){
    // Call update() to update the imu objects sensor data. You can specify
    // which sensors to update by OR'ing UPDATE_ACCEL, UPDATE_GYRO,
    // UPDATE_COMPASS, and/or UPDATE_TEMPERATURE.
    // (The update function defaults to accel, gyro, compass, so you don't
    // have to specify these values.)
    update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);

    fmpuD[IMUa][0] = calcAccel(ax); // accelX is x-axis acceleration in g's
    fmpuD[IMUa][1] = calcAccel(ay); // accelY is y-axis acceleration in g's
    fmpuD[IMUa][2] = calcAccel(az); // accelZ is z-axis acceleration in g's

    fmpuD[IMUg][0] = calcGyro(gx); // gyroX is x-axis rotation in dps
    fmpuD[IMUg][1] = calcGyro(gy); // gyroY is y-axis rotation in dps
    fmpuD[IMUg][2] = calcGyro(gz); // gyroZ is z-axis rotation in dps

    fmpuD[IMUm][0] = calcMag(mx); // magX is x-axis magnetic field in uT
    fmpuD[IMUm][1] = calcMag(my); // magY is y-axis magnetic field in uT
    fmpuD[IMUm][2] = calcMag(mz); // magZ is z-axis magnetic field in uT
    newData = true;
}

void MPU9250_DMP_enh::magcalMPU9250()
{
    uint16_t ii = 0, sample_count = 0, del = 1;
    int32_t mag_scale[3] = {0, 0, 0};
    int mag_max[3] = {INT_MIN, INT_MIN, INT_MIN}, mag_min[3] = {INT_MAX, INT_MAX, INT_MAX};

    // shoot for ~fifteen seconds of mag data
    if(getCompassSampleRate() < 50){
        sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
        del = 135;
    }
    else{
        sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
        del = 12;
    }

    for(ii = 0; ii < sample_count; ii++) {
        update(UPDATE_COMPASS);
        mag_max[0] = max(mag_max[0], mx);
        mag_min[0] = min(mag_min[0], mx);
        mag_max[1] = max(mag_max[1], my);
        mag_min[1] = min(mag_min[1], my);
        mag_max[2] = max(mag_max[2], mz);
        mag_min[2] = min(mag_min[2], mz);
        delay(del);
    }


    // Get hard iron correction
    // get internal correction data
    short intMagCal[3];
    mpu_get_compass_sense_adj(intMagCal);

    for(ii = 0; ii < 3; ii++ ){
        float mBias = (mag_max[ii] + mag_min[ii])/2.0f;  // get average x mag bias in counts
        magBias[ii] = mBias * getMagSens() * ((float)intMagCal[ii]/256.0f);
    }

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    magScale[0] = avg_rad/((float)mag_scale[0]);
    magScale[1] = avg_rad/((float)mag_scale[1]);
    magScale[2] = avg_rad/((float)mag_scale[2]);
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter
// for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples & more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a
// quaternion-based estimate of absolute device orientation -- which can be
// converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional
// Kalman-based filtering algorithms but is much less computationally
// intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
#define Kp 2.0f * 5.0f
#define Ki 0.0f

const float GyroMeasError = PI * (40.0f / 180.0f);
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
const float GyroMeasDrift = PI * (0.0f  / 180.0f);
// There is a tradeoff in the beta parameter between accuracy and response
// speed. In the original Madgwick study, beta of 0.041 (corresponding to
// GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds
// to a stable initial quaternion. Subsequent changes also require a
// longish lag time to a stable output, not fast enough for a quadcopter or
// robot car! By increasing beta (GyroMeasError) by about a factor of
// fifteen, the response time constant is reduced to ~2 sec. I haven't
// noticed any reduction in solution accuracy. This is essentially the I
// coefficient in a PID control sense; the bigger the feedback coefficient,
// the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and
// fusion scheme.
const float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // Compute beta
// Compute zeta, the other free parameter in the Madgwick scheme usually
// set to a small or zero value
const float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

void MPU9250_DMP_enh::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    // Must be called before updating quaternions!
    updateTime();

    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 +
    _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}



// Similar to Madgwick scheme but uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void MPU9250_DMP_enh::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    // Must be called before updating quaternions!
    updateTime();

    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // Handle NaN
    norm = 1.0f / norm;       // Use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity and magnetic field
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void MPU9250_DMP_enh::calcEulerAngles(float corDeclination){

    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.

    // calc Roll (rotation around x-axis)
    float sinr = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    float cosr = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    fEuler[0] = atan2(sinr, cosr);

    // calc pitch (rotation around y-axis)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if( fabs(sinp) >= 1){
        fEuler[1] = copysign(M_PI / 2, sinp);
    }
    else{
        fEuler[1] = asin(sinp);
    }

    // calc yaw (rotation around z-axis)
    float siny = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    fEuler[2] = atan2(siny, cosy);

    fEuler[0] *= RAD_TO_DEG;
    fEuler[1] *= RAD_TO_DEG;
    fEuler[2] *= RAD_TO_DEG;

    fEuler[2] += corDeclination;
}

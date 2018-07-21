#include <arduino.h>
#include "MPU9250_DMP_enh.hpp"
#include "KWutils/quaternionFilters.h"

MPU9250_DMP_enh::MPU9250_DMP_enh(){
    newData = false;
    sumCount = 0;
    tapDir = tapCnt = 0;
    memset(fmpuD, 0, sizeof(fmpuD));
}

void MPU9250_DMP_enh::handleIRQ(){
    readMPU();
    // Check for new data in the FIFO
    if ( fifoAvailable() )
    {
        // DMP FIFO must be updated in order to update tap data
        dmpUpdateFifo();
        // Check for new tap data by polling tapAvailable
        if ( tapAvailable() )
        {
            // If a new tap happened, get the direction and count
            // by reading getTapDir and getTapCount
            tapDir = getTapDir();
            tapCnt = getTapCount();
        }
    }
}

void MPU9250_DMP_enh::updateTime(){
  now = micros();

  // Set integration time by time elapsed since last filter update
  deltat = ((now - lastUpdate) / 1000000.0f);
  lastUpdate = now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
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

    // Must be called before updating quaternions!
    updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    MahonyQuaternionUpdate(fmpuD[IMUa][0], fmpuD[IMUa][1], fmpuD[IMUa][2],
                            fmpuD[IMUg][0], fmpuD[IMUg][1], fmpuD[IMUg][2],
                            fmpuD[IMUm][1], fmpuD[IMUm][0], fmpuD[IMUm][2],
                            deltat);

    newData = true;
}

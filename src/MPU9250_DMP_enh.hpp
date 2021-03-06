#ifndef MPU9250_DMP_ENH_HPP
#define MPU9250_DMP_ENH_HPP

#include "SparkFunMPU9250-DMP.h"

#define IMUa    0       // accel
#define IMUg    1       // gyro
#define IMUm    2       // magnetometer

class MPU9250_DMP_enh : public MPU9250_DMP {
public:
    MPU9250_DMP_enh();
    boolean isDataAvailable() { boolean b = newData; newData = false; return b;};
    short handleIRQ();   // handle IRQs from MPU9250
    short handleFIFOIRQ();
    short calAccelGyro(float*, float*);

    void MahonyQuaternionUpdate(float, float, float, float, float, float, float, float, float);
    void MadgwickQuaternionUpdate(float, float, float, float, float, float, float, float, float);

    void calcEulerAngles(float f = 0.0f);

    void magcalMPU9250();

    void getMagCalData(float* bi, float* sc){
        for( int i = 0; i < 3; i++ ){
            bi[i] = magBias[i];
            sc[i] = magScale[i];
        }
    }

    void setMagCalData(float* bi, float* sc){
        for( int i = 0; i < 3; i++ ){
            magBias[i] = bi[i];
            magScale[i] = sc[i];
        }
    }

    float getUpdateRate(){
//        return (float)sumCount/sum;
        return 1.0f/deltat;
    }
    // array to hold acc, gyro and magneto data
    float fmpuD[3][3];
    // array to hold Euler angles
    float fEuler[3];
    // Vector to hold quaternion
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    int tapDir, tapCnt;

private:
    void readMPU();     // read accel, gyro and magneto data
    void updateTime();

    boolean newData;
    float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    uint32_t now = 0;        // used to calculate integration interval
    uint32_t sumCount = 0;        // used to calculate integration interval

    // Vector to hold integral error for Mahony method
    float eInt[3] = {0.0f, 0.0f, 0.0f};

    // vectors to hold user magnetic field correction
    float magBias[3];
    float magScale[3];
};

#endif

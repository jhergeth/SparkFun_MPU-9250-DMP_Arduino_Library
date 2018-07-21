#ifndef MPU9250_DMP_ENH_HPP
#define MPU9250_DMP_ENH_HPP

#include "SparkFunMPU9250-DMP.h"

#define IMUa    0       // accel
#define IMUg    1       // gyro
#define IMUm    2       // magnetometer
#define IMUw    3       // euler angles

class MPU9250_DMP_enh : MPU9250_DMP {
public:
    MPU9250_DMP_enh();
    boolean isDataAvailable() { boolean b = newData; newData = false; return b;};
    void handleIRQ();   // handle IRQs from MPU9250

    float fmpuD[4][3];
    int tapDir, tapCnt;
private:
    void readMPU();     // read accel, gyro and magneto data
    void updateTime();

    boolean newData;
    float deltat = 0.0f, sum = 0.0f;  // integration interval for both filter schemes
    uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
    uint32_t now = 0;        // used to calculate integration interval
    uint32_t sumCount = 0;        // used to calculate integration interval


};

#endif

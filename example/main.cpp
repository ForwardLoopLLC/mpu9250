#include "mpu9250/mpu9250.h"

int main() {
    MPU9250 mpu9250(0);
    if (mpu9250.error()) {
        printf("ERROR: MPU9250 did not initialize\n");
    }
    printf("MPU9250 initialized\n");
    if(!mpu9250.reset()) {
        printf("ERROR: reset failed\n");
    }
    if(!mpu9250.disableSleepMode()) {
        printf("ERROR: failed to disable sleep mode\n");
    }
    if(!mpu9250.useBestClock()) {
        printf("ERROR: failed to use best clock\n");
    }
    if(!mpu9250.setGyroscopeLowPassFilterFrequency200Hz()) {
        printf("ERROR: failed to set gyroscope low pass filter frequency\n");
    }
    if(!mpu9250.setAccelerometerGyroscopeSampleFrequency200Hz()) {
        printf("ERROR: failed to set accelerometer and gyroscope sample frequency\n");
    }
    if(!mpu9250.setGyroscopeScale2000DPS()) {
        printf("ERROR: failed to set gyroscope scale\n");
    }
    if(!mpu9250.setAccelerometerScale16G()) {
        printf("ERROR: failed to set accelerometer scale\n");
    }
    if(!mpu9250.setAccelerometerBandwidth45Hz()) {
        printf("ERROR: failed to set accelerometer bandwidth\n");
    }
    if(!mpu9250.enableMagnetometer()) {
        printf("ERROR: failed to enable magnetometer\n");
    }
    if(!mpu9250.enableDataReady()) {
        printf("ERROR: failed to enable data ready interrupt\n");
    }
    double sensitivity[3];
    if(!mpu9250.magnetometerSensitivity(sensitivity)) {
         printf("ERROR: failed to read magnetometer sensitivity\n");
    }
    for(int i=0; i<3; i++){
        printf("%f ", sensitivity[i]);
    }
    printf("\n");
    if(!mpu9250.setMagnetometerScale14Bits()) {
        printf("ERROR: failed to set magnetometer scale\n");
    }
    if(!mpu9250.setMagnetometerSampleRate100Hz()) {
        printf("ERROR: failed to set magnetometer rate\n");
    }
    mpu9250.wait(1000000);
    double acceleration[3];
    double rotationRate[3];
    double magneticField[3];
    for (int i=0; i<5; i++) {
        if(mpu9250.dataReady()) {
            mpu9250.readAcceleration(acceleration);
            mpu9250.readRotationRate(rotationRate);
            mpu9250.readMagneticField(magneticField);
            mpu9250.wait(1000000);
            printf("Acceleration\n");
            for (int j=0; j<3; j++) {
                printf("%f ", acceleration[j]);
            }
            printf("\n");
            printf("Rotation Rate\n");
            for (int j=0; j<3; j++) {
                printf("%f ", rotationRate[j]);
            }
            printf("\n");
            printf("Magnetic Field\n");
            for (int j=0; j<3; j++) {
                printf("%f ", magneticField[j]);
            }
            printf("\n");
        }
    }
    return 0;
}

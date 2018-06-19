#include "mpu9250/mpu9250.h"

int main() {
    MPU9250 mpu9250(0);
    if (mpu9250.error()) {
        printf("ERROR: MPU9250 did not initialize\n");
    }
    printf("MPU9250 initialized\n");
    if(!mpu9250.setAccelerometerScale2G()) {
        printf("ERROR: failed to set accelerometer scale\n");
    }
    if(!mpu9250.setAccelerometerBandwidth45Hz()) {
        printf("ERROR: failed to set accelerometer bandwidth\n");
    }
    if(!mpu9250.setGyroscopeScale250DPS()) {
        printf("ERROR: failed to set gyroscope scale\n");
    }
    if(!mpu9250.setGyroscopeLowPassFilterFrequency200Hz()) {
        printf("ERROR: failed to set gyroscope LP frequency\n");
    }
    if(!mpu9250.setAccelerometerGyroscopeSampleFrequency200Hz()) {
        printf("ERROR: failed to set accelerometer/gyroscope sample frequency\n");
    }
    if(!mpu9250.enableDataReady()) {
        printf("ERROR: failed to enable data ready interrupt\n");
    }
    if(!mpu9250.enableMagnetometer()) {
        printf("ERROR: failed to enable magnetometer\n");
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
    return 0;
}

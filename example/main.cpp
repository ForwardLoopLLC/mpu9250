#include "mpu9250/mpu9250.h" // drivers
#include "mpu9250/quaternion.h" //madgwickQuaternionUpdate
#include <sys/time.h>

double now() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    double ms = tp.tv_sec + tp.tv_usec/1000000.0;
    return ms;
}

int main() {
    MPU9250 mpu9250(0);
    if (mpu9250.error()) {
        printf("ERROR: MPU9250 did not initialize\n");
    }
    printf("MPU9250 initialized\n");
    if(!mpu9250.reset()) {
        printf("ERROR: reset failed\n");
    }
    if(!mpu9250.becomeSlave()) {
        printf("ERROR: failed to become slave device\n");
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
    if(!mpu9250.setAccelerometerGyroscopeSampleFrequency10Hz()) {
        printf("ERROR: failed to set accelerometer and gyroscope sample frequency\n");
    }
    if(!mpu9250.setGyroscopeScale250DPS()) {
        printf("ERROR: failed to set gyroscope scale\n");
    }
    if(!mpu9250.setAccelerometerScale2G()) {
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
    double magnetometerSensitivity[3];
    if(!mpu9250.magnetometerSensitivity(magnetometerSensitivity)) {
         printf("ERROR: failed to read magnetometer magnetometerSensitivity\n");
    }
    if(!mpu9250.setMagnetometerScale14Bits()) {
        printf("ERROR: failed to set magnetometer scale\n");
    }
    if(!mpu9250.setMagnetometerSampleRate100Hz()) {
        printf("ERROR: failed to set magnetometer rate\n");
    }
    double acceleration[3] = {0.0};
    double rotationRate[3] = {0.0};
    double magneticField[3] = {0.0};
    double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
    double lastUpdate = 0.0;
    int sample = 0;
    while(true) {
        if(mpu9250.dataReady()) {
            // collect data samples and compute Madgwick quaternions while
            // waiting for new data to become available
            // (more iterations == more accurate orientation)
            // write and overwrite data to a file
            if(sample > 1) {
                FILE* fid = fopen("/floop/quaternion", "w");
                if (fid != NULL) {
                    fprintf(fid,
                            "%1.6f,%1.6f,%1.6f,%1.6f\n",
                            quaternion[0],
                            quaternion[1],
                            quaternion[2],
                            quaternion[3]);
                    fflush(fid);
                    fclose(fid);
                }
            }
            // read data
            mpu9250.readAccelerationAndRotationRate(acceleration, rotationRate);
            if(mpu9250.magnetometerReady()){
                mpu9250.readMagneticField(magneticField);
                for(int i=0; i<3; i++) {
                    magneticField[i] *= magnetometerSensitivity[i];
                }
            }
            sample++;
            if (sample > 100) {
                break;
            }
        }
        double deltat = now() - lastUpdate;
        lastUpdate = now();
        // calculate most accurate quaternion while waiting for new data
        if(sample > 1){
            // gets about ~2000 iterations per data read on a Forward Loop Zero
            if(!madgwickQuaternionUpdate(quaternion, acceleration, rotationRate, magneticField, deltat)) {
                printf("Madgwick failed!\n");
            }
        }
    }
    return 0;
}

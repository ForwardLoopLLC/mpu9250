#ifndef MPU9250_H
#define MPU9250_H
#include <math.h>
#include "i2c/i2c.h"
#define MPU9250_I2CADDR 0x68
#define AK8963_I2CADDR 0x0C
// Registers
//// Acceleration 
////// Organized as X, Y, Z
////// (6 sequential 8-bit registers 
////// to be read as 3 signed 16-bit sequential registers)
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
// Power Management 1
#define MPU9250_PWR_MGMT_1 0x6B
//// Power management 1 modes
enum class MPU9250_PWR_MGMT_1_MODE {
    // sleep mode is enabled by default
    DISABLE_SLEEP = 0x00,
    ENABLE_SLEEP = 0x40,
    // 20MHz internal clock
    INTERNAL_CLOCK = 0x00,
    // use PLL if available, else internal clock
    BEST_CLOCK = 0x01,
    // reset all registers to factory defaults
    RESET = 0x80
};
#define MPU9250_CONFIG 0x1A
enum class MPU9250_CONFIG_MODE {
    _200HZ = 0x03
};
#define MPU9250_SMPLRT_DIV 0x19
enum class MPU9250_SMPLRT_DIV_MODE {
    _200HZ = 0x04,
    _100HZ = 0x09,
    _50HZ = 0x31,
	_10HZ = 0x63
};
#define MPU9250_GYRO_CONFIG 0x1B
enum class MPU9250_GYRO_CONFIG_MODE {
    _250DPS = 0x00,
    _500DPS = 0x01,
    _1000DPS = 0x02,
    _2000DPS = 0x03
};

#define MPU9250_ACCEL_CONFIG 0x1C
enum class MPU9250_ACCEL_CONFIG_MODE {
    _2G = 0x00,
    _4G = 0x01,
    _8G = 0x02,
    _16G = 0x03
};

#define MPU9250_ACCEL_CONFIG2 0x1D
enum class MPU9250_ACCEL_CONFIG2_MODE {
    _45HZ = 0x03
};

#define MPU9250_I2C_MST_CTRL 0x24
enum class MPU9250_I2C_MST_CTRL_MODE {
    SLAVE = 0x00 //400kHz clock frequency
}; 

#define MPU9250_INT_PIN_CFG 0x37
enum class MPU9250_INT_PIN_CFG_MODE {
    ENABLE_BYPASS = 0x22
};

#define MPU9250_INT_ENABLE 0x38
enum class MPU9250_INT_ENABLE_MODE {
    ENABLED = 0x01
};
#define MPU9250_INT_STATUS 0x3A
#define MPU9250_USER_CTRL 0x6A
enum class MPU9250_USER_CTRL_MODE {
	DISABLE_MASTER = 0x00
};

#define AK8963_WHO_AM_I 0x00 
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_CNTL 0x0A
enum class AK8963_CNTL_MODE {
    POWER_DOWN = 0x00,
    FUSE_ROM = 0x0F,
    // scale bits get up-shifted 4 bits
    _14BITS = 0x00,
    _16BITS = 0x01,
    // rate bits do not get shifted
    _8HZ = 0x02,
    _100HZ = 0x06
};
#define AK8963_ASAX 0x10

class MPU9250 {
    public:
        //! Constructor initializes I2C resources
        /*!
            \param bus the I2C bus to which the MPU9250 sensor is connected 
        */
        MPU9250(const uint8_t bus);
        //! Destructor cleans up I2C resources
        ~MPU9250();
        //! Pause main thread execution
        /*!
            \param delay time in milliseconds to pause 
        */
        void wait(const int delay);
        //! Check if there was an error during object construction
        /*!
            \return True, if constructor failed. False, if succeeded.
        */ 
        bool error();
        //! Reset all registers
        /*!
            \return False, if reset failed. True, if succeeded.
        */
        bool reset();
        //! Disable sensor sleep mode 
        /*!
            \return False, if disable failed. True, if succeeded.
        */
        bool disableSleepMode();
        //! Enable sensor sleep mode 
        /*!
            \return False, if enable failed. True, if succeeded.
        */
        bool enableSleepMode();
        //! Use internal 20MHz clock 
        /*!
            \return False, if enable failed. True, if succeeded.
        */
        bool useInternalClock();
        //! If available, use phase-locked loop (PLL) clock. Otherwise, use internal 20MHz clock
        /*!
            \return False, if enable failed. True, if succeeded.
        */
        bool useBestClock();
        //! Set gyroscope low-pass maximum delay to 4.9 milliseconds, which corresponds to a maximum sample rate of roughly 200Hz 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setGyroscopeLowPassFilterFrequency200Hz();
        //! Set sample frequency to 10Hz. If called with `enableDataReady()`, this causes `dataReady()` to return true roughly 10 times per second 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerGyroscopeSampleFrequency10Hz();
        //! Set sample frequency to 50Hz. If called with `enableDataReady()`, this causes `dataReady()` to return true roughly 50 times per second 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerGyroscopeSampleFrequency50Hz();
        //! Set sample frequency to 100Hz. If called with `enableDataReady()`, this causes `dataReady()` to return true roughly 100 times per second 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerGyroscopeSampleFrequency100Hz();
        //! Set sample frequency to 200Hz. If called with `enableDataReady()`, this causes `dataReady()` to return true roughly 200 times per second 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerGyroscopeSampleFrequency200Hz();
        //! Set gyroscope maximum scale to ±250°/s 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setGyroscopeScale250DPS();
        //! Set gyroscope maximum scale to ±500°/s 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setGyroscopeScale500DPS();
        //! Set gyroscope maximum scale to ±1000°/s 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setGyroscopeScale1000DPS();
        //! Set gyroscope maximum scale to ±2000°/s 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setGyroscopeScale2000DPS();
        //! Set accelerometer maximum scale to ±2g
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerScale2G();
        //! Set accelerometer maximum scale to ±4g
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerScale4G();
        //! Set accelerometer maximum scale to ±8g 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerScale8G();
        //! Set accelerometer maximum scale to ±16g 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerScale16G();
        //! Set accelerometer bandwidth to 44.8Hz
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setAccelerometerBandwidth45Hz();
        //! Turn on AK8963 magnetometer so it is accessible from the I2C bus.
        /*!
            \return False, if enable failed. True, if succeeded.
        */
        bool enableMagnetometer();
        //! Make MPU9250 and AK8963 act as slave, not master
        /*!
            \return False, if enable failed. True, if succeeded.
        */
        bool becomeSlave();
        //! Enable interrupt that signals when new data is available. This causes `dataReady()` to return true at the accelerometer and gyroscope sample frequency 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool enableDataReady();
        //! Signal that new accelerometer and gyroscope data is available
        /*!
            \return True, if data available. False, if data is stale. 
        */
        bool dataReady();
        //! Signal that new magnetometer data is available
        /*!
            \return True, if data available. False, if data is stale. 
        */
        bool magnetometerReady();
        //! Read factory calibration for magnetometer sensitivity
        /*!
            \param sensitivity magnetometer sensitivity in the x, y, and z directions.
            \return True, if `sensitivity` contains the magnetometer sensitivity values. False, if `sensitivity` contains meaningless data. 
        */
        bool magnetometerSensitivity(double sensitivity[3]);
        //! Set magnetometer scale to 14 bits of accuracy 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setMagnetometerScale14Bits();
        //! Set magnetometer scale to 16 bits of accuracy 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setMagnetometerScale16Bits();
        //! Set magnetometer output data rate to 8Hz 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setMagnetometerSampleRate8Hz();
        //! Set magnetometer output data rate to 100Hz 
        /*!
            \return False, if set failed. True, if succeeded.
        */
        bool setMagnetometerSampleRate100Hz();
        //! Read acceleration values into an array. 
        /*! This will read the accelerometer data even if it is stale. Always check `dataReady()` to ensure data is up-to-date and accurate.

            This method only reads the accelerometer data. For applications that need acceleration and rotation rate values, consider using the faster `readAccelerationAndRotationRate()` method.

            \param acceleration array to hold acceleration in the x, y, and z directions in units of gravitational acceleration.
            \return False, if read failed. True, if succeeded.
        */
        bool readAcceleration(double acceleration[3]);
        //! Read rotation rate values into an array. 
        /*! This will read the gyroscope data even if it is stale. Always check `dataReady()` to ensure data is up-to-date and accurate.

            This method only reads the gyroscope data. For applications that need acceleration and rotation rate values, consider using the faster `readAccelerationAndRotationRate()` method.

            \param rotationRate array to hold rotation rate in the x, y, and z directions in units of °/s
            \return False, if read failed. True, if succeeded.
        */
        bool readRotationRate(double rotationRate[3]);
        //! Read acceleration and rotation rate values into arrays. 
        /*! This will read the accelerometer and gyroscope data even if it is stale. Always check `dataReady()` to ensure data is up-to-date and accurate.

            For applications that need acceleration and rotation rate values, this method is faster than calling `readAcceleration()` and `readRotationRate()` one after the other. 

            \param acceleration array to hold acceleration in the x, y, and z directions in units of gravitational acceleration.
            \param rotationRate array to hold rotation rate in the x, y, and z directions in units of °/s
            \return False, if read failed. True, if succeeded.
        */
		bool readAccelerationAndRotationRate(double acceleration[3], double rotationRate[3]);
        //! Read magnetic field values into an array. 
        /*! This will read the magnetometer data even if it is stale. Always check `magnetometerReady()` to ensure data is up-to-date and accurate.
            \param magneticField array to hold magnetic field in the x, y, and z directions in units of milliGauss 
            \return False, if read failed. True, if succeeded.
        */
        bool readMagneticField(double magneticField[3]);
    private:
        I2C mpu9250I2C;
        I2C* ak8963I2C;
        const uint8_t bus;
        bool err = false;
        bool magnetometerEnabled = false;
        double accelerometerResolution = 0.0;
        double gyroscopeResolution = 0.0;
        double magnetometerResolution = 0.0;
        bool checkMagnetometer();
        bool readAccelerometerGyroscopeBytes(const uint8_t reg, const uint8_t len, uint8_t* bytes);
        bool readMagnetometerBytes(const uint8_t reg, const uint8_t len, uint8_t* bytes);
        bool setGyroscopeScale(const uint8_t scale);
        bool setAccelerometerScale(const uint8_t scale);
        bool setAccelerometerBandwidth(const uint8_t bandwidth);
        bool enableI2CBypass();
        bool setMagnetometerScale(const uint8_t scale);
        bool setMagnetometerSampleRate(const uint8_t rate);
};

MPU9250::MPU9250(const uint8_t bus) : bus(bus), mpu9250I2C(bus, MPU9250_I2CADDR) {
    if(mpu9250I2C.error()) {
        err = true;
    }
}

MPU9250::~MPU9250() {
    if(magnetometerEnabled) {
        delete ak8963I2C;
    }
}

void MPU9250::wait(const int delay) {
    mpu9250I2C.wait(delay);
}

bool MPU9250::error() {
    return err;
}

bool MPU9250::readAccelerometerGyroscopeBytes(const uint8_t reg, const uint8_t len, uint8_t* bytes) {
	return mpu9250I2C.read_block(reg, len, bytes);
}

bool MPU9250::readMagnetometerBytes(const uint8_t reg, const uint8_t len, uint8_t* bytes) {
	return ak8963I2C->read_block(reg, len, bytes);
}

bool MPU9250::reset() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::RESET)) {
        return false;
    }
    wait(10000);
    return true;
}

bool MPU9250::disableSleepMode() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::DISABLE_SLEEP)) {
        return false;
    }
    wait(10000);
    return true;
}

bool MPU9250::enableSleepMode() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::ENABLE_SLEEP)) {
        return false;
    }
    return true;
}

bool MPU9250::useInternalClock() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::INTERNAL_CLOCK)) {
        return false;
    }
    return true;
}
bool MPU9250::useBestClock() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::BEST_CLOCK)) {
        return false;
    }
    return true;
}

bool MPU9250::setGyroscopeLowPassFilterFrequency200Hz() {
    if(!mpu9250I2C.write_byte(MPU9250_CONFIG, (uint8_t)MPU9250_CONFIG_MODE::_200HZ)) {
        return false;
    }
    return true;
}

bool MPU9250::setAccelerometerGyroscopeSampleFrequency200Hz() {
    if(!mpu9250I2C.write_byte(MPU9250_SMPLRT_DIV, (uint8_t)MPU9250_SMPLRT_DIV_MODE::_200HZ)) {
        return false;
    }
    return true;
}

bool MPU9250::setAccelerometerGyroscopeSampleFrequency100Hz() {
    if(!mpu9250I2C.write_byte(MPU9250_SMPLRT_DIV, (uint8_t)MPU9250_SMPLRT_DIV_MODE::_100HZ)) {
        return false;
    }
    return true;
}

bool MPU9250::setAccelerometerGyroscopeSampleFrequency50Hz() {
    if(!mpu9250I2C.write_byte(MPU9250_SMPLRT_DIV, (uint8_t)MPU9250_SMPLRT_DIV_MODE::_50HZ)) {
        return false;
    }
    return true;
}

bool MPU9250::setAccelerometerGyroscopeSampleFrequency10Hz() {
    if(!mpu9250I2C.write_byte(MPU9250_SMPLRT_DIV, (uint8_t)MPU9250_SMPLRT_DIV_MODE::_10HZ)) {
        return false;
    }
    return true;
}

bool MPU9250::setGyroscopeScale(const uint8_t scale) {
    uint8_t gyroConfig = mpu9250I2C.read_byte(MPU9250_GYRO_CONFIG);
    // clear Fchoice
    gyroConfig &= ~0x02;
    // clear AFS
    gyroConfig &= ~0x18;
    // set scale
    gyroConfig |= scale << 3;
    if(!mpu9250I2C.write_byte(MPU9250_GYRO_CONFIG, gyroConfig)) {
        return false;
    }
    gyroscopeResolution = 250.0*(scale+1)/32768.0;
    return true;
} 

bool MPU9250::setGyroscopeScale250DPS() {
    return setGyroscopeScale((uint8_t)MPU9250_GYRO_CONFIG_MODE::_250DPS);
} 

bool MPU9250::setGyroscopeScale500DPS() {
    return setGyroscopeScale((uint8_t)MPU9250_GYRO_CONFIG_MODE::_500DPS);
} 

bool MPU9250::setGyroscopeScale1000DPS() {
    return setGyroscopeScale((uint8_t)MPU9250_GYRO_CONFIG_MODE::_1000DPS);
} 

bool MPU9250::setGyroscopeScale2000DPS() {
    return setGyroscopeScale((uint8_t)MPU9250_GYRO_CONFIG_MODE::_2000DPS);
} 

bool MPU9250::setAccelerometerScale(const uint8_t scale) {
    uint8_t accelConfig = mpu9250I2C.read_byte(MPU9250_ACCEL_CONFIG);
    // clear AFS
    accelConfig &= ~0x18;
    // set scale
    accelConfig |= scale << 3;
    if(!mpu9250I2C.write_byte(MPU9250_ACCEL_CONFIG, accelConfig)) {
        return false;
    }
    accelerometerResolution = pow(2, scale+1)/32768.0;
    return true;
}

bool MPU9250::setAccelerometerScale2G() {
    return setAccelerometerScale((uint8_t)MPU9250_ACCEL_CONFIG_MODE::_2G);
}

bool MPU9250::setAccelerometerScale4G() {
    return setAccelerometerScale((uint8_t)MPU9250_ACCEL_CONFIG_MODE::_4G);
}

bool MPU9250::setAccelerometerScale8G() {
    return setAccelerometerScale((uint8_t)MPU9250_ACCEL_CONFIG_MODE::_8G);
}

bool MPU9250::setAccelerometerScale16G() {
    return setAccelerometerScale((uint8_t)MPU9250_ACCEL_CONFIG_MODE::_16G);
}

bool MPU9250::setAccelerometerBandwidth(const uint8_t bandwidth) {
    uint8_t accelConfig2 = mpu9250I2C.read_byte(MPU9250_ACCEL_CONFIG2);
    // clear Fchoice
    accelConfig2 &= ~0x0F;
    // set scale
    accelConfig2 |= bandwidth;
    if(!mpu9250I2C.write_byte(MPU9250_ACCEL_CONFIG2, accelConfig2)) {
        return false;
    }
    return true;
}

bool MPU9250::setAccelerometerBandwidth45Hz() {
    return setAccelerometerBandwidth((uint8_t)MPU9250_ACCEL_CONFIG2_MODE::_45HZ); 
}

bool MPU9250::enableI2CBypass() {
    if(!mpu9250I2C.write_byte(MPU9250_INT_PIN_CFG, (uint8_t)MPU9250_INT_PIN_CFG_MODE::ENABLE_BYPASS)) {
        return false;
    }
    return true;
}

bool MPU9250::becomeSlave() {
    if(!mpu9250I2C.write_byte(MPU9250_I2C_MST_CTRL, (uint8_t)MPU9250_I2C_MST_CTRL_MODE::SLAVE)) {
        return false;
    }
    if(!mpu9250I2C.write_byte(MPU9250_USER_CTRL, (uint8_t)MPU9250_USER_CTRL_MODE::DISABLE_MASTER)) {
        return false;
    }
    return true;
}

bool MPU9250::enableDataReady() {
    if(!mpu9250I2C.write_byte(MPU9250_INT_ENABLE, (uint8_t)MPU9250_INT_ENABLE_MODE::ENABLED)) {
        return false;
    }
    return true;
}

bool MPU9250::dataReady() {
    return mpu9250I2C.read_byte(MPU9250_INT_STATUS) & 0x01;
}

bool MPU9250::enableMagnetometer() {
    if(!enableI2CBypass()) {
        return false;
    }
    ak8963I2C = new I2C(bus, AK8963_I2CADDR); 
    if(ak8963I2C->error()) {
        return false;
    }
    if(!checkMagnetometer()) {
        return false;
    }
    magnetometerEnabled = true;
    return true;
}

bool MPU9250::magnetometerSensitivity(double sensitivity[3]) {
    if(!magnetometerEnabled) {
        return false;
    }
    if(!ak8963I2C->write_byte(AK8963_CNTL, (uint8_t)AK8963_CNTL_MODE::POWER_DOWN)) {
        return false;
    }
    wait(10);
    if(!ak8963I2C->write_byte(AK8963_CNTL, (uint8_t)AK8963_CNTL_MODE::FUSE_ROM)) {
        return false;
    }
    wait(10);
    uint8_t sensitivityBuffer[3];
    readAccelerometerGyroscopeBytes(AK8963_ASAX, 3, sensitivityBuffer);
    for (int i=0; i<3; i++){
        sensitivity[i] = (double)(sensitivityBuffer[i] - 128)/256.0 + 1.0;
    }
    if(!ak8963I2C->write_byte(AK8963_CNTL, (uint8_t)AK8963_CNTL_MODE::POWER_DOWN)) {
        return false;
    }
    wait(10);
    return true;
} 

bool MPU9250::setMagnetometerScale(const uint8_t scale) {
    if(!magnetometerEnabled) {
        return false;
    }
    uint8_t magConfig = ak8963I2C->read_byte(AK8963_CNTL);
    magConfig &= 0xF0;
    magConfig |= scale << 4;
    if(!ak8963I2C->write_byte(AK8963_CNTL, magConfig)) {
        return false;
    }
    magnetometerResolution = 49120.0/8190.0;
	if (scale) {
		magnetometerResolution = 49120.0/32760.0;
	}
    return true;
}

bool MPU9250::setMagnetometerScale14Bits() {
    return setMagnetometerScale((uint8_t)AK8963_CNTL_MODE::_14BITS);
}

bool MPU9250::setMagnetometerScale16Bits() {
    return setMagnetometerScale((uint8_t)AK8963_CNTL_MODE::_16BITS);
}

bool MPU9250::setMagnetometerSampleRate(const uint8_t rate) {
    if(!magnetometerEnabled) {
        return false;
    }
    uint8_t magConfig = ak8963I2C->read_byte(AK8963_CNTL);
    magConfig &= 0xF;
    magConfig |= rate;
    if(!ak8963I2C->write_byte(AK8963_CNTL, magConfig)) {
        return false;
    }
    return true;
}

bool MPU9250::setMagnetometerSampleRate8Hz() {
    return setMagnetometerSampleRate((uint8_t)AK8963_CNTL_MODE::_8HZ);
}

bool MPU9250::setMagnetometerSampleRate100Hz() {
    return setMagnetometerSampleRate((uint8_t)AK8963_CNTL_MODE::_100HZ);
}

bool MPU9250::checkMagnetometer() {
    return ak8963I2C->read_byte(AK8963_WHO_AM_I) == 0x48;
}

bool MPU9250::readAcceleration(double acceleration[3]) {
    uint8_t accelData[6];
    readAccelerometerGyroscopeBytes(MPU9250_ACCEL_XOUT_H, 6, accelData);
    for (int i=0; i<3; i++) {
        acceleration[i] = accelerometerResolution * (double)(int16_t)(((int16_t)accelData[2*i] << 8) | accelData[2*i+1]);
    }
    return true;
}

bool MPU9250::readRotationRate(double rotationRate[3]) {
    uint8_t gyroData[6];
    readAccelerometerGyroscopeBytes(MPU9250_GYRO_XOUT_H, 6, gyroData);
    for (int i=0; i<3; i++) {
        rotationRate[i] = gyroscopeResolution * (double)(int16_t)((int16_t)gyroData[2*i] << 8 | gyroData[2*i+1]);
    }
    return true;
}

bool MPU9250::readAccelerationAndRotationRate(double acceleration[3], double rotationRate[3]) {
	uint8_t data[14];
    readAccelerometerGyroscopeBytes(MPU9250_ACCEL_XOUT_H, 14, data);
    for (int i=0; i<3; i++) {
        acceleration[i] = accelerometerResolution * (double)(int16_t)(((int16_t)data[2*i] << 8) | data[2*i+1]);
    }
    for (int i=0; i<3; i++) {
        rotationRate[i] = gyroscopeResolution * (double)(int16_t)(((int16_t)data[2*i+6] << 8) | data[2*i+1+6]);
    }
	return true;
}

bool MPU9250::readMagneticField(double magneticField[3]) {
    uint8_t magData[6];
    readMagnetometerBytes(AK8963_XOUT_L, 6, magData);
    for (int i=0; i<3; i++) {
        magneticField[i] = magnetometerResolution * (double)(int16_t)((int16_t)magData[2*i+1] << 8 | magData[2*i]);
    }
    return true;
}

bool MPU9250::magnetometerReady() {
	return ak8963I2C->read_byte(AK8963_ST1) & 0x01;
}

#endif

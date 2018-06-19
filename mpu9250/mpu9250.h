#ifndef MPU9250_H
#define MPU9250_H
#include "i2c/i2c.h"
#define MPU9250_I2CADDR 0x68
#define AK8963_I2CADDR 0x0C
// Registers
//// Acceleration 
////// Organized as X, Y, Z
////// (6 sequential 8-bit registers 
////// to be read as 3 signed 16-bit sequential registers)
#define MPU9250_ACCEL_XOUT_H 0x3B
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
    _200HZ = 0x03
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
    SLAVE = 0x00
}; 

#define MPU9250_INT_PIN_CFG 0x37
enum class MPU9250_INT_PIN_CFG_MODE {
    ENABLE_BYPASS = 0x22
};

#define MPU9250_INT_ENABLE 0x38
enum class MPU9250_INT_ENABLE_MODE {
    ENABLED = 0x01
};

#define AK8963_WHO_AM_I 0x00 
#define AK8963_CNTL 0x0A
enum class AK8963_CNTL_MODE {
    POWER_DOWN = 0x00,
    FUSE_ROM = 0x0F,
    // scale bits get up-shifted 4 bits
    _14BITS = 0x00,
    _16BITS = 0x01,
    // rate bits do not get shifted
    _100HZ = 0x06
};
#define AK8963_ASAX 0x10

class MPU9250 {
    public:
        MPU9250(const uint8_t bus);
        ~MPU9250();
        void wait(const int delay);
        bool error();
        bool reset();
        bool disableSleepMode();
        bool enableSleepMode();
        bool useInternalClock();
        bool useBestClock();
        bool setGyroscopeLowPassFilterFrequency200Hz();
        bool setAccelerometerGyroscopeSampleFrequency200Hz();
        bool setGyroscopeScale250DPS();
        bool setGyroscopeScale500DPS();
        bool setGyroscopeScale1000DPS();
        bool setGyroscopeScale2000DPS();
        bool setAccelerometerScale2G();
        bool setAccelerometerScale4G();
        bool setAccelerometerScale8G();
        bool setAccelerometerScale16G();
        bool setAccelerometerBandwidth45Hz();
        bool enableMagnetometer();
        bool becomeSlave();
        bool enableDataReady();
        bool magnetometerSensitivity(double sensitivity[3]);
        bool setMagnetometerScale14Bits();
        bool setMagnetometerSampleRate100Hz();
    private:
        I2C mpu9250I2C;
        I2C* ak8963I2C;
        const uint8_t bus;
        bool err = false;
        bool magnetometerEnabled = false;
        bool checkMagnetometer();
        bool read_bytes(const uint8_t reg, const uint8_t len, uint8_t* bytes);
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

bool MPU9250::read_bytes(const uint8_t reg, const uint8_t len, uint8_t* bytes) {
    for(int i=0; i < len; i++) {
        bytes[i] = mpu9250I2C.read_byte(reg + i); 
    }
    return true;
}

bool MPU9250::reset() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::RESET)) {
        return false;
    }
    return true;
}

bool MPU9250::disableSleepMode() {
    if(!mpu9250I2C.write_byte(MPU9250_PWR_MGMT_1, (uint8_t)MPU9250_PWR_MGMT_1_MODE::DISABLE_SLEEP)) {
        return false;
    }
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
    return true;
}

bool MPU9250::enableDataReady() {
    if(!mpu9250I2C.write_byte(MPU9250_INT_ENABLE, (uint8_t)MPU9250_INT_ENABLE_MODE::ENABLED)) {
        return false;
    }
    return true;
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
    read_bytes(AK8963_ASAX, 3, sensitivityBuffer);
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
        printf("c\n");
        return false;
    }
    return true;
}

bool MPU9250::setMagnetometerScale14Bits() {
    return setMagnetometerScale((uint8_t)AK8963_CNTL_MODE::_14BITS);
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

bool MPU9250::setMagnetometerSampleRate100Hz() {
    return setMagnetometerSampleRate((uint8_t)AK8963_CNTL_MODE::_100HZ);
}

bool MPU9250::checkMagnetometer() {
    return ak8963I2C->read_byte(AK8963_WHO_AM_I) == 0x48;
}

#endif

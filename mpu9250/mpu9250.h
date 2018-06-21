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
        bool setAccelerometerGyroscopeSampleFrequency10Hz();
        bool setAccelerometerGyroscopeSampleFrequency50Hz();
        bool setAccelerometerGyroscopeSampleFrequency100Hz();
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
        bool dataReady();
        bool testReadWord();
        bool magnetometerReady();
        bool magnetometerSensitivity(double sensitivity[3]);
        bool setMagnetometerScale14Bits();
        bool setMagnetometerSampleRate100Hz();
        bool readAcceleration(double acceleration[3]);
        bool readRotationRate(double rotationRate[3]);
		bool readAccelerationAndRotationRate(double acceleration[3], double rotationRate[3]);
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
    printf("Gyro res: %f\n", gyroscopeResolution);
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

bool MPU9250::testReadWord() {
	uint8_t tmp[14];
	mpu9250I2C.read_block(MPU9250_ACCEL_XOUT_H, 14, tmp);
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
	if (!magnetometerReady()) {
		return false;
	}
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
#define PI 3.14159265358979323846
bool MadgwickQuaternionUpdate(
	double quaternion[4],
	double acceleration[3],
	double rotationRate[3],
	double magneticField[3],
	double deltat)
{
	double beta = 2.5;//sqrt(0.75)*PI/3.0;
	double q1 = quaternion[0], q2 = quaternion[1], q3 = quaternion[2], q4 = quaternion[3];
	double ax = acceleration[0], ay = acceleration[1], az = acceleration[2];
	double gx = (PI/180.0)*rotationRate[0], gy = (PI/180.0)*rotationRate[1], gz = (PI/180.0)*rotationRate[2];
	double mx = magneticField[0], my = magneticField[1], mz = magneticField[2];

	double _2q1 = 2.0 * q1;
	double _2q2 = 2.0 * q2;
	double _2q3 = 2.0 * q3;
	double _2q4 = 2.0 * q4;
	double _2q1q3 = 2.0 * q1 * q3;
	double _2q3q4 = 2.0 * q3 * q4;
	double q1q1 = q1 * q1;
	double q1q2 = q1 * q2;
	double q1q3 = q1 * q3;
	double q1q4 = q1 * q4;
	double q2q2 = q2 * q2;
	double q2q3 = q2 * q3;
	double q2q4 = q2 * q4;
	double q3q3 = q3 * q3;
	double q3q4 = q3 * q4;
	double q4q4 = q4 * q4;

	double norm = sqrt(ax * ax + ay * ay + az * az);
	if (abs(norm) < 1e-6) {
		return false;
	}
	norm = 1.0/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	norm = sqrt(mx * mx + my * my + mz * mz);
	if (abs(norm) < 1e-6) {
		return false;
	}
	norm = 1.0/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	double _2q1mx = 2.0 * q1 * mx;
	double _2q1my = 2.0 * q1 * my;
	double _2q1mz = 2.0 * q1 * mz;
	double _2q2mx = 2.0 * q2 * mx;
	double hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	double hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	double _2bx = sqrt(hx * hx + hy * hy);
	double _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	double _4bx = 2.0f * _2bx;
	double _4bz = 2.0f * _2bz;

	double s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	double s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	double s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	double s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax) + _2q3 * (2.0 * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); 
	norm = 1.0/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	double qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	double qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	double qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	double qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); 
	if (abs(norm) < 1e-6) {
		return false;
	}
	norm = 1.0/norm;
	quaternion[0] = q1 * norm;
	quaternion[1] = q2 * norm;
	quaternion[2] = q3 * norm;
	quaternion[3] = q4 * norm;
	return true;
}

#endif

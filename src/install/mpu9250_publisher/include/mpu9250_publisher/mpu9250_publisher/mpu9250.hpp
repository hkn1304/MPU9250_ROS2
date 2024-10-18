#ifndef MPU9250_H
#define MPU9250_H

#include <cstdint>

// Sensitivity scales
#define ACCEL_SENSITIVITY 16384.0
#define GYRO_SENSITIVITY  131.0
#define MAG_SENSITIVITY   4912.0 / 32760.0  // Assuming 16-bit resolution

class MPU9250 {
public:
    MPU9250(int i2c_bus, uint8_t address);

    bool initialize();
    bool readAccelData(/*int16_t& accelX, int16_t& accelY, int16_t& accelZ*/);
    bool readGyroData(/*int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ*/);
    bool readMagnetometerData(/*int16_t& magX, int16_t& magY, int16_t& magZ*/);

    int16_t getAccX() const {return _accelX;}; int16_t getAccY() const {return _accelY;}; int16_t getAccZ() const {return _accelZ;};
    int16_t getGyroX() const {return _gyroX;}; int16_t getGyroY() const {return _gyroY;}; int16_t getGyroZ() const {return _gyroZ;};
    int16_t getMagX() const {return _magX;}; int16_t getMagY() const {return _magY;}; int16_t getMagZ() const {return _magZ;};    

private:
    int i2c_fd;
    uint8_t device_address;

    int16_t _accelX; int16_t _accelY; int16_t _accelZ;
    int16_t _gyroX; int16_t _gyroY; int16_t _gyroZ;
    int16_t _magX; int16_t _magY; int16_t _magZ;

    bool readBytes(uint8_t reg, uint8_t count, uint8_t* dest);
    bool writeByte(uint8_t reg, uint8_t value);

    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
    static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
    static constexpr uint8_t ACCEL_XOUT_L = 0x3C;
    static constexpr uint8_t ACCEL_YOUT_L = 0x3E;
    static constexpr uint8_t ACCEL_ZOUT_L = 0x40;
    
    static constexpr uint8_t TEMP_OUT_H = 0x41;
    static constexpr uint8_t TEMP_OUT_L = 0x42;

    static constexpr uint8_t GYRO_XOUT_H = 0x43;
    static constexpr uint8_t GYRO_YOUT_H = 0x45;
    static constexpr uint8_t GYRO_ZOUT_H = 0x47;
    static constexpr uint8_t GYRO_XOUT_L = 0x44;
    static constexpr uint8_t GYRO_YOUT_L = 0x46;
    static constexpr uint8_t GYRO_ZOUT_L = 0x48;

    static constexpr uint8_t MAG_XOUT_H = 0x03;
    static constexpr uint8_t MAG_CNTL = 0x0A;


};

#endif // MPU9250_H

#include "mpu9250_publisher/mpu9250.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

MPU9250::MPU9250(int i2c_bus, uint8_t address) : device_address(address) {
    char filename[20];
    sprintf(filename, "/dev/i2c-%d", i2c_bus);
    i2c_fd = open(filename, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Failed to open I2C bus" << std::endl;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) {
        std::cerr << "Failed to connect to MPU9250" << std::endl;
    }
}

bool MPU9250::initialize() {
    // Initialize accelerometer, gyro, and magnetometer
    return writeByte(MAG_CNTL, 0x16); // Enable magnetometer
}

bool MPU9250::readAccelData(/*int16_t& accelX, int16_t& accelY, int16_t& accelZ*/) {
    uint8_t rawData[6];
    if (readBytes(ACCEL_XOUT_H, 6, rawData)) {
        _accelX = (rawData[0] << 8) | rawData[1];
        _accelY = (rawData[2] << 8) | rawData[3];
        _accelZ = (rawData[4] << 8) | rawData[5];
        return true;
    }
    return false;
}

bool MPU9250::readGyroData(/*int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ*/) {
    uint8_t rawData[6];
    if (readBytes(GYRO_XOUT_H, 6, rawData)) {
        _gyroX = (rawData[0] << 8) | rawData[1];
        _gyroY = (rawData[2] << 8) | rawData[3];
        _gyroZ = (rawData[4] << 8) | rawData[5];
        return true;
    }
    return false;
}

bool MPU9250::readMagnetometerData(/*int16_t& magX, int16_t& magY, int16_t& magZ*/) {
    uint8_t rawData[6];
    if (readBytes(MAG_XOUT_H, 6, rawData)) {
        _magX = (rawData[0] << 8) | rawData[1];
        _magY = (rawData[2] << 8) | rawData[3];
        _magZ = (rawData[4] << 8) | rawData[5];
        return true;
    }
    return false;
}

bool MPU9250::readBytes(uint8_t reg, uint8_t count, uint8_t* dest) {
    if (write(i2c_fd, &reg, 1) == 1) {
        if (read(i2c_fd, dest, count) == count) {
            return true;
        }
    }
    return false;
}

bool MPU9250::writeByte(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    if (write(i2c_fd, data, 2) == 2) {
        return true;
    }
    return false;
}

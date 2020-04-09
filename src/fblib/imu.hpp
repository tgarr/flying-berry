
#ifndef _FBIMU_HPP_
#define _FBIMU_HPP_

#include <pigpio.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>
#include "config.hpp"

#define I2C_BUS 1
#define MPU6050_I2C_ADDR 0x68

#define MPU6050_USER_CTRL 0x6A
#define MPU6050_USER_CTRL_VALUE 0x20 // disable DMP and FIFO, and enable master mode

#define MPU6050_FIFO 0x23
#define MPU6050_FIFO_VALUE 0x00 // disable FIFO

#define MPU6050_PWR_1 0x6B
#define MPU6050_PWR_RESET 0x80 // reset
#define MPU6050_PWR_WAKE 0x01 // wake and set X gyroscope clock

#define MPU6050_PWR_2 0x6C
#define MPU6050_PWR_2_VALUE 0x00 // disable standby

#define MPU6050_SAMPLE_RATE 0x19
#define MPU6050_SAMPLE_RATE_DIV 0x07 // 8kHz / (1 + SAMPLE_RATE_DIV)

#define MPU6050_CONFIG 0x1A
#define MPU6050_CONFIG_VALUE 0x00 // disable FSYNC and set DLPF
#define MPU6050_MAX_DLPF 6

#define MPU6050_I2C_MASTER_CLOCK 0x24
#define MPU6050_I2C_MASTER_CLOCK_VALUE 0x0D // 400 kHz

#define MPU6050_GYROCONFIG 0x1B
#define MPU6050_GYROCONFIG_VALUE 0x00 // gyroscope range 250
#define MPU6050_GYRO_SENSITIVITY 131

#define MPU6050_ACCELCONFIG 0x1C
#define MPU6050_ACCELCONFIG_VALUE 0x00 // accelerometer range 2g
#define MPU6050_ACCEL_SENSITIVITY 16384

#define MPU6050_ACCEL_X1 0x3B
#define MPU6050_ACCEL_X2 0x3C
#define MPU6050_ACCEL_Y1 0x3D
#define MPU6050_ACCEL_Y2 0x3E
#define MPU6050_ACCEL_Z1 0x3F
#define MPU6050_ACCEL_Z2 0x40

#define MPU6050_TEMP1 0x41
#define MPU6050_TEMP2 0x42

#define MPU6050_GYRO_X1 0x43
#define MPU6050_GYRO_X2 0x44
#define MPU6050_GYRO_Y1 0x45
#define MPU6050_GYRO_Y2 0x46
#define MPU6050_GYRO_Z1 0x47
#define MPU6050_GYRO_Z2 0x48

#define RAD_TO_DEGREE 57.29578f

typedef short int SensorDataType;
typedef SensorDataType SensorData[3];

class IMU {
    int handle;

    SensorData accel_data = {0,0,0};
    SensorData gyro_data = {0,0,0};

    SensorData accel_offsets = {0,0,0};
    SensorData gyro_offsets = {0,0,0};

    Angle* comp_angles;
    Angle* gyro_angles;
    Angle* accel_angles;
    
    void read_sensors();
    void complementary_filter(float);

public:
    bool ok = false;

    IMU();
    ~IMU();
    float temperature();
    void calibrate();
    void update(float);
    Angle* attitude();
    Angle* gyro();
    Angle* accel();
};

#endif



#include "imu.hpp"

IMU::IMU(){
    // connect to device
    handle = i2cOpen(I2C_BUS,MPU6050_I2C_ADDR,0);
    if(handle < 0){
        ok = false;
        return;
    }

    // wake
    i2cWriteByteData(handle,MPU6050_PWR_1,MPU6050_PWR_WAKE);
    usleep(200 * 1000); // 200ms

    // config
    int dlpf = fbconfig.dlpf_level;
    if(dlpf < 0) dlpf = 0;
    if(dlpf > MPU6050_MAX_DLPF) dlpf = MPU6050_MAX_DLPF;

    i2cWriteByteData(handle,MPU6050_USER_CTRL,MPU6050_USER_CTRL_VALUE); // user options
    i2cWriteByteData(handle,MPU6050_FIFO,MPU6050_FIFO_VALUE); // FIFO
    i2cWriteByteData(handle,MPU6050_PWR_2,MPU6050_PWR_2_VALUE); // disable standby
    i2cWriteByteData(handle,MPU6050_SAMPLE_RATE,MPU6050_SAMPLE_RATE_DIV); // sample rate
    i2cWriteByteData(handle,MPU6050_CONFIG,MPU6050_CONFIG_VALUE + dlpf); // FSYNC and DLPF
    i2cWriteByteData(handle,MPU6050_I2C_MASTER_CLOCK,MPU6050_I2C_MASTER_CLOCK_VALUE); // I2C master clock
    i2cWriteByteData(handle,MPU6050_GYROCONFIG,MPU6050_GYROCONFIG_VALUE); // gyroscope range
    i2cWriteByteData(handle,MPU6050_ACCELCONFIG,MPU6050_ACCELCONFIG_VALUE); // accelerometer range

    comp_angles = new Angle[3]{0,0,0};
    gyro_angles = new Angle[3]{0,0,0};
    accel_angles = new Angle[3]{0,0,0};

    ok = true;
}

IMU::~IMU(){
    i2cClose(handle);

    delete comp_angles;
    delete gyro_angles;
    delete accel_angles;
}

float IMU::temperature(){
    SensorDataType temp1 = i2cReadByteData(handle,MPU6050_TEMP1);
    SensorDataType temp2 = i2cReadByteData(handle,MPU6050_TEMP2);
    SensorDataType temp = (temp1 << 8) | temp2;
    return temp/340.0 + 36.53;
}

void IMU::read_sensors(){
    char data[14];
    i2cReadI2CBlockData(handle,MPU6050_ACCEL_X1,data,14);
    accel_data[X] = (((SensorDataType)data[0] << 8) | (SensorDataType)data[1]) - accel_offsets[X];
    accel_data[Y] = (((SensorDataType)data[2] << 8) | (SensorDataType)data[3]) - accel_offsets[Y];
    accel_data[Z] = ((SensorDataType)(data[4] << 8) | (SensorDataType)data[5]) - accel_offsets[Z];
    gyro_data[X] = ((SensorDataType)(data[8] << 8) | (SensorDataType)data[9]) - gyro_offsets[X];
    gyro_data[Y] = ((SensorDataType)(data[10] << 8) | (SensorDataType)data[11]) - gyro_offsets[Y];
    gyro_data[Z] = ((SensorDataType)(data[12] << 8) | (SensorDataType)data[13]) - gyro_offsets[Z];
}

void IMU::calibrate(){
    int accel_sum[3] = {0,0,0};
    int gyro_sum[3] = {0,0,0};
    int interval = 1.0f / fbconfig.looprate * 1000 * 1000;
    int n = fbconfig.calibration_time * 1000 * 1000 / interval;
    struct timeval st, et;

    // sum of n samples
    for(int i=0;i<n;i++){
        gettimeofday(&st,NULL);
        read_sensors();

        accel_sum[X] += accel_data[X];
        accel_sum[Y] += accel_data[Y];
        accel_sum[Z] += accel_data[Z] - (fbconfig.accel_multipliers[Z] * MPU6050_ACCEL_SENSITIVITY);

        gyro_sum[X] += gyro_data[X];
        gyro_sum[Y] += gyro_data[Y];
        gyro_sum[Z] += gyro_data[Z];

        // sleep remaining time to keep the sample rate
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec-st.tv_sec)*1000000)+(et.tv_usec-st.tv_usec);
        int remaining = interval - elapsed;
        if(remaining > 0) usleep(remaining);
    }

    // average all samples
    for(int i=0;i<3;i++){
        accel_offsets[i] = accel_sum[i] / (float)n;
        gyro_offsets[i] = gyro_sum[i] / (float)n;
    }
}

void IMU::complementary_filter(float dt){
    float ax = fbconfig.accel_multipliers[X] * accel_data[X] / (float)MPU6050_ACCEL_SENSITIVITY;
    float ay = fbconfig.accel_multipliers[Y] * accel_data[Y] / (float)MPU6050_ACCEL_SENSITIVITY;
    float az = fbconfig.accel_multipliers[Z] * accel_data[Z] / (float)MPU6050_ACCEL_SENSITIVITY;
    float gx = fbconfig.gyro_multipliers[X] * gyro_data[X] / (float)MPU6050_GYRO_SENSITIVITY;
    float gy = fbconfig.gyro_multipliers[Y] * gyro_data[Y] / (float)MPU6050_GYRO_SENSITIVITY;
    float gz = fbconfig.gyro_multipliers[Z] * gyro_data[Z] / (float)MPU6050_GYRO_SENSITIVITY;

    gyro_angles[ROLL] += gx * dt;
    gyro_angles[PITCH] += gy * dt;
    gyro_angles[YAW] += gz * dt;
    if(gyro_angles[YAW] > 180) gyro_angles[YAW] -= 360;
    if(gyro_angles[YAW] < -180) gyro_angles[YAW] += 360;

    accel_angles[ROLL] = atan2(ay, az) * RAD_TO_DEGREE;
    accel_angles[PITCH] = atan2(ax, az) * RAD_TO_DEGREE;
    accel_angles[YAW] = gyro_angles[YAW];

    comp_angles[ROLL] = (fbconfig.comp_filter_coefficient*(comp_angles[ROLL]+gx*dt)) + ((1-fbconfig.comp_filter_coefficient)*accel_angles[ROLL]);
    comp_angles[PITCH] = (fbconfig.comp_filter_coefficient*(comp_angles[PITCH]+gy*dt)) + ((1-fbconfig.comp_filter_coefficient)*accel_angles[PITCH]);
    comp_angles[YAW] = gyro_angles[YAW];
}

void IMU::update(float dt){
    read_sensors();
    complementary_filter(dt);
}

Angle* IMU::attitude(){
    return comp_angles;
}

Angle* IMU::gyro(){
    return gyro_angles;
}

Angle* IMU::accel(){
    return accel_angles;
}



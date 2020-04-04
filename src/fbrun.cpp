
#include<iostream>
#include<pigpio.h>
#include<sys/time.h>
#include<unistd.h>
#include<math.h>

#define DRONE_LOOP_RATE 400 // loop rate in Hz
#define IMU_CALIBRATION_TIME 5 // calibration time in seconds

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
//#define MPU6050_SAMPLE_RATE_DIV 0x13 // 8kHz / (1 + SAMPLE_RATE_DIV)

#define MPU6050_CONFIG 0x1A
//#define MPU6050_CONFIG_VALUE 0x00 // disable FSYNC and DLPF
#define MPU6050_CONFIG_VALUE 0x02 // disable FSYNC and set DLPF

#define MPU6050_I2C_MASTER_CLOCK 0x24
#define MPU6050_I2C_MASTER_CLOCK_VALUE 0x0D // 400 kHz

#define MPU6050_GYROCONFIG 0x1B
#define MPU6050_GYROCONFIG_VALUE 0x00 // gyroscope range 250

#define MPU6050_ACCELCONFIG 0x1C
#define MPU6050_ACCELCONFIG_VALUE 0x00 // accelerometer range 2g

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

#define MPU6050_ACCEL_SENSITIVITY 16384
#define MPU6050_GYRO_SENSITIVITY 131

#define RAD_TO_DEGREE 57.29578f
#define COMP_FILTER_COEFFICIENT 0.98f

// indexation
#define X 0
#define Y 1
#define Z 2
#define ROLL 0
#define PITCH 1
#define YAW 2
#define DIM 3

typedef short int SensorDataType;
typedef SensorDataType SensorData[DIM];
typedef float Angles[DIM];

SensorDataType accx_offset = 0;
SensorDataType accy_offset = 0;
SensorDataType accz_offset = 0;
SensorDataType gyrox_offset = 0;
SensorDataType gyroy_offset = 0;
SensorDataType gyroz_offset = 0;

void read_sensors(int handle, SensorData acc, SensorData gyro){
    char values[14];
    i2cReadI2CBlockData(handle,MPU6050_ACCEL_X1,values,14);
    acc[X] = (((SensorDataType)values[0] << 8) | (SensorDataType)values[1]) - accx_offset;
    acc[Y] = (((SensorDataType)values[2] << 8) | (SensorDataType)values[3]) - accy_offset;
    acc[Z] = ((SensorDataType)(values[4] << 8) | (SensorDataType)values[5]) - accz_offset;
    gyro[X] = ((SensorDataType)(values[8] << 8) | (SensorDataType)values[9]) - gyrox_offset;
    gyro[Y] = ((SensorDataType)(values[10] << 8) | (SensorDataType)values[11]) - gyroy_offset;
    gyro[Z] = ((SensorDataType)(values[12] << 8) | (SensorDataType)values[13]) - gyroz_offset;
}

float read_temp(int handle){
    SensorDataType temp1 = i2cReadByteData(handle,MPU6050_TEMP1);
    SensorDataType temp2 = i2cReadByteData(handle,MPU6050_TEMP2);
    SensorDataType temp = (temp1 << 8) | temp2;
    return temp/340.0 + 36.53;
}

int init_mpu6050(){
    // connect to device
    int handle = i2cOpen(I2C_BUS,MPU6050_I2C_ADDR,0);

    if(handle < 0){
        std::cout << "Error connecting to device" << std::endl;
        return -1;
    }

    // wake
    i2cWriteByteData(handle,MPU6050_PWR_1,MPU6050_PWR_WAKE);
    usleep(200 * 1000); // 200ms

    // config
    i2cWriteByteData(handle,MPU6050_USER_CTRL,MPU6050_USER_CTRL_VALUE); // user options
    i2cWriteByteData(handle,MPU6050_FIFO,MPU6050_FIFO_VALUE); // FIFO
    i2cWriteByteData(handle,MPU6050_PWR_2,MPU6050_PWR_2_VALUE); // disable standby
    i2cWriteByteData(handle,MPU6050_SAMPLE_RATE,MPU6050_SAMPLE_RATE_DIV); // sample rate
    i2cWriteByteData(handle,MPU6050_CONFIG,MPU6050_CONFIG_VALUE); // FSYNC and DLPF
    i2cWriteByteData(handle,MPU6050_I2C_MASTER_CLOCK,MPU6050_I2C_MASTER_CLOCK_VALUE); // I2C master clock
    i2cWriteByteData(handle,MPU6050_GYROCONFIG,MPU6050_GYROCONFIG_VALUE); // gyroscope range
    i2cWriteByteData(handle,MPU6050_ACCELCONFIG,MPU6050_ACCELCONFIG_VALUE); // accelerometer range

    sleep(1);

    return handle;
}

void compute_offsets(int handle){
    int accx_sum,accy_sum,accz_sum; accx_sum = accy_sum = accz_sum = 0;
    int gyrox_sum,gyroy_sum,gyroz_sum; gyrox_sum = gyroy_sum = gyroz_sum = 0;
    int interval = 1.0 / DRONE_LOOP_RATE * 1000 * 1000;
    int n = IMU_CALIBRATION_TIME * 1000 * 1000 / interval;
    SensorData accel,gyro;
    struct timeval st, et;

    for(int i=0;i<n;i++){
        gettimeofday(&st,NULL);
        read_sensors(handle,accel,gyro);

        accx_sum += accel[X];
        accy_sum += accel[Y];
        accz_sum += accel[Z] - MPU6050_ACCEL_SENSITIVITY;

        gyrox_sum += gyro[X];
        gyroy_sum += gyro[Y];
        gyroz_sum += gyro[Z];
        
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);
        int remaining = interval - elapsed;
        if(remaining > 0) usleep(remaining);
    }

    accx_offset = accx_sum / (float)n;
    accy_offset = accy_sum / (float)n;
    accz_offset = accz_sum / (float)n;
    
    gyrox_offset = gyrox_sum / (float)n;
    gyroy_offset = gyroy_sum / (float)n;
    gyroz_offset = gyroz_sum / (float)n;
}

void complementary_filter(SensorData accel, SensorData gyro, Angles comp_angles, Angles gyro_angles, Angles accel_angles, float dt){
    float ax = accel[X] / (float)MPU6050_ACCEL_SENSITIVITY;
    float ay = accel[Y] / (float)MPU6050_ACCEL_SENSITIVITY;
    float az = accel[Z] / (float)MPU6050_ACCEL_SENSITIVITY;
    float gx = gyro[X] / (float)MPU6050_GYRO_SENSITIVITY;
    float gy = gyro[Y] / (float)MPU6050_GYRO_SENSITIVITY;
    float gz = gyro[Z] / (float)MPU6050_GYRO_SENSITIVITY;

    gyro_angles[ROLL] += gx * dt;
    gyro_angles[PITCH] += gy * dt;
    gyro_angles[YAW] += gz * dt;
    if(gyro_angles[ROLL] > 180) gyro_angles[ROLL] -= 360;
    if(gyro_angles[ROLL] < -180) gyro_angles[ROLL] += 360;
    if(gyro_angles[PITCH] > 180) gyro_angles[PITCH] -= 360;
    if(gyro_angles[PITCH] < -180) gyro_angles[PITCH] += 360;
    if(gyro_angles[YAW] > 180) gyro_angles[YAW] -= 360;
    if(gyro_angles[YAW] < -180) gyro_angles[YAW] += 360;

    accel_angles[ROLL] = atan2(ay, az) * RAD_TO_DEGREE;
    accel_angles[PITCH] = atan2(ax, az) * RAD_TO_DEGREE;
    accel_angles[YAW] = gyro_angles[YAW];

    comp_angles[ROLL] = (COMP_FILTER_COEFFICIENT*(comp_angles[ROLL]+gx*dt)) + ((1-COMP_FILTER_COEFFICIENT)*accel_angles[ROLL]);
    comp_angles[PITCH] = (COMP_FILTER_COEFFICIENT*(comp_angles[PITCH]+gy*dt)) + ((1-COMP_FILTER_COEFFICIENT)*accel_angles[PITCH]);
    comp_angles[YAW] = gyro_angles[YAW];
}

int main() {
    // initialise the pigpio library
    gpioInitialise();

    int handle = init_mpu6050();
    compute_offsets(handle);

    int interval = 1.0 / DRONE_LOOP_RATE * 1000 * 1000;
    SensorData accel,gyro;
    Angles comp_angles = {0,0,0};
    Angles gyro_angles = {0,0,0};
    Angles accel_angles = {0,0,0};
    float dt;
    struct timeval st,et;

    int looptime = 0;
    int i = 0;
    while(1){
        gettimeofday(&st,NULL);
        read_sensors(handle,accel,gyro);
        dt = looptime / 1000000.0f;
        complementary_filter(accel,gyro,comp_angles,gyro_angles,accel_angles,dt);

        i++;
        if((i*interval) % (500*1000) == 0){
            std::cout << "COMP: " << comp_angles[ROLL] << " " << comp_angles[PITCH] << " " << comp_angles[YAW] << " | ";
            std::cout << "GYRO: " << gyro_angles[ROLL] << " " << gyro_angles[PITCH] << " " << gyro_angles[YAW] << " | ";
            std::cout << "ACCEL: " << accel_angles[ROLL] << " " << accel_angles[PITCH] << " " << accel_angles[YAW] << std::endl;
        }
       
        // sleep until next update loop 
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);
        int remaining = interval - elapsed;
        if(remaining > 0) usleep(remaining);

        // compute looptime
        gettimeofday(&et,NULL);
        looptime = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);
    }

    // close connection to device
    i2cClose(handle);

    // terminate the pigpio library
    gpioTerminate();

    return 0;
}


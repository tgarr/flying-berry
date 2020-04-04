
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
#define MPU6050_DEFAULT_ACCEL_Z 16384

#define MPU6050_TEMP1 0x41
#define MPU6050_TEMP2 0x42

#define MPU6050_GYRO_X1 0x43
#define MPU6050_GYRO_X2 0x44
#define MPU6050_GYRO_Y1 0x45
#define MPU6050_GYRO_Y2 0x46
#define MPU6050_GYRO_Z1 0x47
#define MPU6050_GYRO_Z2 0x48

#define X 0
#define Y 1
#define Z 2
#define DIM 3

#define ROLL 0
#define PITCH 1
#define YAW 2

typedef short int IMUDataType;
typedef IMUDataType XYZ[DIM];

IMUDataType accx_offset = 0;
IMUDataType accy_offset = 0;
IMUDataType accz_offset = 0;
IMUDataType gyrox_offset = 0;
IMUDataType gyroy_offset = 0;
IMUDataType gyroz_offset = 0;

void read_sensors(int handle, XYZ acc, XYZ gyro){
    char values[14];
    i2cReadI2CBlockData(handle,MPU6050_ACCEL_X1,values,14);
    acc[X] = (((IMUDataType)values[0] << 8) | (IMUDataType)values[1]) - accx_offset;
    acc[Y] = (((IMUDataType)values[2] << 8) | (IMUDataType)values[3]) - accy_offset;
    acc[Z] = ((IMUDataType)(values[4] << 8) | (IMUDataType)values[5]) - accz_offset;
    gyro[X] = ((IMUDataType)(values[8] << 8) | (IMUDataType)values[9]) - gyrox_offset;
    gyro[Y] = ((IMUDataType)(values[10] << 8) | (IMUDataType)values[11]) - gyroy_offset;
    gyro[Z] = ((IMUDataType)(values[12] << 8) | (IMUDataType)values[13]) - gyroz_offset;
}

void read_gyroscope(int handle, XYZ values){
    values[X] = (i2cReadByteData(handle,MPU6050_GYRO_X1) << 8 | i2cReadByteData(handle,MPU6050_GYRO_X2)) - gyrox_offset;
    values[Y] = (i2cReadByteData(handle,MPU6050_GYRO_Y1) << 8 | i2cReadByteData(handle,MPU6050_GYRO_Y2)) - gyroy_offset;
    values[Z] = (i2cReadByteData(handle,MPU6050_GYRO_Z1) << 8 | i2cReadByteData(handle,MPU6050_GYRO_Z2)) - gyroz_offset;
}

void read_accelerometer(int handle, XYZ values){
    values[X] = (i2cReadByteData(handle,MPU6050_ACCEL_X1) << 8 | i2cReadByteData(handle,MPU6050_ACCEL_X2)) - accx_offset;
    values[Y] = (i2cReadByteData(handle,MPU6050_ACCEL_Y1) << 8 | i2cReadByteData(handle,MPU6050_ACCEL_Y2)) - accy_offset;
    values[Z] = (i2cReadByteData(handle,MPU6050_ACCEL_Z1) << 8 | i2cReadByteData(handle,MPU6050_ACCEL_Z2)) - accz_offset;
}

float read_temp(int handle){
    IMUDataType temp1 = i2cReadByteData(handle,MPU6050_TEMP1);
    IMUDataType temp2 = i2cReadByteData(handle,MPU6050_TEMP2);
    IMUDataType temp = (temp1 << 8) | temp2;
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
    XYZ accel,gyro;
    struct timeval st, et;

    for(int i=0;i<n;i++){
        gettimeofday(&st,NULL);
        read_sensors(handle,accel,gyro);

        accx_sum += accel[X];
        accy_sum += accel[Y];
        accz_sum += accel[Z] - MPU6050_DEFAULT_ACCEL_Z;

        gyrox_sum += gyro[X];
        gyroy_sum += gyro[Y];
        gyroz_sum += gyro[Z];
        
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);
        int remaining = interval - elapsed;
        if(remaining > 0) usleep(remaining);
        //std::cout << elapsed << " " << remaining << std::endl;
    }

    accx_offset = accx_sum / (float)n;
    accy_offset = accy_sum / (float)n;
    accz_offset = accz_sum / (float)n;
    
    gyrox_offset = gyrox_sum / (float)n;
    gyroy_offset = gyroy_sum / (float)n;
    gyroz_offset = gyroz_sum / (float)n;

    std::cout << "offsets: " << accx_offset << " " << accy_offset << " " << accz_offset << " ";
    std::cout << gyrox_offset << " " << gyroy_offset << " " << gyroz_offset << std::endl;
}


// Madgwick AHRS algorithm
#define sampleFreq DRONE_LOOP_RATE 
volatile float beta = 0.1f; // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void roll_pitch_yaw(float values[3]){
    values[YAW] = atan2(2*q1*q2-2*q0*q3,2*q0*q0+2*q1*q1-1);
    values[PITCH] = -1*asin(2*q1*q3+2*q0*q2);
    values[ROLL] = atan2(2*q2*q3-2*q0*q1,2*q0*q0+2*q3*q3-1);

    values[ROLL] *= (180/3.141592);
    values[PITCH] *= (180/3.141592);
    values[YAW] *= (180/3.141592);
}

int main() {
    // initialise the pigpio library
    gpioInitialise();

    int handle = init_mpu6050();
    compute_offsets(handle);

    int interval = 1.0 / DRONE_LOOP_RATE * 1000 * 1000;
    XYZ accel,gyro;
    float angles[3];
    struct timeval st, et;

    int i = 0;
    while(1){
        gettimeofday(&st,NULL);
        read_sensors(handle,accel,gyro);
        MadgwickAHRSupdateIMU(gyro[X],gyro[Y],gyro[Z],accel[X],accel[Y],accel[Z]);

        i++;
        if((i*interval) % (500*1000) == 0){
            //roll_pitch_yaw(angles);
            //std::cout << angles[ROLL] << " " << angles[PITCH] << " " << angles[YAW] << std::endl;
            std::cout << q0 << " " << q1 << " " << q2 << " " << q3 << std::endl;
        }
        
        gettimeofday(&et,NULL);
        int elapsed = ((et.tv_sec - st.tv_sec) * 1000000) + (et.tv_usec - st.tv_usec);
        int remaining = interval - elapsed;
        if(remaining > 0) usleep(remaining);
    }

    // close connection to device
    i2cClose(handle);

    // terminate the pigpio library
    gpioTerminate();

    return 0;
}


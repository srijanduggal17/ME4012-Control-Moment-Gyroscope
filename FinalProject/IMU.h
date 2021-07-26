#ifndef _IMU
#define _IMU

#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
#define I2Cclock 400000
#define I2Cport Wire

#define IMU_AX_BIAS 0.0
#define IMU_AY_BIAS -0.02
#define IMU_AZ_BIAS -0.03

#define IMU_GX_BIAS 0.098
#define IMU_GY_BIAS -0.962
#define IMU_GZ_BIAS 0.275

#define ACCEL_CONST 0.02
#define GYRO_CONST 0.98

void setupIMU();
void readIMU(float *accX, float *accY, float *accZ, float *gyrX, float *gyrY, float *gyrZ);

#endif
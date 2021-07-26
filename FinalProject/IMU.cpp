#include "MPU9250.h"
#include "IMU.h"

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setupIMU() {
    myIMU.initMPU9250();
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
}

void readIMU(float *accX, float *accY, float *accZ, float *gyrX, float *gyrY, float *gyrZ) {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = ((float)myIMU.accelCount[0] * myIMU.aRes) + IMU_AX_BIAS; // - myIMU.accelBias[0];
    myIMU.ay = ((float)myIMU.accelCount[1] * myIMU.aRes) + IMU_AY_BIAS; // - myIMU.accelBias[1];
    myIMU.az = ((float)myIMU.accelCount[2] * myIMU.aRes) + IMU_AZ_BIAS; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = ((float)myIMU.gyroCount[0] * myIMU.gRes) + IMU_GX_BIAS;
    myIMU.gy = ((float)myIMU.gyroCount[1] * myIMU.gRes) + IMU_GY_BIAS;
    myIMU.gz = ((float)myIMU.gyroCount[2] * myIMU.gRes) + IMU_GZ_BIAS;
  }

  // Must be called before updating quaternions!
  myIMU.updateTime();
  
  *accX = myIMU.ax;
  *accY = myIMU.ay;
  *accZ = myIMU.az;
  *gyrX = myIMU.gx;
  *gyrY = myIMU.gy;
  *gyrZ = myIMU.gz;
  myIMU.count = millis();
  return;
}
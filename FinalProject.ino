#include "quaternionFilters.h"
#include "MPU9250.h"
#include "IMU.h"
#include "Encoder.h"
#include "Stepper.h"
#include "Flywheel.h"
#include <math.h>
#include <SPI.h>

//CONTROLLER PARAMS
#define Kp 2.8
#define Kd .25
#define Ki .015
#define TARGET_ANGLE 92
#define MAX_MOTOR_SPEED 5
float error_integral = 0;

//SYSTEM PARAMS
#define IC 7.24E-3
#define MC 1.55
#define G 9.81
#define L 0.141
#define IW 5.24e-4
#define OMEGA_W 209

//STATE ESTIMATES
float theta_estimate = 0.0;
float ball_angle = 0;
float cube_angle = 0;
float cube_anglevel = 0;

//TIMING PARAMS
float loopFreq = 8000; //Hz
float samplingFreq = 100; //Hz
int samplingRatio = loopFreq / samplingFreq;
int loop_us = (int) (1E6 / loopFreq);
float dt = 1.0 / samplingFreq;
unsigned long loopIter = 0;

void setup() {
  Serial.begin(115200); // For serial monitor
  SPI.begin();          // For encoder
  Wire.begin();         // For I2C
  setupEncoder();
  setupIMU();
  setupStepper();
  moveSphereToZero();
  setupFlywheel();
}

void loop() {
  loopIter++;
  unsigned long t_start = micros();

  if(loopIter % samplingRatio == 0){
      getStateData(&ball_angle, &cube_angle, &cube_anglevel);
      float e_theta = cube_angle - TARGET_ANGLE;
   
      if(loopIter > (5 * loopFreq)) {
        error_integral = error_integral + e_theta * dt;
        float moment = Kp * (e_theta * M_PI / 180.0) + Kd * (cube_anglevel * M_PI / 180.0) + Ki * (error_integral * M_PI / 180.0);
        float motor_speed = moment / (IW * OMEGA_W * cos(ball_angle * M_PI / 180.0));
        if(motor_speed > MAX_MOTOR_SPEED) {
          motor_speed = MAX_MOTOR_SPEED;
        }else if(motor_speed < -MAX_MOTOR_SPEED) {
          motor_speed = -MAX_MOTOR_SPEED;
        }
        setStepperVel(motor_speed);
      }
      if(loopIter > (5 * loopFreq) && (abs(e_theta) > 15 || abs(ball_angle) > 60)) {
          setStepperVel(0);
          brakeFlywheel();
          while(true){};
      }
  }
  
  pulseStepper();
  unsigned long t_end = micros();
  long delay_ = loop_us - (t_end - t_start);
  if(delay_ < 0) {
    delay_ = 0;
  }
  delayMicroseconds(delay_);
}

// Frame angle estimation
float getStateData(float *ball_angle, float *cube_angle, float *cube_anglevel) {
  // Initialize variables
  float imuAccX;
  float imuAccY;
  float imuAccZ;
  float imuGyrX;
  float imuGyrY;
  float imuGyrZ;

  // Read raw IMU and encoder data
  readIMU(&imuAccX, &imuAccY, &imuAccZ, &imuGyrX, &imuGyrY, &imuGyrZ);
  int cur_enc_pos = readEncoder();

  // Get angle of sphere relative to frame
  float cur_ball_angle = getBallAngle(cur_enc_pos);
  float ball_angle_rad = cur_ball_angle*M_PI/180.0;

  // Adjust axes to match dynamics equations
  float gyrI = imuGyrX;
  float gyrJ = imuGyrY;
  float gyrK = imuGyrZ;

  //Rotate IMU axes to compensate for misalignment of IMU
  float tilt1 = -3.2 * M_PI / 180.0;
  gyrI = cos(tilt1) * gyrI - sin(tilt1) * gyrK;
  gyrK = sin(tilt1) * gyrI + cos(tilt1) * gyrK;
  imuAccX = cos(tilt1) * imuAccX - sin(tilt1) * imuAccZ;
  imuAccZ = sin(tilt1) * imuAccX + cos(tilt1) * imuAccZ;

  float accI = -imuAccZ;
  float accJ = imuAccY;
  float accK = -imuAccX;
  
  // Estimate 1
  float theta1;
  if (abs(ball_angle_rad) > (M_PI/2)) {
    theta1 = -atan2(accI, -accK*cos(ball_angle_rad));
  } else {
    theta1 = M_PI - atan2(accI, -accK*cos(ball_angle_rad));
  }

  // Estimate 2
  float theta2;
  if (ball_angle_rad > 0) {
    theta2 = M_PI + atan2(accJ, -accK*sin(ball_angle_rad));
  } else {
    theta2 = atan2(accJ, -accK*sin(ball_angle_rad));
  }

  // Combine Estimates based on encoder position
  float theta_acc;
  if (abs(cur_ball_angle) < 45 || abs(cur_ball_angle) > 135) {
    theta_acc = theta1 * 180.0/M_PI;
  } else {
    theta_acc = theta2 * 180.0/M_PI;
  }

  // Get angle estimate from Gyro
  float omega_gyr;
  omega_gyr = cos(ball_angle_rad) * gyrJ - sin(ball_angle_rad) * gyrK;

  // Overall Estimate from Complementary Filter
  theta_estimate = theta_acc * ACCEL_CONST + (theta_estimate + omega_gyr * dt) * GYRO_CONST;

  Serial.println(theta_estimate);
  *cube_angle = theta_estimate;
  *ball_angle = cur_ball_angle;
  *cube_anglevel = omega_gyr;
}

void moveSphereToZero() {
  float cur_ball_angle;
  int cur_enc_pos = readEncoder();

  // Get angle of sphere relative to frame
  cur_ball_angle = getBallAngle(cur_enc_pos);

  while (abs(cur_ball_angle) > 0.25) {
    cur_enc_pos = readEncoder();
    cur_ball_angle = getBallAngle(cur_enc_pos);
    if (cur_ball_angle > 0) {
      setStepperVel(-.5);
      pulseStepper();
    } else {
      setStepperVel(.5);
      pulseStepper();
    }
  }
  setStepperVel(0);
}

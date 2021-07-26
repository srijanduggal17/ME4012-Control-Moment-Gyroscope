#ifndef _Stepper
#define _Stepper

#define STEP_PIN 4
#define DIR_PIN 6
#define EN_PIN 5

#define RAD_PER_REV (2 * M_PI)
#define STEP_ANGLE 1.80
#define MICROSTEPSIZE 8
#define GEAR_RATIO (48.0 / 17.0)

void setStepperVel(float omega);
void pulseStepper();
void setupStepper();

#endif
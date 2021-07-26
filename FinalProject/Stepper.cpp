#include <Arduino.h>
#include <SPI.h>
#include "Stepper.h"

int stepsPerRev = (int) (360.0 / STEP_ANGLE);
int microstepsPerRev = stepsPerRev * MICROSTEPSIZE;

long last_pulse_micros = 0;
long step_interval = 0;

void setStepperVel(float omega) {
	if(omega == 0) {
		step_interval = 1E9;
		return;
	}
	omega = omega * GEAR_RATIO;
	if (omega < 0) {
	  digitalWrite(DIR_PIN, LOW); // Set direction
	} else {
	  digitalWrite(DIR_PIN, HIGH); // Set direction
	}
	float revPerSec = abs(omega) / RAD_PER_REV;
	float pulsesPerSec = revPerSec * microstepsPerRev;
	float microsecBwPulses = (1E6 / pulsesPerSec);
	step_interval = microsecBwPulses;
}

void pulseStepper() {
	long t = micros();
	if(t - last_pulse_micros > step_interval) {
		last_pulse_micros = t;
		digitalWrite(STEP_PIN, HIGH);
  		digitalWrite(STEP_PIN, LOW);
	}
}

void setupStepper() {
  	pinMode(STEP_PIN, OUTPUT); 		// Step pin. Cycle high low to move 1 step
  	pinMode(DIR_PIN, OUTPUT); 		// Direction pin. 
  	pinMode(EN_PIN, OUTPUT); 		// Driver enable pin. Pull High to de energize motor
  	digitalWrite(EN_PIN, LOW); 		// Enable driver
  	digitalWrite(DIR_PIN, LOW); 	// Set direction
}
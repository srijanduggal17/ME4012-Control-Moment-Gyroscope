#include <Arduino.h>
#include "Flywheel.h"

void setupFlywheel() {
	pinMode(FLYWHEEL_PWM, OUTPUT); 					// Set pin as output
	float dutyCycle = 0.75;
	analogWrite(FLYWHEEL_PWM, dutyCycle*255); 		// 0.75 is no motion for flywheel
	delay(5000); 									// Delay 8 seconds
	dutyCycle = 0.78; 								// 0.78 is slowest speed in forward direction
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(5000); 

	// dutyCycle = 0.786; 							//For standing
	// analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	// delay(5000); 

	dutyCycle = 0.80; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(3000); 

	dutyCycle = 0.83; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(3000); 

	dutyCycle = 0.86; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(3000); 

	dutyCycle = 0.89; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(3000); 

	dutyCycle = 0.92; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(3000);

	dutyCycle = 0.95; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(3000);

	dutyCycle = 0.98; 								//For jumping
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(5000);
}

void brakeFlywheel() {
	float dutyCycle = 0.50; 						// Set duty cycle back to 0.75 to let it freely spin to rest
	analogWrite(FLYWHEEL_PWM, dutyCycle*255);
	delay(8000); 
    dutyCycle = 0.75; 								// 0.78 is slowest speed in forward direction
    analogWrite(FLYWHEEL_PWM, dutyCycle*255);
}
#ifndef ROME_h
#define ROME_h

#include "Arduino.h"

#define wheel_diameter .1
#define pi 3.1415926535
#define pulsesPerRev 6
#define gearRatio 50
#define distancePerRevolution (pi*wheel_diameter)

class ROME{
	public:
		uint8_t pwm_pin;
		uint8_t hall_pin;
		uint8_t relay_pin;
		ROME(int pin, int Hpin, int Rpin);
		void motorOn(int volt);
		void motorForward();
		void motorReverse();
		void disableMotor();
		void updatePulses();
		void clearPulses();
		double getPulses();

	private:
		double _pulseCounter;
};

#endif  

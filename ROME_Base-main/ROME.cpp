#include "Arduino.h"
#include "ROME.h"

ROME::ROME(int pin, int Hpin, int Rpin) {
  pwm_pin = (uint8_t) pin;  
  pinMode(pwm_pin, OUTPUT);
  hall_pin = (uint8_t) Hpin; 
  pinMode(hall_pin, INPUT_PULLUP);
  relay_pin = (uint8_t) Rpin;
  pinMode(relay_pin, OUTPUT);
}

void ROME::motorOn(int pwm_val) {
	//commented out section was to map pwm input to a voltge resemblant value
	//if (volt > 24)
		//volt = 24;
	//int avg_volt = (volt * 255) / 24;
	//now the input is in between 0-255
	analogWrite(pwm_pin, pwm_val);
}

void ROME::motorForward(){
	digitalWrite(relay_pin,HIGH); // CCW direction
}

void ROME::motorReverse(){
	digitalWrite(relay_pin,LOW); // CW direction
}

void ROME::disableMotor() {
	analogWrite(pwm_pin, 0);
}

double ROME::getPulses() {
	return _pulseCounter;
}

void ROME::updatePulses() {
	_pulseCounter++;
}

void ROME::clearPulses() {
	_pulseCounter = 0;
}

#include "IPS.hpp"
#include <stdint.h>
#include <Arduino.h>

#define IPS_ENABLE 0

bool waiting_for_stable_input = true;
bool IPS_checkVoltage() {
#if IPS_ENABLE == 0
	return true;
#else
	int16_t senseVoltage = analogRead(PIN_SENSE);
	
	//85 is the scaling factor between the analog reading and the real voltage
	
	if (waiting_for_stable_input) {
		waiting_for_stable_input = senseVoltage < (11 * 85); //11 volts
	} else {
		waiting_for_stable_input = senseVoltage < (10 * 85); //10 volts
	}
	
	return !waiting_for_stable_input;
#endif
}
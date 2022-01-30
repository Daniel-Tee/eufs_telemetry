#include "Logging.hpp"
#include "IPS.hpp" //Interruptable power supply
#include <stdint.h>
#include <Arduino.h>

bool L_status; //Status of 'L' LED

bool Logging_makeRecord(char* dataString) {
	if (!IPS_checkVoltage()) { //Don't write to card if the input is unstable
		Serial.println("Waiting for stable input");
		while (!IPS_checkVoltage());
		return false; 
	}

	// open the file. note that only one file can be open at a time,
	// so you have to close this one before opening another.
	uint32_t start = millis();
	File dataFile = SD.open("datalog.txt", FILE_WRITE);
	
	if (!IPS_checkVoltage()) {
		Serial.println("Waiting for stable input 2"); //Don't write to card if the input is unstable
		while (!IPS_checkVoltage());
		dataFile.close();
		return false;
	}
	
	// if the file is available, write to it:
	if (dataFile) {
		dataFile.println(dataString);
		dataFile.close();
		uint32_t end = millis();

		// print to the serial port too:
		Serial.println(dataString);

		//Print time taken to log
		Serial.print("took ");
		Serial.print(end-start);
		Serial.println("ms");

		//Blink L LED
		L_status = !L_status;
		digitalWrite(13, L_status);
	} else {
		// if the file isn't open, pop up an error:
		Serial.println("error opening datalog.txt");
		return false;
	}
	return true;
}
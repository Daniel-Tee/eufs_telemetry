#include "Dashboard.hpp"

//Serial buffer length
#define BUFFER_LENGTH 64

#include <string.h> //sprintf
#include <Arduino.h> //Serial

#define RECORD_LENGTH 5
static uint32_t Record[RECORD_LENGTH+1];
uint32_t* Dashboard_get_record() {
	return Record;
}

char sendStr[BUFFER_LENGTH];

void Dashboard_update_sendStr(uint16_t wheelSpeed, uint16_t tps, uint16_t rpm, uint16_t warning, uint16_t gear) {
	sprintf(sendStr, "%03d,%03d,%05d,%d,%d\n", wheelSpeed, tps, rpm, warning, gear);
}

uint32_t last_run_time;

void Dashboard_init() {
	//First element of Record should be RECORD_LENGTH
	Record[0] = RECORD_LENGTH;

	Serial3.begin(115200);  // Baud rate for HC-05 
	//Default values
	Record[1] = 50;
	Record[2] = 100;
	Record[3] = 6500;
	Record[4] = 0;
	Record[5] = 3;
	
	last_run_time = micros();
}

void Dashboard_update() {
	//If at least 16ms has passed since the last update
	if (micros() - last_run_time > 16000) {
		last_run_time = micros();
		
		//Update string with data from the record
		Dashboard_update_sendStr(Record[1], Record[2], Record[3], Record[4], Record[5]);
		
		Serial3.write(sendStr);
	}
}

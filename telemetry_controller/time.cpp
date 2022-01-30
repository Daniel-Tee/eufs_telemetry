#include "time.hpp"
#include "Arduino.h"

#define RECORD_LENGTH 1
static uint32_t Record[RECORD_LENGTH+1];
uint32_t* Time_get_record() {
	return Record;
}

bool Time_init() {
	//First element of Record should be RECORD_LENGTH
	Record[0] = RECORD_LENGTH;
	return true;
}

bool Time_task() {
	Record[1] = micros();
	return true;
}

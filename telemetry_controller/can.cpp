#include "can.hpp"
#include "due_can.h"

#define RECORD_LENGTH 1
static uint32_t Record[RECORD_LENGTH+1];

uint32_t* CAN_get_record() {
	return Record;
}

static uint32_t can_fail_count = 0;

bool CAN_init() {
	//First element of Record should be RECORD_LENGTH
	Record[0] = RECORD_LENGTH;
	
	uint32_t baud = Can0.begin(250000, 0xFF); //250kbps, no enable pin
	if (baud == 0) {
		can_fail_count++;
		return false;
	}
	
	Can0.watchFor();
	return true;
}

bool CAN_task() {
	
	if (can_fail_count > 0) {
		//do something smart
		return false;
	}
	
	
	uint32_t waiting_frames = Can0.available();
	if (waiting_frames > 0) {
		//Process the incoming frames
		
		//Read a frame
		CAN_FRAME incoming;
		Can0.read(incoming);
		
		//This is really a float but it's the same size as uint32_t
		Record[1] = ((uint32_t*)incoming.data.bytes)[0];
		return true;
	}
	return false;
}

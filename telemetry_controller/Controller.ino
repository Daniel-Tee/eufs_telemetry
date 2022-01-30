/*
Environment requirements:
SD library by Arduino


Physical setup:
Power supply sense - pin A0 (set by PIN_SENSE in IPS.hpp)

SD card
	Attached to SPI header:
		MOSI - pin 4
		MISO - pin 1
		CLK  - pin 3
	CS is on normal pins:
		CS   - pin  4 (set by PIN_CHIP_SELECT)

Bluetooth
	Connected to UART3:
		RXD - pin 14 (TX3)
		TXD - pin 15 (RX3)
 */

#include <SPI.h>
#include <SD.h>
#include <stdint.h>

#include "Logging.hpp" //SD card logging
#include "Dashboard.hpp"
#include "time.hpp"
#include "gyro.hpp"
#include "can.hpp"

#define PIN_CHIP_SELECT 4

void setup() {
	pinMode(13, OUTPUT);
	
	pinMode(A8, OUTPUT);
	pinMode(A10, OUTPUT);
	pinMode(A9, INPUT);
	digitalWrite(A8, 1);
	digitalWrite(A10, 0);
	
	// Open serial communications and wait for port to open
	Serial.begin(115200);
	while (!Serial); //Wait for serial port to connect


	Serial.print("Initializing SD card...");

	//See if the card is present and can be initialized:
	if (!SD.begin(PIN_CHIP_SELECT)) {
		Serial.println("Card failed, or not present");
		// don't do anything more:
		while (1);
	}
	Serial.println("card initialized.");
	
	Dashboard_init();
	Time_init();
	//Gyro_init();
	if (!CAN_init()) {
		Serial.println("CAN failed to init");
		delay(1000);
	}
}

char dataString[300];
void loop() {
	int16_t value = analogRead(A9);
	if (CAN_task()) {
		Serial.println("received");
	}
	
	float wssFrequency = ((float*)CAN_get_record())[1];
	float wssGroundSpeed = wssFrequency / 3;
	
	Dashboard_get_record()[1] = (uint32_t) wssGroundSpeed;
	Dashboard_get_record()[3] = value * 13;
	Dashboard_update();
	Time_task();
	//Gyro_update();
	
	uint32_t time = Time_get_record()[1];
	uint32_t* gyro_data = Gyro_get_record();
	
	sprintf(dataString, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		time,
		gyro_data[1], gyro_data[2], gyro_data[3], gyro_data[4], gyro_data[5], gyro_data[6], gyro_data[7], gyro_data[8], gyro_data[9]
	);
	
	Logging_makeRecord(dataString);
}









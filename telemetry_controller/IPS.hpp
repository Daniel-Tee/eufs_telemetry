#pragma once

#define PIN_SENSE 0 //A0 = Sense line for input voltage

//Check if we have good voltage on the input
//Returns: true if good voltage
bool IPS_checkVoltage();
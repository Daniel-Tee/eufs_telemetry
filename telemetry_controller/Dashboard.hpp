#pragma once

#include <stdint.h>

#define PIN_BLUETOOTH_RX 10
#define PIN_BLUETOOTH_TX 11

uint32_t* Dashboard_get_record();

void Dashboard_init();

void Dashboard_update();
#pragma once

#include <stdint.h>

bool CAN_init();

bool CAN_task();

uint32_t* CAN_get_record();
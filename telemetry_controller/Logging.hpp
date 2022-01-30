#pragma once

#include <SPI.h>
#include <SD.h>

//Save this string to the SD card
//Returns: true on success
bool Logging_makeRecord(char*);
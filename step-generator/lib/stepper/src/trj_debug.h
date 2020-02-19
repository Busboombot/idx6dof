#pragma once

#include <Arduino.h>

extern Stream &debug_serial;


void ser_printf(const char* fmt, ...);
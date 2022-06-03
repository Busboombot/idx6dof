#pragma once

#include <Arduino.h>

extern Stream &debug_serial;

// This should be a macro, or something, 
// but I got lazy trying to figure out macro varargs
#define SER_PRINT_ENABLED false

void ser_printf(const char* fmt, ...);
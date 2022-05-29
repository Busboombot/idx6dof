#include <stdarg.h>
#include <Arduino.h>

extern Stream &debug_serial;
char printf_buffer[1024];

// Set or clear externally to turn printing off and on
bool ser_printf_flag = true;

// Printf to the debug serial port
void ser_printf(const char* fmt, ...){

    if (!ser_printf_flag){
        return;
    }

    va_list args;
    va_start(args,fmt);
    vsprintf(printf_buffer, fmt,args);
    va_end(args);
    debug_serial.println(printf_buffer);
    debug_serial.flush();
}


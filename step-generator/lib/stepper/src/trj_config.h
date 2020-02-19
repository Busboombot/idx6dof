#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <limits.h>


// Configuration record for one axis
// 8 Bytes
struct AxisConfig {

    uint8_t axis;           // Axis number
    uint8_t step_pin;       // Step output, or quadture b
    uint8_t direction_pin;  // Direction output, or quadrature b
    uint8_t enable_pin;
    uint32_t v_max;
    uint32_t a_max;
};

// Main Configuration class, 68 bytes
struct Config {

    uint8_t n_axes = 0;         // Number of axes
    uint8_t interrupt_delay=5;    // How often interrupt is called, in microseconds
    bool debug_print = true;
    bool debug_tick = true;
};

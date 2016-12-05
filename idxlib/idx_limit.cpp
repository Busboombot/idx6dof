/*
 *  Class for an IDX Robot Limit Switches
 */


#include "Arduino.h"

#include "idx_limit.h"
// #include "digitalWriteFast.h"

// Need for MCP Operations
#include <Wire.h>
#include <Adafruit_MCP23008.h>

Adafruit_MCP23008 mcp;


IDXLimit::IDXLimit() {
    // Serial.println("IDXLimit");
};


/*
 * ===========================================================================================================
 *                  Pin Limit
 */

IDXPinLimit::IDXPinLimit(int pin) : IDXLimit()  {
    // Serial.println("IDXPinLimit");
    this->pin = pin;
    
};

void IDXPinLimit::begin() {
    
    pinMode(this->pin, INPUT);
    
};

int IDXPinLimit::limitValue() {
    
    this->p_value = digitalRead(this->pin);
    
    return this->p_value;
    
};

bool IDXPinLimit::isInLimit() {
    
    limitValue();
    
    if (this->p_value == 0) {
        return true;
    }
    else {
        return false;
    }
};


/*
 * ===========================================================================================================
 *                  MCP Limit
 */

IDXMcpLimit::IDXMcpLimit(int pin, int mcp_address) : IDXLimit() {
    // Serial.println("IDXMcpLimit");
    this->pin = pin;
    
    this->mcp_address = mcp_address;
    
};

void IDXMcpLimit::begin() {
    
    // Adafruit_MCP23008 mcp;
    // Begin MCP on IC2 with address 0
    mcp.begin(this->mcp_address);
    
    mcp.pinMode(this->pin, INPUT);
    
    mcp.pullUp(this->pin, HIGH);
    
};

int IDXMcpLimit::limitValue() {
    // Serial.println(this->p_value);
    this->p_value = mcp.digitalRead(this->pin);
    
    return this->p_value;
    
};

bool IDXMcpLimit::isInLimit() {
    
    limitValue();
    
    if (this->p_value == 0) {
        return true;
    }
    else {
        return false;
    }
};
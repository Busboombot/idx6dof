/*
* Fake Arduino.h for compiing tests
*/

#pragma once

#define OUTPUT 0
#define INPUT 1
#define LOW 0
#define HIGH 1
#define digitalPinToBitMask(x) (1<<x)
#define pinMode(p,s) (NULL)



class Stream {


};

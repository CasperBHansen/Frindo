/*
 * Sensor.cpp
 */

#include "Sensor.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

Sensor::Sensor(int id, float (* func)(int))
{
    this->id = id;
    this->func = func;
}

float Sensor::read(void)
{
    return func(analogRead(id));
}

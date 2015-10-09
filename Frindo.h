#ifndef Frindo_h
#define Frindo_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Vector.h"
#include "Sensor.h"
#include "SerialInterpreter.h"

class Frindo
{
    public:
    Frindo();
    ~Frindo();

    // high-level API
    void stop(void);

    // getters
    float readFront(void);
    float readLeft(void);
    float readRight(void);

    // planned
    void turn(const float theta);

    // setters
    void setVelocity(const float s);
    void setAngle(const float theta);
    void setPolar(const float s, const float theta);
    void setDirection(const Vector& v);
    void setDirection(const float x, const float y);

    void setWheel(const Vector& v);

    private:
    float velocity;
    Vector direction;
    Vector wheel;

    Sensor * front;
    Sensor * left;
    Sensor * right;
};

#endif

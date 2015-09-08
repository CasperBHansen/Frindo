#ifndef Frindo_h
#define Frindo_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Vector.h"
#include "Sensor.h"

class Frindo
{
    public:
    Frindo();
    ~Frindo();

    // low-level API
    void set_motor_l(int vel);
    void set_motor_r(int vel);

    // high-level API
    void go(void);
    void stop(void);

    // getters
    float readFront(void);
    float readLeft(void);
    float readRight(void);

    // setters
    void setSpeed(const float s);
    void setAngle(const float theta);
    void setPolar(const Vector& v);
    void setDirection(const Vector& v);

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

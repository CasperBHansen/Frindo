/*
 * Sensor.h
 */

#ifndef Sensor_h
#define Sensor_h

class Sensor
{
    public:
    Sensor(int id, float (* func)(int));

    float read(void);

    private:
    int id;

    int raw;

    float (* func)(int);
};

#endif

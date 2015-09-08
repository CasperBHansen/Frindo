/*
 * Vector.cpp
 */

#include "Vector.h"

#include <math.h>

Vector::Vector(float x, float y)
{
    this->x = x;
    this->y = y;
}

// getters
const float Vector::getX(void) const { return x; }
const float Vector::getY(void) const { return y; }

const float Vector::magnitude(void) const
{
    return sqrt((x * x) + (y * y));
}

const float Vector::magnitude(const Vector& v)
{
    return v.magnitude();
}

Vector Vector::normalize(const Vector& v)
{
    const float a = v.getX();
    const float b = v.getY();
    const float c = v.magnitude();
    return Vector(a / c, b / c); // operation not defined yet!
}

// operators
Vector& Vector::operator=(const Vector& rhs)
{
    x = rhs.getX();
    y = rhs.getY();
    return (* this);
}

Vector& Vector::operator+(const Vector& rhs)
{
    x = x + rhs.getX();
    y = y + rhs.getY();
    return (* this);
}

Vector& Vector::operator-(const Vector& rhs)
{
    x = x - rhs.getX();
    y = y - rhs.getY();
    return (* this);
}

Vector& Vector::operator*(const float s)
{
    x *= s;
    y *= s;
    return (* this);
}

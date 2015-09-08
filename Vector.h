/*
 * Vector.h
 */

#ifndef Vector_h
#define Vector_h

class Vector
{
    public:
    Vector(float x = 0.0, float y = 0.0);

    // getters
    const float getX(void) const;
    const float getY(void) const;
    const float magnitude(void) const;

    // static functions
    static const float magnitude(const Vector& vec);
    static Vector normalize(const Vector& vec);

    // operators
    Vector& operator=(const Vector& rhs);
    Vector& operator+(const Vector& rhs);
    Vector& operator-(const Vector& rhs);

    Vector& operator*(const float s);

    private:
    float x, y;
};

#endif

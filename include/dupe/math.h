#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <cmath>


typedef float real;

class Vector3
{
public:
    real x, y, z;

private:
    real w;

public:
    Vector3() : x(0), y(0), z(0) {}

    Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

    void operator*=(const real value)
    {
        x *= value;
        y *= value;
        z *= value;
    }

    Vector3 operator*(const real value)
    {
        return Vector3(x * value, y * value, z * value);
    }

    void operator+=(const real value)
    {
        x += value;
        y += value;
        z += value;
    }

    Vector3 operator+(const real value)
    {
        return Vector3(x + value, y + value, z + value);
    }

    void operator+=(const Vector3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    Vector3 operator+(const Vector3& v)
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    void operator-=(const Vector3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    Vector3 operator-(const Vector3& v)
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    void addScaledVector(const Vector3& v, real scale)
    {
        x += v.x * scale;
        y += v.y * scale;
        z += v.z * scale;
    }

    Vector3 componentProduct(const Vector3& v)
    {
        return Vector3(x * v.x, y * v.y, z * v.z);
    }

    void componentProductUpdate(const Vector3& v)
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
    }

    real operator*(const Vector3& v)
    {
        return x * v.x + y * v.y + z * v.z;
    }

    Vector3 vectorProduct(const Vector3& v)
    {
        return Vector3(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.z - y*v.x);
    }

    void operator %=(const Vector3& v)
    {
        *this = vectorProduct(v);
    }

    Vector3 operator%(const Vector3& v)
    {
        return Vector3(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.z - y*v.x);
    }

    void invert()
    {
        x = -x;
        y = -y;
        z = -z;
    }

    real magnitude()
    {
        return std::sqrt(x*x + y*y + z*z);
    }

    real squared_mangitude()
    {
        return x*x + y*y + z*z;
    }

    void normalize()
    {
        real m = magnitude();
        if (m > 0)
            (*this) *= ((real)1) / m;
    }

    void clear()
    {
        x = 0;
        y = 0;
        z = 0;
    }
};


inline void createOrthonormalBasis(Vector3* a, Vector3* b, Vector3* c)
{
    a->normalize();
    (*c) = (*a) % (*b);
    if (c->squared_mangitude() == 0.0) return;
    c->normalize();
    (*b) = (*c) % (*a);
}


#endif
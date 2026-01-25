#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <cmath>
#include <limits>

typedef float real;
constexpr real REAL_MAX = std::numeric_limits<real>::max();

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


class Quaternion
{
public:
    Quaternion(real r, real i, real j, real k) : r(r), i(i), j(j), k(k) {}

    union {
        struct 
        {
            real r;
            real i;
            real j;
            real k;
        };

        real data[4];
    };

    void normalize()
    {
        real d = r*r + i*i + j*j + k*k;

        if (d == 0)
        {
            r = 1;
            return;
        }

        d = ((real)1.0) / std::sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }

    void operator*=(const Quaternion& other)
    {
        Quaternion q = *this;
        r = q.r*other.r - q.i*other.i - q.j*other.j - q.k*other.k;
        i = q.r*other.i + q.i*other.r + q.j*other.k - q.k*other.j;
        j = q.r*other.j + q.j*other.r + q.k*other.i - q.i*other.k;
        k = q.r*other.k + q.k*other.r + q.i*other.j - q.j*other.i;
    }

    void rotateByVector(const Vector3& v)
    {
        Quaternion q(0, v.x, v.y, v.z);
        (*this) *= q;
    }

    void addScaledVector(const Vector3& v, real scale)
    {
        Quaternion q(0, v.x * scale, v.y * scale, v.z * scale);
        q *= *this;
        r += q.r * ((real)0.5);
        i += q.i * ((real)0.5);
        j += q.j * ((real)0.5);
        k += q.k * ((real)0.5);
    }
};


class Matrix3
{
public:
    real data[9];

    Vector3 operator*(const Vector3& v) const
    {
        return Vector3(
            v.x * data[0] + v.y * data[1] + v.z * data[2],
            v.x * data[3] + v.y * data[4] + v.z * data[5],
            v.x * data[6] + v.y * data[7] + v.z * data[8]);
    }

    Vector3 transform(const Vector3& v) const
    {
        return (*this) * v;
    }

    Matrix3 operator*(const Matrix3& o) const
    {
        return Matrix3{
            data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
            data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
            data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],
            data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
            data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
            data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],
            data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
            data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
            data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]};
    }

    void operator*=(const Matrix3& o)
    {
        real t1;
        real t2;
        real t3;
        t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
        t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
        t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
        data[0] = t1;
        data[1] = t2;
        data[2] = t3;
        t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
        t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
        t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
        data[3] = t1;
        data[4] = t2;
        data[5] = t3;
        t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
        t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
        t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
        data[6] = t1;
        data[7] = t2;
        data[8] = t3;
    }

protected:
    void setInverse(const Matrix3& m)
    {
        real t1 = m.data[0]*m.data[4];
        real t2 = m.data[0]*m.data[5];
        real t3 = m.data[1]*m.data[3];
        real t4 = m.data[2]*m.data[3];
        real t5 = m.data[1]*m.data[6];
        real t6 = m.data[2]*m.data[6];
        // Calculate the determinant.
        real det = (t1*m.data[8] - t2*m.data[7] - t3*m.data[8]+
        t4*m.data[7] + t5*m.data[5] - t6*m.data[4]);
        // Make sure the determinant is non-zero.
        if (det == (real)0.0f) return;
        real invd = (real)1.0f/det;
        data[0] = (m.data[4]*m.data[8]-m.data[5]*m.data[7])*invd;
        data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*invd;
        data[2] = (m.data[1]*m.data[5]-m.data[2]*m.data[4])*invd;
        data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*invd;
        data[4] = (m.data[0]*m.data[8]-t6)*invd;
        data[5] = -(t2-t4)*invd;
        data[6] = (m.data[3]*m.data[7]-m.data[4]*m.data[6])*invd;
        data[7] = -(m.data[0]*m.data[7]-t5)*invd;
        data[8] = (t1-t3)*invd;
    }

public:
    Matrix3 inverse() const
    {
        Matrix3 result;
        result.setInverse(*this);
        return result;
    }

    void invert()
    {
        setInverse(*this);
    }

protected:
    void setTranspose(const Matrix3& m)
    {
        data[0] = m.data[0];
        data[1] = m.data[3];
        data[2] = m.data[6];
        data[3] = m.data[1];
        data[4] = m.data[4];
        data[5] = m.data[7];
        data[6] = m.data[2];
        data[7] = m.data[5];
        data[8] = m.data[8];
    }

public:
    Matrix3 transpose() const
    {
        Matrix3 result;
        result.setTranspose(*this);
        return result;
    }

    void setOrientation(const Quaternion &q)
    {
        data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
        data[1] = 2*q.i*q.j + 2*q.k*q.r;
        data[2] = 2*q.i*q.k - 2*q.j*q.r;
        data[3] = 2*q.i*q.j - 2*q.k*q.r;
        data[4] = 1 - (2*q.i*q.i + 2*q.k*q.k);
        data[5] = 2*q.j*q.k + 2*q.i*q.r;
        data[6] = 2*q.i*q.k + 2*q.j*q.r;
        data[7] = 2*q.j*q.k - 2*q.i*q.r;
        data[8] = 1 - (2*q.i*q.i + 2*q.j*q.j);
    }
};


class Matrix4
{
public:
    real data[12];

    Vector3 operator*(const Vector3& v) const
    {
        return Vector3(
            v.x * data[0] + v.y * data[1] + v.z * data[2] + data[3],
            v.x * data[4] + v.y * data[5] + v.z * data[6] + data[7],
            v.x * data[8] + v.y * data[9] + v.z * data[10] + data[11]);
    }

    Vector3 transform(const Vector3& v) const
    {
        return (*this) * v;
    }

    Matrix4 operator*(const Matrix4 &o) const
    {
        Matrix4 result;
        result.data[0] = o.data[0]*data[0] + o.data[4]*data[1] + o.data[8]*data[2];
        result.data[4] = o.data[0]*data[4] + o.data[4]*data[5] + o.data[8]*data[6];
        result.data[8] = o.data[0]*data[8] + o.data[4]*data[9] + o.data[8]*data[10];
        result.data[1] = o.data[1]*data[0] + o.data[5]*data[1] + o.data[9]*data[2];
        result.data[5] = o.data[1]*data[4] + o.data[5]*data[5] + o.data[9]*data[6];
        result.data[9] = o.data[1]*data[8] + o.data[5]*data[9] + o.data[9]*data[10];
        result.data[2] = o.data[2]*data[0] + o.data[6]*data[1] + o.data[10]*data[2];
        result.data[6] = o.data[2]*data[4] + o.data[6]*data[5] + o.data[10]*data[6];
        result.data[10] = o.data[2]*data[8] + o.data[6]*data[9] + o.data[10]*data[10];
        result.data[3] = o.data[3]*data[0] + o.data[7]*data[1] + o.data[11]*data[2] + data[3];
        result.data[7] = o.data[3]*data[4] + o.data[7]*data[5] + o.data[11]*data[6] + data[7];
        result.data[11] = o.data[3]*data[8] + o.data[7]*data[9] + o.data[11]*data[10] + data[11];
        return result;
    }

    real getDeterminant() const
    {
        return data[8]*data[5]*data[2]+
            data[4]*data[9]*data[2]+
            data[8]*data[1]*data[6]-
            data[0]*data[9]*data[6]-
            data[4]*data[1]*data[10]+
            data[0]*data[5]*data[10];
    }

protected:
    void setInverse(const Matrix4& m)
    {
        // Make sure the determinant is non-zero.
        real det = getDeterminant();
        if (det == 0) return;
        det = ((real)1.0f)/det;
        data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
        data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
        data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9]*m.data[15])*det;
        data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
        data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
        data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9]*m.data[15])*det;
        data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6]*m.data[15])*det;
        data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6]*m.data[15])*det;
        data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5]*m.data[15])*det;
        data[3] = (m.data[9]*m.data[6]*m.data[3]
            -m.data[5]*m.data[10]*m.data[3]
            -m.data[9]*m.data[2]*m.data[7]
            +m.data[1]*m.data[10]*m.data[7]
            +m.data[5]*m.data[2]*m.data[11]
            -m.data[1]*m.data[6]*m.data[11])*det;
        data[7] = (-m.data[8]*m.data[6]*m.data[3]
            +m.data[4]*m.data[10]*m.data[3]
            +m.data[8]*m.data[2]*m.data[7]
            -m.data[0]*m.data[10]*m.data[7]
            -m.data[4]*m.data[2]*m.data[11]
            +m.data[0]*m.data[6]*m.data[11])*det;
        data[11] =(m.data[8]*m.data[5]*m.data[3]
            -m.data[4]*m.data[9]*m.data[3]
            -m.data[8]*m.data[1]*m.data[7]
            +m.data[0]*m.data[9]*m.data[7]
            +m.data[4]*m.data[1]*m.data[11]
            -m.data[0]*m.data[5]*m.data[11])*det;
    }

public:
    Matrix4 inverse() const
    {
        Matrix4 result;
        result.setInverse(*this);
        return result;
    }

    void invert()
    {
        setInverse(*this);
    }

    void setOrientationAndPos(const Quaternion& q, const Vector3& pos)
    {
        data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
        data[1] = 2*q.i*q.j + 2*q.k*q.r;
        data[2] = 2*q.i*q.k - 2*q.j*q.r;
        data[3] = pos.x;
        data[4] = 2*q.i*q.j - 2*q.k*q.r;
        data[5] = 1 - (2*q.i*q.i + 2*q.k*q.k);
        data[6] = 2*q.j*q.k + 2*q.i*q.r;
        data[7] = pos.y;
        data[8] = 2*q.i*q.k + 2*q.j*q.r;
        data[9] = 2*q.j*q.k - 2*q.i*q.r;
        data[10] = 1 - (2*q.i*q.i + 2*q.j*q.j);
        data[11] = pos.z;
    }

    Vector3 transformInverse(const Vector3& v) const
    {
        Vector3 tmp = v;
        tmp.x -= data[3];
        tmp.y -= data[7];
        tmp.z -= data[11];
        return Vector3(
            tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
            tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
            tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]);
    }

    Vector3 transformDirection(const Vector3& v) const
    {
        return Vector3(
            v.x * data[0] + v.y * data[1] + v.z * data[2],
            v.x * data[4] + v.y * data[5] + v.z * data[6],
            v.x * data[8] + v.y * data[9] + v.z * data[10]);
    }

    Vector3 transformInverseDirection(const Vector3 &v) const
    {
        return Vector3(
            v.x * data[0] + v.y * data[4] + v.z * data[8],
            v.x * data[1] + v.y * data[5] + v.z * data[9],
            v.x * data[2] + v.y * data[6] + v.z * data[10]);
    }
};


inline Vector3 localToWorld(const Vector3& local, const Matrix4& transform)
{
    return transform.transform(local);
}

inline Vector3 worldToLocal(const Vector3& world, const Matrix4& transform)
{
    return transform.transformInverse(world);
}

inline Vector3 localToWorldDirection(const Vector3& local, const Matrix4& transform)
{
    return transform.transformDirection(local);
}

inline Vector3 worldToLocalDirection(const Vector3& world, const Matrix4& transform)
{
    return transform.transformInverseDirection(world);
}


#endif
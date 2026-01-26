#ifndef BODY_H
#define BODY_H

#include <assert.h>

#include "math.h"


class RigidBody
{
protected:
    real inverseMass;
    real linearDamping;
    real angularDamping;
    Vector3 position;
    Quaternion orientation;
    Vector3 velocity;
    Vector3 acceleration;
    Vector3 rotation;

    Vector3 lastFrameAcceleration;

    Matrix4 transform;
    Matrix3 inverseInertia, inverseInertiaWorld;

    bool isAwake;
    Vector3 forceAcc, torqueAcc;

public:
    void calculateDerivedData();
    void setInertiaTensor(const Matrix3& inertiaTensor);

    Vector3 toWorldSpace(const Vector3& bodyPoint);
    Vector3 toBodySpace(const Vector3& worldPoint);

    void addForce(const Vector3& force);
    void addForceAtPoint(const Vector3& force, const Vector3& point);
    void addForceAtBodyPoint(const Vector3& force, const Vector3& point);

    void clearAccumulators();

    void integrate(real duration);

    bool hasFiniteMass();

    real getMass() 
    {
        assert(hasFiniteMass());
        return 1 / inverseMass;
    }
};

#endif
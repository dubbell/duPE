#ifndef BODY_H
#define BODY_H

#include "math.h"


class RigidBody
{
protected:
    real inverseMass;
    real linearDamping;
    Vector3 position;
    Quaternion orientation;
    Vector3 velocity;
    Vector3 rotation;
    Matrix4 transform;

    void calculateDerivedData();
};

#endif
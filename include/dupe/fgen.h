#ifndef FGEN_H
#define FGEN_H


#include "math.h"
#include "body.h"

class ForceGenerator
{
public:
    virtual void updateForce(RigidBody* body, real duration) = 0;
};

class Gravity : public ForceGenerator
{
    Vector3 gravity;

public:
    Gravity(const Vector3& gravity);

    void updateForce(RigidBody* body, real duration);
};

class Spring : public ForceGenerator
{
    Vector3 connectionPoint;
    Vector3 otherConnectionPoint;
    RigidBody* other;
    real springConstant;
    real restLength;

public:
    Spring(const Vector3& localConnectionPt, RigidBody* other, const Vector3& otherConnectionPt, real springConstant, real restLength)
        : connectionPoint(localConnectionPt), otherConnectionPoint(otherConnectionPt), other(other), springConstant(springConstant), restLength(restLength) {}

    void updateForce(RigidBody* body, real duration);
};

#endif
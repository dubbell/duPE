#include "fgen.h"


void Gravity::updateForce(RigidBody* body, real duration)
{
    if (!body->hasFiniteMass()) return;
    body->addForce(gravity * body->getMass());
}


void Spring::updateForce(RigidBody* body, real duration)
{
    Vector3 lws = body->toWorldSpace(connectionPoint);
    Vector3 ows = body->toWorldSpace(otherConnectionPoint);

    Vector3 force = lws - ows;

    real magnitude = force.magnitude();
    magnitude = std::abs(magnitude - restLength);
    magnitude *= springConstant;

    force.normalize();
    force *= -magnitude;
    body->addForceAtPoint(force, lws);
}



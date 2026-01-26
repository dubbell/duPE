#include "fgen.h"



void ForceRegistry::add(RigidBody* body, ForceGenerator* fg)
{
    registrations.push_back({body, fg});
}

void ForceRegistry::add(RigidBody* body, ForceGenerator* fg)
{
    auto it = registrations.begin();
    while (it != registrations.end())
    {
        if (it->body == body && it->fg == fg)
        {
            registrations.erase(it);
            break;
        }
    }
}

void ForceRegistry::clear()
{
    registrations.clear();
}

void ForceRegistry::updateForces(real duration)
{
    auto it = registrations.begin();
    for (; it != registrations.end(); it++)
        it->fg->updateForce(it->body, duration);
}


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



#ifndef PARTICLE_H
#define PARTICLE_H

#include "math.h"

#include <vector>
#include <assert.h>


class Particle
{
protected:
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;

    Vector3 forceAccumulator;

    real inverseMass, damping;

public:
    void setInverseMass(real inverseMass)
    {
        this->inverseMass = inverseMass;
    }

    void setMass(real mass)
    {
        assert (mass > 0);
        inverseMass = 1 / mass;
    }

    void integrate(real duration);

    void clearAccumulator()
    {
        forceAccumulator.clear();
    }

    void addForce(const Vector3& force)
    {
        forceAccumulator += force;
    }

    bool hasFiniteMass()
    {
        return inverseMass != 0;
    }

    real getMass()
    {
        return 1 / inverseMass;
    }

    Vector3 getVelocity()
    {
        return velocity;
    }

    Vector3 getPosition()
    {
        return position;
    }
};


#endif
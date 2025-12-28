#include "dupe/particle.h"

#include <assert.h>


void Particle::integrate(real duration)
{
    if (inverseMass <= 0.0f) return;

    assert(duration > 0.0);

    position.addScaledVector(velocity, duration);

    Vector3 totalAcceleration = acceleration;
    totalAcceleration.addScaledVector(forceAccumulator, inverseMass);

    velocity.addScaledVector(totalAcceleration, duration);

    velocity *= std::pow(damping, duration);

    clearAccumulator();
}
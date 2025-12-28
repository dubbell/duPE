#include "dupe/pfgen.h"


void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* fg)
{
    registrations.push_back({particle, fg});
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* fg)
{
    auto it = registrations.begin();
    while (it != registrations.end())
    {
        if (it->particle == particle && it->fg == fg) 
        {
            registrations.erase(it);
            break;
        }
    }
}

void ParticleForceRegistry::clear()
{
    registrations.clear();
} 

void ParticleForceRegistry::updateForces(real duration)
{
    auto it = registrations.begin();
    for (; it != registrations.end(); it++)
        it->fg->updateForce(it->particle, duration);
}


void ParticleGravity::updateForce(Particle* particle, real duration)
{
    if (!particle->hasFiniteMass()) return;
    particle->addForce(gravity * particle->getMass());
}

void ParticleDrag::updateForce(Particle* particle, real duration)
{
    Vector3 force = particle->getVelocity();

    real drag = force.magnitude();
    drag = k1 * drag + k2 * drag * drag;

    force.normalize();
    force *= drag;
    particle->addForce(force);
}

void ParticleSpring::updateForce(Particle* particle, real duration)
{
    Vector3 force = particle->getPosition();
    force -= other->getPosition();

    real magnitude = std::abs(force.magnitude() - restLength) * springConstant;

    force.normalize();
    force *= -magnitude;

    particle->addForce(force);
}

void ParticleAnchoredSpring::updateForce(Particle* particle, real duration)
{
    Vector3 force = particle->getPosition();
    force -= *anchor;
    
    real magnitude = (restLength - force.magnitude()) * springConstant;

    force.normalize();
    force *= magnitude;

    particle->addForce(force);
}

void ParticleBungee::updateForce(Particle* particle, real duration)
{
    Vector3 force = particle->getPosition();
    force -= other->getPosition();

    real magnitude = force.magnitude();
    if (magnitude <= restLength) return;

    magnitude = springConstant * (restLength - magnitude);

    force.normalize();
    force *= -magnitude;
    particle->addForce(force);
}

void ParticleBuoyancy::updateForce(Particle* particle, real duration)
{
    real depth = particle->getPosition().y;
    if (depth >= waterHeight + maxDepth) return;

    Vector3 force(0, 0, 0);

    if (depth <= waterHeight - maxDepth)
    {
        force.y = liquidDensity * volume;
        particle->addForce(force);
        return;
    }

    force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
    particle->addForce(force);
}
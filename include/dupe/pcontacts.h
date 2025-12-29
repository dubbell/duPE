#ifndef PCONTACTS_H
#define PCONTACTS_H

#include "math.h"
#include "particle.h"


class ParticleContact
{
public:
    Particle* particle[2];
    real restitution;
    Vector3 contactNormal;
    real penetration;
    Vector3 particleMovement[2];

    real calculateSeparatingVelocity() const;
    void resolve(real duration);

private:
    void resolveVelocity(real duration);
    void resolveInterpenetration(real duration);
};

class ParticleContactResolver
{
protected:
    unsigned iterations;
    unsigned iterationsUsed;

public:
    ParticleContactResolver(unsigned iterations) : iterations(iterations) {}

    void setIterations(unsigned iterations);

    void resolveContacts(ParticleContact* contactArray, unsigned numContacts, real duration);
};

class ParticleContactGenerator 
{
public:
    virtual unsigned addContact(ParticleContact* contact, unsigned limit) const = 0;
};

#endif
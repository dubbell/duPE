#ifndef PLINKS_H
#define PLINKS_H

#include "pcontacts.h"


class ParticleLink : public ParticleContactGenerator
{
public:
    Particle* particle[2];

protected:
    real currentLength() const;

public:
    unsigned addContact(ParticleContact* contact, unsigned limit) const override = 0;
};

class ParticleCable : public ParticleLink
{
public:
    real maxLength;
    real restitution;

    unsigned addContact(ParticleContact* contact, unsigned limit) const override;
};

class ParticleRod : public ParticleLink
{
public:
    real length;

    unsigned addContact(ParticleContact* contact, unsigned limit) const override;
};

#endif
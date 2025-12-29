#ifndef PWORLD_H
#define PWORLD_H

#include <vector>

#include "particle.h"
#include "pcontacts.h"
#include "pfgen.h"


class ParticleWorld
{
public:
    typedef std::vector<Particle*> Particles;
    typedef std::vector<ParticleContactGenerator*> ContactGenerators;

protected:
    Particles particles;

    ParticleForceRegistry registry;
    ParticleContactResolver resolver;

    bool calculateIterations;

    ContactGenerators contactGenerators;

    unsigned maxContacts;
    ParticleContact* contacts;

    unsigned generateContacts();
    void integrate(real duration);

public:
    ParticleWorld(unsigned maxContacts, unsigned iterations = 0) : maxContacts(maxContacts), resolver(iterations), calculateIterations(iterations > 0) {}

    void startFrame();
    void runPhysics(real duration);
};

#endif
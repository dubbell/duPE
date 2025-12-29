#include "pworld.h"


unsigned ParticleWorld::generateContacts()
{
    unsigned limit = maxContacts;
    ParticleContact* nextContact = contacts;

    for (auto g = contactGenerators.begin(); g != contactGenerators.end(); g++)
    {
        unsigned used = (*g)->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        if (limit <= 0) break;
    }

    return maxContacts - limit;
}

void ParticleWorld::integrate(real duration)
{
    for (auto p = particles.begin(); p != particles.end(); p++)
        (*p)->integrate(duration);
}

void ParticleWorld::runPhysics(real duration)
{
    registry.updateForces(duration);

    integrate(duration);

    unsigned usedContacts = generateContacts();

    if (usedContacts)
    {
        if (calculateIterations) resolver.setIterations(usedContacts * 2);
        resolver.resolveContacts(contacts, usedContacts, duration);
    }
}
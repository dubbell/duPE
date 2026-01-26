#include "world.h"


void World::startFrame()
{
    for (auto rb : bodies)
    {
        rb->clearAccumulators();
        rb->calculateDerivedData();
    }
}

void World::runPhysics(real duration)
{
    registry.updateForces(duration);
    for (auto rb : bodies)
        rb->integrate(duration);
}
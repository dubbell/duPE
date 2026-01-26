#ifndef WORLD_H
#define WORLD_H

#include <vector>

#include "body.h"
#include "fgen.h"


class World
{
public:
    typedef std::vector<RigidBody*> RigidBodies;

protected:
    RigidBodies bodies;

    ForceRegistry registry;

    void startFrame();
    void runPhysics(real duration);
};

#endif
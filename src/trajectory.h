#pragma once
#include "joints.h"
#include "math.h"
#include "timeutils.h"


struct Knot
{
    int64_t ts;
    Joints q;
};

using Trajectory = std::vector<Knot>;

std::ostream& operator << (std::ostream& s, Knot const& knot);
std::ostream& operator << (std::ostream& s, Trajectory const& traj);
std::string to_string(Knot const& knot);
std::string to_string(Trajectory const& traj);

Trajectory velocity(Trajectory const& traj);
Trajectory acceleration(Trajectory const& traj);

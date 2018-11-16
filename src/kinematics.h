#pragma once
#include "properties.h"
#include "joints.h"
#include "transforms.h"


struct Joint
{
    std::string name;
    Isometry3d T;
    double lower, upper;
    double max_velocity;
    double max_accel;
};

using Chain = std::vector<Joint>;

namespace kinematic_model
{
    void init();
    Isometry3d solvefk(Joints const& q);
    Joints joints_min();
    Joints joints_max();
    Joints velocities_max();
    Joints accel_max();
    int njoints();
}

#include <iostream>
#include "kinematics.h"

static Chain _kinematic_chain;
static Joints _joints_min;
static Joints _joints_max;
static Joints _velocities_max;
static Joints _accel_max;
static bool _initialized = false;


Isometry3d parse_transform(Property const& cfg_transform)
{
    auto const& cfg_dh = properties::get(cfg_transform, "dh");
    double alpha = properties::get_double(cfg_dh, "alpha");
    double r = properties::get_double(cfg_dh, "r");
    double theta = properties::get_double(cfg_dh, "theta");
    double d = properties::get_double(cfg_dh, "d");
    return dh_transform(r, alpha, d, theta);
}

Joint parse_joint(Property const& cfg_joint)
{
    Joint joint;
    joint.name = properties::get_string(cfg_joint, "name");
    auto const& cfg_range = properties::get(cfg_joint, "range");
    joint.lower = properties::get_double(cfg_range, 0);
    joint.upper = properties::get_double(cfg_range, 1, joint.lower, 1e+6);
    joint.max_velocity = properties::get_double(cfg_joint, "max_velocity", 0, 1e+6);
    joint.max_accel = properties::get_double(cfg_joint, "max_accel", 0, 1e+6);
    auto const& cfg_transform = properties::get(cfg_joint, "transform");
    joint.T = parse_transform(cfg_transform);
    return joint;
}

void kinematic_model::init()
{
    auto const& cfg_chain = properties::get("kinematic_chain");
    const int n = cfg_chain.size();
    _initialized = false;
    _kinematic_chain.reserve(n);

    for (int i = 0; i < n; ++ i)
    {
        auto const& cfg_joint = cfg_chain[i];
        _kinematic_chain.emplace_back(parse_joint(cfg_joint));
    }

    _joints_min.resize(n);
    _joints_max.resize(n);
    _velocities_max.resize(n);
    _accel_max.resize(n);

    for (int i = 0; i < n; ++ i)
    {
        _joints_min[i] = _kinematic_chain[i].lower;
        _joints_max[i] = _kinematic_chain[i].upper;
        _velocities_max[i] = _kinematic_chain[i].max_velocity;
        _accel_max[i] = _kinematic_chain[i].max_accel;
    }
    _initialized = true;
}

Isometry3d kinematic_model::solvefk(Joints const& q)
{
    assert(_initialized);
    assert(_kinematic_chain.size() == q.size());
    const int n = _kinematic_chain.size();
    auto T = Isometry3d::Identity();

    for (int i = 0; i < n; ++ i)
    {
        T = T * rotz(q[i]) * _kinematic_chain[i].T;
    }

    return T;
}

Joints kinematic_model::joints_min()
{
    assert(_initialized);
    return _joints_min;
}

Joints kinematic_model::joints_max()
{
    assert(_initialized);
    return _joints_max;
}

Joints kinematic_model::velocities_max()
{
    assert(_initialized);
    return _velocities_max;
}

Joints kinematic_model::accel_max()
{
    assert(_initialized);
    return _accel_max;
}

int kinematic_model::njoints()
{
    assert(_initialized);
    return _velocities_max.size();
}

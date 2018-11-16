#pragma once
#include "joints.h"
#include "trajectory.h"
#include "timeutils.h"


/*
 * The resulting trajectory is defines as
 *  q = q1 + (q2 - q1) * s(t)
 * where the function s(t) is a typical B-spline
 * The function find the coefficients of the spline such, that
 * the resulting trajectory fits the limits in velocity and acceleration  
 */
Trajectory
plan_trajectory(
    Joints const& q1,
    Joints const& q2,
    Joints const& v_max,
    Joints const& a_max,
    int64_t timestep_usec
    )
{
    const double eps = 1e-8;
    const int n = q1.size();

    assert(q2.size() == n && a_max.size() == n && v_max.size() == n);
    assert(a_max.min() > 0 && v_max.min() > 0);
    assert(timestep_usec > 0);

    if (L_inf_dist(q1, q2) < eps)
        return Trajectory{{0, q1}, {timestep_usec, q2}};

    auto Dq = q2 - q1;

    // 1. find acceleration for s
    int i = Dq.abs().divw(a_max).argmax();
    double a = a_max[i] / std::fabs(Dq[i]);

    // 2. find velocity for s
    i = Dq.abs().divw(v_max).argmax();
    double v = v_max[i] / std::fabs(Dq[i]);

    // 3. find s1
    double s1 = square(v) / (2 * a);
    Trajectory traj;

    if (s1 < 0.5)
    {
        double t1 = v / a;
        double s2 = 1 - s1;
        double t2 = t1 + (s2 - s1) / v;
        double t3 = t2 + t1;

        int n = int(std::ceil(t3 / usec_to_sec(timestep_usec))) + 1;    
        traj.resize(n);

        for (int i = 0; i < n; ++ i)
        {
            int64_t ts = i * timestep_usec;
            double t = usec_to_sec(ts);
            double s;

            if (t < t1)
                s = a/2 * square(t);
            else if (t < t2)
                s = a*t1*t - a/2*square(t1);
            else if (t < t3)
                s = a*t1*t2 - a/2*square(t-t3);
            else
                s = 1.;

            traj[i] = {ts, q1 + s * (q2 - q1)};
        }
    }
    else
    {
        s1 = 0.5;
        v = sqrt(a);
        double t1 = 1 / sqrtf(a);
        double t2 = 2 * t1;

        int n = int(std::ceil(t2 / usec_to_sec(timestep_usec))) + 1;    
        traj.resize(n);

        for (int i = 0; i < n; ++ i)
        {
            int64_t ts = i * timestep_usec;
            double t = usec_to_sec(ts);
            double s, ds;

            if (t < t1)
                s = a/2 * square(t);
            else if (t < t2)
                s = -a/2 * square(t) + 2*a*t1*t - a*square(t1);
            else
                s = 1.;

            traj[i] = {ts, q1 + s * (q2 - q1)};
        }
    }

    return traj;
}

inline Trajectory
plan_trajectory(
    Joints const& q1,
    Joints const& q2,
    double v_max,
    double a_max,
    int64_t timestep_usec
    )
{
    const int n = q1.size();
    Joints _a_max(n);
    Joints _v_max(n);

    _a_max.fill(a_max);
    _v_max.fill(v_max);

    return plan_trajectory(q1, q2, _v_max, _a_max, timestep_usec);
}

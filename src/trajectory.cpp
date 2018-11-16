#include "trajectory.h"


Trajectory velocity(Trajectory const& traj)
{
    const int n = traj.size();
    Trajectory velocities(n);

    velocities[0].ts = traj[0].ts;
    velocities[0].q = Joints::zeros(traj[0].q.size());

    for (int i = 1; i < n - 1; ++ i)
    {
        int njoints = traj[i].q.size();
        double t0 = usec_to_sec(traj[i-1].ts - traj[i].ts);
        double t1 = 0;
        double t2 = usec_to_sec(traj[i+1].ts - traj[i].ts);
        velocities[i].ts = traj[i].ts;
        velocities[i].q.resize(njoints);

        for (int j = 0; j < njoints; ++ j)
        {
            auto coefs = fitparabola(t0, traj[i-1].q[j], t1, traj[i].q[j], t2, traj[i+1].q[j]);
            velocities[i].q[j] = coefs[1];
        }
    }

    velocities[n-1].ts = traj[n-1].ts;
    velocities[n-1].q = Joints::zeros(traj[n-1].q.size());
    return velocities;
}

Trajectory acceleration(Trajectory const& traj)
{
    const int n = traj.size();
    Trajectory accelerations(n);

    accelerations[0].ts = traj[0].ts;
    accelerations[0].q = Joints::zeros(traj[0].q.size());

    for (int i = 1; i < n - 1; ++ i)
    {
        int njoints = traj[i].q.size();
        double t0 = usec_to_sec(traj[i-1].ts - traj[i].ts);
        double t1 = 0;
        double t2 = usec_to_sec(traj[i+1].ts - traj[i].ts);
        accelerations[i].ts = traj[i].ts;
        accelerations[i].q.resize(njoints);

        for (int j = 0; j < njoints; ++ j)
        {
            auto coefs = fitparabola(t0, traj[i-1].q[j], t1, traj[i].q[j], t2, traj[i+1].q[j]);
            accelerations[i].q[j] = 2*coefs[0];
        }
    }

    accelerations[n-1].ts = traj[n-1].ts;
    accelerations[n-1].q = Joints::zeros(traj[n-1].q.size());
    return accelerations;
}

std::ostream& operator << (std::ostream& s, Knot const& knot)
{
    s << "(" << knot.ts << ", " << knot.q << ")";
    return s;
}

std::ostream& operator << (std::ostream& s, Trajectory const& traj)
{
    const int n = traj.size();
    s << "[";

    for (int i = 0; i < n; ++ i)
    {
        if (i == n-1)
            s << traj[i] << "]";
        else
            s << traj[i] << ";\n";
    }

    return s;
}

std::string to_string(Knot const& knot)
{
    std::stringstream ss;
    ss << knot;
    return ss.str();
}

std::string to_string(Trajectory const& traj)
{
    std::stringstream ss;
    ss << traj;
    return ss.str();
}
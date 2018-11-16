#include <iostream>
#include "../src/planner.h"
#include "../src/throws.h"


void test1()
{
    Joints q1 = {1,2,3.};
    Joints q2 = {1,2,3. + 1e-10};
    const int64_t step = 1e+3;
    auto const& trajectory = plan_trajectory(q1, q2, 0.1, 0.8, 4e+3);

    if (trajectory.size() != 2)
        throw_runtime_error("trajectory must have only 2 points");

    if (trajectory[0].q != q1)
        throw_runtime_error("trajectory first point must be ", q1, " but actually is ", trajectory[0].q);

    if (trajectory[1].q != q2)
        throw_runtime_error("trajectory first point must be ", q1, " but actually is ", trajectory[0].q);
}


void test_trajecotry(Joints const& q1, Joints const& q2, double max_vel, double max_accel, int64_t step)
{
    auto const& trajectory = plan_trajectory(q1, q2, max_vel, max_accel, step);

    if (trajectory.size() < 2)
        throw_runtime_error("trajectory must have at least 2 points");

    if (trajectory.begin()->q != q1)
        throw_runtime_error("trajectory does not begin with the point q1");

    if (trajectory.rbegin()->q != q2)
        throw_runtime_error("trajectory does not come to point q2");
    
    auto const& velocities = velocity(trajectory);
    auto const& accelerations = acceleration(trajectory);

    auto max_component = [](double max_val, Knot const& knot) -> double {
        return std::max(max_val, knot.q.max());
    };
    auto min_component = [](double max_val, Knot const& knot) -> double {
        return std::min(max_val, knot.q.min());
    };
    double max_vel_ = aggregate(velocities, 0., max_component);
    double min_vel_ = aggregate(velocities, 0., min_component);
    double max_accel_ = aggregate(accelerations, 0., max_component);
    double min_accel_ = aggregate(accelerations, 0., min_component);

    if (fabs(max_accel_ - max_accel) > 1e-8)
        throw_runtime_error("trajectory must have the maximal available acceleration");

    if (fabs(min_accel_ + max_accel) > 1e-8)
        throw_runtime_error("trajectory must have the maximal available acceleration");

    if (fabs(max_vel_) - max_vel > 1e-8)
        throw_runtime_error("velocity limit vilated");

    if (fabs(min_vel_) - max_vel > 1e-8)
        throw_runtime_error("velocity limit vilated");
    
    for (int i = 1; i < trajectory.size(); ++ i)
    {
        int64_t d = trajectory[i].ts - trajectory[i-1].ts;
        if (d != step)
            throw_runtime_error("trajectory has an incorrect timestep");
    }
}


void test2()
{
    Joints q1 = {0.23453, 1.98743432, -2.2342, 0.22145};
    Joints q2 = {0.86543, -0.241235, 1.2452134, 0.736243};
    const int64_t step = 4324;

    test_trajecotry(q1, q2, 1.4, 1.6, step);
    test_trajecotry(q1, q2, 5.0, 1.2, step);
}


void test3()
{
    Joints q1 = {0.23453, 1.98743432, -2.2342, 0.22145};
    Joints q2 = {0.86543, -0.241235, 1.2452134, 0.736243};
    const int64_t step = 2342;
    double max_vel = 1.4;
    double max_accel = 1.6;

    auto const& trajectory = plan_trajectory(q1, q2, max_vel, max_accel, step);
    auto const& velocities = velocity(trajectory);
    auto max_component = [](double max_val, Knot const& knot) -> double {
        return std::max(max_val, knot.q.abs().max());
    };
    double max_vel_ = aggregate(velocities, 0., max_component);

    if (fabs(max_vel_ - max_vel) > 1e-8)
        throw_runtime_error("velocity is less than given");
}


int main()
{
    try
    {
        std::cout << "test1: ";
        test1();
        std::cout << "ok" << std::endl;
        std::cout << "test2: ";
        test2();
        std::cout << "ok" << std::endl;
        std::cout << "test3: ";
        test3();
        std::cout << "ok" << std::endl;
    }
    catch (std::exception const& e)
    {
        std::cerr << "failed: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "failed: undef" << std::endl;
    }

    return 0;
}

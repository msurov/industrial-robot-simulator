#include "robot.h"
#include "timeutils.h"
#include "planner.h"
#include "kinematics.h"


RobotSim::RobotSim(Joints const& q_initial, int64_t period)
{
    assert(period > 0);
    _q = q_initial;
    _ts = 0;
    _period = period;
    _state = RobotState::Stopped;
}

RobotSim::~RobotSim()
{
    stop();
}

void RobotSim::actual_position(Joints& q, int64_t& ts) const
{
    q = _q;
    ts = _ts;
}

bool RobotSim::is_busy() const
{
    return _state == RobotState::Busy;
}

bool RobotSim::is_ran() const
{
    return _state != RobotState::Stopped;
}

void RobotSim::loop()
{
    _ts = (epoch_usec() / _period) * _period;

    while (_state != RobotState::Stopped)
    {
        sleep_usec(_ts + _period - epoch_usec());
        _ts += _period;

        if (_state != RobotState::Busy)
            continue;
        
        if (!_traj)
            continue;

        int64_t d = _ts - _traj_start;
        if (d < 0)
            continue;

        int i = int(d / _period);

        if (i >= _traj->size())
        {
            _q = _traj->rbegin()->q;
            _traj = nullptr;
            _traj_start = 0;
            _state = RobotState::Ready;
            continue;
        }

        _q = _traj->at(i).q;
    }
}

bool RobotSim::run()
{
    if (_state != RobotState::Stopped)
        return false;
    _state = RobotState::Ready;
    _loop_thread = std::thread(&RobotSim::loop, this);
    return true;
}

void RobotSim::stop()
{
    _state = RobotState::Stopped;
    if (_loop_thread.joinable())
        _loop_thread.join();
}

bool RobotSim::movej(Joints const& target)
{
    return movej(target, kinematic_model::velocities_max(), kinematic_model::accel_max());
}

bool RobotSim::movej(Joints const& target, double v, double a)
{
    const int n = target.size();
    Joints vel(n), acc(n);
    vel.fill(v);
    acc.fill(a);
    return movej(target, vel, acc);
}

bool verify_target(Joints const& target, Joints const& vel, Joints const& acc)
{
    return 
        within(target, kinematic_model::joints_min(), kinematic_model::joints_max()) &&
        all_leq(acc, kinematic_model::accel_max()) &&
        all_leq(vel, kinematic_model::velocities_max());
}

bool RobotSim::movej(Joints const& target, Joints const& vel, Joints const& acc)
{
    if (!verify_target(target, vel, acc))
        return false;

    std::lock_guard<std::mutex> lock(_mtx_change_state);

    if (_state != RobotState::Ready)
        return false;

    auto const& trajectory = plan_trajectory(_q, target, vel, acc, _period);
    _traj.reset(new Trajectory(trajectory));
    _traj_start = _ts + sec_to_usec(1);
    _state = RobotState::Busy;
    return true;
}

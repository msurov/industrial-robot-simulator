#pragma once
#include <thread>
#include <mutex>
#include <memory>
#include "joints.h"
#include "trajectory.h"


enum class RobotState
{
    Stopped, Busy, Ready
};

class RobotSim
{
private:
    Joints _q;
    int64_t _ts;
    int64_t _period;

    RobotState _state;
    std::thread _loop_thread;
    std::mutex _mtx_change_state;

    int64_t _traj_start;
    std::unique_ptr<Trajectory> _traj;

    void loop();

public:
    RobotSim(Joints const& q_initial, int64_t period);
    RobotSim(RobotSim const&) = delete;
    RobotSim(RobotSim&&) = delete;
    ~RobotSim();

    bool run();
    void stop();

    bool movej(Joints const& target, Joints const& vel, Joints const& acc);
    bool movej(Joints const& target, double vel, double acc);
    bool movej(Joints const& target);

    void actual_position(Joints& q, int64_t& ts) const;
    bool is_busy() const;
    bool is_ran() const;
};

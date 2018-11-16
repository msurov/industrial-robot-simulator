#include <iostream>
#include "robot.h"
#include "properties.h"
#include "kinematics.h"


int main(int argc, char const* argv[])
{
    if (argc != 2)
    {
        std::cerr << "expect config as the first argument" << std::endl;
        return -1;
    }

    properties::init(argv[1]);
    kinematic_model::init();
    auto const& cfg_robot = properties::get("robot");
    auto const& cfg_q0 = properties::get(cfg_robot, "initial_configuration");
    auto const& q0 = Joints::from_vec(properties::to_vector<double>(cfg_q0));
    int64_t period = sec_to_usec(properties::get_double(cfg_robot, "period", 0., 1.));

    RobotSim robot(q0, period);
    robot.run();

    sleep_sec(1);
    robot.movej({1,2,3,4,5,6});

    for (int i = 0; i < 100; ++ i)
    {
        Joints q;
        int64_t ts;
        robot.actual_position(q, ts);
        std::cout << ts << " " << q << std::endl;
        sleep_sec(0.1);
    }

    return 0;
}

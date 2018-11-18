#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include "robot_sim.h"
#include "properties.h"
#include "kinematics.h"
#include "publisher.h"
#include "signals.h"


class Application
{
private:
    std::unique_ptr<RobotSim> _robot;
    std::unique_ptr<PosePublisher> _pub;
    std::unique_ptr<ros::NodeHandle> _node;
    ros::Subscriber _sub;
    int64_t _pub_period;
    bool _stop;


    void init_robot()
    {
        auto const& cfg_robot = properties::get("robot");
        auto const& cfg_q0 = properties::get(cfg_robot, "initial_configuration");
        auto const& q0 = Joints::from_vec(properties::to_vector<double>(cfg_q0));
        int64_t period = sec_to_usec(
            properties::get_double(cfg_robot, "period", 0.1e-3, 1.)
        );
        _robot.reset(new RobotSim(q0, period));
    }

    void init_state_publisher()
    {
        auto const& cfg_pub = properties::get("publisher");
        _pub_period = sec_to_usec(
            properties::get_double(cfg_pub, "period", 1e-3, 1e+3)
        );
        auto const& topic = properties::get_string(cfg_pub, "topic");
        _pub.reset(new PosePublisher(*_node, topic));
    }

    void command_handler(std_msgs::Float64MultiArrayConstPtr const& msg)
    {
        try
        {
            if (!_robot || !_robot->is_ran())
            {
                std::cerr << "robot is not initialized" << std::endl;
                return;
            }

            if (_robot->is_busy())
            {
                std::cerr << "robot is busy" << std::endl;
                return;
            }

            if (msg->data.size() != kinematic_model::njoints())
            {
                std::cerr << "an incorrect request received" << std::endl;
                return;
            }

            Joints q_target = Joints::from_vec(msg->data);
            std::cout << "received request: movej " << q_target << std::endl;
            _robot->movej(q_target);
        }
        catch (std::exception const& e)
        {
            std::cerr << "failed processing request: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "failed processing request: undef" << std::endl;
        }
    }

    void init_command_handler()
    {
        auto const& cfg_interface = properties::get("interface");
        auto const& topic = properties::get_string(cfg_interface, "topic");
        _sub = _node->subscribe(topic, 1, &Application::command_handler, this);
    }

    void actual_robot_pose(Isometry3d& pose, int64_t& ts)
    {
        Joints q;
        _robot->actual_position(q, ts);
        pose = kinematic_model::solvefk(q);
    }

public:
    Application(int argc, char const* argv[])
    {
        ros::init(argc, (char**)argv, "robotsim");
        _node.reset(new ros::NodeHandle());

        if (argc != 2)
            throw_invalid_arg("expect config as the first argument");

        properties::init(argv[1]);
        kinematic_model::init();
        init_robot();
        init_state_publisher();
        init_command_handler();
    }

    ~Application()
    {
        _sub.shutdown();
    }

    void run()
    {
        _robot->run();
        _stop = false;
        int64_t ts = (epoch_usec() / _pub_period) * _pub_period;

        std::cout << "waiting for incoming commands.." << std::endl;

        while (!_stop)
        {
            ts += _pub_period;
            sleep_usec(ts - epoch_usec());
            if (!_robot->is_ran())
                continue;

            Isometry3d pose;
            int64_t robot_ts;
            actual_robot_pose(pose, robot_ts);
            _pub->send(robot_ts, pose);
            ros::spinOnce();
        }
    }

    void stop()
    {
        _stop = true;
        if (_robot)
            _robot->stop();
    }
};

int main(int argc, char const* argv[])
{
    try
    {
        Application app(argc, argv);
        auto f = [&app]() {
            std::cout << "stopping.." << std::endl;
            app.stop();
        };
        signals::set_sigterm_handler(f);
        signals::set_sigint_handler(f);
        app.run();
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

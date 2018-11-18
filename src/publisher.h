#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "transforms.h"


inline geometry_msgs::Pose eigen2msg(Isometry3d const& pose)
{
    geometry_msgs::Pose msg;
    auto p = pose.translation();
    auto q = Eigen::Quaterniond(pose.linear());
    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.position.x = p.x();
    msg.position.y = p.y();
    msg.position.z = p.z();
    return msg;
}

inline Isometry3d msg2eigen(geometry_msgs::Pose const& msg)
{
    Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    Eigen::Vector3d p(msg.position.x, msg.position.y, msg.position.z);
    Isometry3d T = Isometry3d::Identity();
    T.translate(p);
    T.rotate(q);
    return T;
}

class PosePublisher
{
private:
    ros::Publisher _pub;
public:
    PosePublisher(ros::NodeHandle& node, std::string const& topic)
    {
        _pub = node.advertise<geometry_msgs::PoseStamped>(topic, 1, true);
    }

    ~PosePublisher()
    {
        _pub.shutdown();
    }

    void send(int64_t ts, Isometry3d const& pose)
    {
        geometry_msgs::PoseStamped msg;
        msg.pose = eigen2msg(pose);
        msg.header.stamp.fromNSec(ts * 1000);
        _pub.publish(msg);
        // std::cout << "[debug] publishing robot state: " << ts << " " << pose << std::endl;
    }
};

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "../src/timeutils.h"
#include "../src/publisher.h"
#include "../src/throws.h"


void test1()
{
    auto T = maketransform(1,2,3,0.4,0.5,0.6);
    auto msg = eigen2msg(T);
    auto T1 = msg2eigen(msg);
    if (!transeq(T, T1))
        throw_runtime_error("eigen to ros messages conversion doesn't work");
}

struct TestPoseReceiver
{
    Isometry3d T = Isometry3d::Identity();
    double t = -1;

    void cb(geometry_msgs::PoseStamped::ConstPtr const& msg)
    {
        T = msg2eigen(msg->pose);
        t = msg->header.stamp.toSec();
    };
};

void test2(int argc, char **argv)
{
    ros::init(argc, argv, "test_pose_publish");
    ros::NodeHandle node;
    char const* topic = "test_pose_publisher";
    auto T = maketransform(1,2,3,4,5,6);
    TestPoseReceiver receiver;
    ros::Subscriber sub = node.subscribe(topic, 1, &TestPoseReceiver::cb, &receiver);
    PosePublisher pub(node, topic);
    pub.send(1, T);
    sleep_sec(0.1);
    ros::spinOnce();
    if (receiver.t < 0.)
        throw_runtime_error("couldn't receive pose");

    if (!transeq(receiver.T, T))
        throw_runtime_error("sent data corrupted");
}

struct TestMultiArrReceiver
{
    std::vector<double> arr;
    bool received = false;

    void cb(std_msgs::Float64MultiArrayConstPtr const& msg)
    {
        arr = msg->data;
        received = true;
    }
};

void test3(int argc, char **argv)
{
    ros::init(argc, argv, "test_multiarr_subscr");
    ros::NodeHandle node;
    char const* topic = "test_multiarr_subscriber";

    TestMultiArrReceiver receiver;
    ros::Subscriber sub = node.subscribe(topic, 1, &TestMultiArrReceiver::cb, &receiver);
    ros::Publisher pub = node.advertise<std_msgs::Float64MultiArray>(topic, 1, true);
    std_msgs::Float64MultiArray msg;
    msg.data = {1,2,3,4,5,6};
    pub.publish(msg);
    sleep_sec(0.1);
    ros::spinOnce();
    if (!receiver.received)
        throw_runtime_error("couldn't receive arr");
    
    if (receiver.arr.size() != msg.data.size())
        throw_runtime_error("sent data corrupted");
}

int main(int argc, char **argv)
{
    try
    {
        test1();
        test2(argc, argv);
        test3(argc, argv);
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

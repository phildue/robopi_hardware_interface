//
// Created by phil on 21.03.20.
//
#include <ros/ros.h>
#include <pi_ln298n/pi_ln298n.h>
#include <chrono>
#include "std_msgs/Float32.h"
constexpr int enA = 21;
constexpr int enB = 13;
constexpr int in1 = 20;
constexpr int in2 = 16;
constexpr int in3 = 26;
constexpr int in4 = 19;
using namespace pi_ln298n;
std::shared_ptr<GpioMotor> left,right;

void leftCallBack(const std_msgs::Float32::ConstPtr& msg)
{
   ROS_INFO("I heard: [%f]", msg->data);

    left->set(msg->data);
 }

void rightCallBack(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("I heard: [%f]", msg->data);

    right->set(msg->data);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpio_motor");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    left = std::make_shared<GpioMotor>(in4, in3, enB);
    right = std::make_shared<GpioMotor>(in1, in2, enA);

    ros::Subscriber subLeft = n.subscribe("gpio_left", 10, leftCallBack);
    ros::Subscriber subRight = n.subscribe("gpio_right", 10, rightCallBack);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    left->stop();
    right->stop();
}
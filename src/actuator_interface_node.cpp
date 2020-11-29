//
// Created by phil on 22.03.20.
//

#include <controller_manager/controller_manager.h>
#include "ActuatorInterfaceEffort.h"
#include "ActuatorInterfaceVelocity.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
int main(int argc, char** argv) {
    ros::init(argc, argv, "actuator_interface");
    ros::CallbackQueue callbackQueue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&callbackQueue);
    std::string interfaceType;
    nh.getParam("/robopi/actuator_interface/interface_type", interfaceType);

    if (interfaceType == "effort") {
        ROS_INFO("Hardware interface set to effort.");
        ActuatorInterfaceEffort rhi(nh);
        ros::MultiThreadedSpinner spinner(0);
        spinner.spin(&callbackQueue);
        return 0;
    }else{
        ROS_INFO("Hardware interface set to velocity.");
        ActuatorInterfaceVelocity rhi(nh);
        ros::MultiThreadedSpinner spinner(0);
        spinner.spin(&callbackQueue);
        return 0;
    }
    //ros::AsyncSpinner spinner(0,&callbackQueue);
    //spinner.start();
    //ros::waitForShutdown();

}
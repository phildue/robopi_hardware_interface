//
// Created by phil on 22.03.20.
//

#include <controller_manager/controller_manager.h>
#include "HardwareInterfaceArduino.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
int main(int argc, char** argv) {
    ros::init(argc, argv, "hardware_interface");
    ros::CallbackQueue callbackQueue;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&callbackQueue);
    ROS_INFO("Velocity interface started.");
    HardwareInterfaceArduino rhi(nh);
    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&callbackQueue);


}
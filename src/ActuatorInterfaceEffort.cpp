//
// Created by phil on 22.03.20.
//

#include "ActuatorInterfaceEffort.h"

//TODO move this to config
constexpr int enA = 21;
constexpr int enB = 13;
constexpr int in1 = 20;
constexpr int in2 = 16;
constexpr int in3 = 26;
constexpr int in4 = 19;

ActuatorInterfaceEffort::ActuatorInterfaceEffort(ros::NodeHandle &nh) : _nh(nh)
                                                            {
    init();
    _ctrlManager.reset(new controller_manager::ControllerManager(this, _nh));
    _nh.param("/robopi/actuator_interface/loop_hz", _loopHz, 0.1);
    ros::Duration update_freq = ros::Duration(1.0 / _loopHz);
    _nonRealTimeLoop = _nh.createTimer(update_freq, &ActuatorInterfaceEffort::update, this);


}

void ActuatorInterfaceEffort::init()
{
    _nh.getParam("/robopi/actuator_interface/wheels", _wheelNames);
    for(const auto &wheelName : _wheelNames)
    {
        ROS_INFO("Found wheel: [%s]",wheelName.c_str());
        int en,inF,inB;
        _nh.getParam("/robopi/gpio_wheels/" + wheelName + "/en", en);
        _nh.getParam("/robopi/gpio_wheels/" + wheelName + "/inF", inF);
        _nh.getParam("/robopi/gpio_wheels/" + wheelName + "/inB", inB);
        _wheels.insert({wheelName, pi_ln298n::GpioMotor(inF, inB, en)});
    }
    _numJoints = _wheelNames.size();

    _jointEffort.resize(_numJoints);
    _jointPosition.resize(_numJoints);
    _jointVelocity.resize(_numJoints);
    _jointEffortCommand.resize(_numJoints);
    for (int i = 0; i < _numJoints; ++i) {

        JointStateHandle jointStateHandle(_wheelNames[i], &_jointPosition[i], &_jointVelocity[i], &_jointEffort[i]);
        _jointStateInterface.registerHandle(jointStateHandle);
        ROS_INFO("[%s]: Registered joint state handle.",_wheelNames[i].c_str());

        JointHandle jointEffortHandle(jointStateHandle, &_jointEffortCommand[i]);
        JointLimits limits;
        SoftJointLimits softLimits;
        getJointLimits(_wheelNames[i], _nh, limits);
        getSoftJointLimits(_wheelNames[i], _nh, softLimits);
        joint_limits_interface::EffortJointSoftLimitsHandle jointLimitsHandle(jointEffortHandle, limits, softLimits);
        _effortJointSoftLimitInterface.registerHandle(jointLimitsHandle);
        _effortJointInterface.registerHandle(jointEffortHandle);
        ROS_INFO("[%s]: Registered joint effort handle.",_wheelNames[i].c_str());
    }

    registerInterface(&_jointStateInterface);
    ROS_INFO("Registered joint state interface");

    registerInterface(&_effortJointInterface);
    ROS_INFO("Registered joint effort interface");
    registerInterface(&_effortJointSoftLimitInterface);
    ROS_INFO("Registered joint effort limits interface");
}

void ActuatorInterfaceEffort::update(const ros::TimerEvent& e) {

    _elapsedTime = ros::Duration(e.current_real - e.last_real);
    //ROS_INFO("Update [%d]: (%f,%f)",_elapsedTime.nsec,_jointEffortCommand[0],_jointEffortCommand[1]);
    read();
    _ctrlManager->update(ros::Time::now(), _elapsedTime);
    write(_elapsedTime);
}

void ActuatorInterfaceEffort::read() {
    for (int i = 0; i < _numJoints; i++) {
        _jointEffort[i] = _jointEffortCommand[i];
    }
}

void ActuatorInterfaceEffort::write(ros::Duration elapsed_time) {
    _effortJointSoftLimitInterface.enforceLimits(elapsed_time);
    for (int i = 0; i < _numJoints; i++) {
        _wheels.find(_wheelNames[i])->second.set(_jointEffortCommand[i]);
    }
}
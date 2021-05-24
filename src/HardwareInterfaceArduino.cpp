//
// Created by phil on 22.03.20.
//

#include <std_msgs/Float32MultiArray.h>
#include "HardwareInterfaceArduino.h"


HardwareInterfaceArduino::HardwareInterfaceArduino(ros::NodeHandle &nh) : _nh(nh)
                                                            {
    init();
    _ctrlManager.reset(new controller_manager::ControllerManager(this, _nh));
    _nh.param("/robopi/hardware_interface_arduino/loop_hz", _loopHz, 0.1);
    _deviceName = "/dev/ttyACM0";
    _nh.param("/robopi/hardware_interface_arduino/device_name",_deviceName);
    _serial._serialPort.SetDevice(_deviceName);
    _serial._serialPort.SetBaudRate(mn::CppLinuxSerial::BaudRate::B_9600);
    _serial._serialPort.SetTimeout(10);
    _serial._serialPort.Open();
    ros::Duration update_freq = ros::Duration(1.0 / _loopHz);
    _nonRealTimeLoop = _nh.createTimer(update_freq, &HardwareInterfaceArduino::update, this);

}

void HardwareInterfaceArduino::init()
{
    _nh.getParam("/robopi/hardware_interface_arduino/wheels", _wheelNames);
    for(const auto &wheelName : _wheelNames)
    {
        ROS_INFO("Found wheel: [%s]",wheelName.c_str());
    }
    _numJoints = _wheelNames.size();

    _jointEffort.resize(_numJoints);
    _jointPosition.resize(_numJoints);
    _jointVelocity.resize(_numJoints);
    _jointVelocityCommand.resize(_numJoints);
    for (int i = 0; i < _numJoints; ++i) {

        JointStateHandle jointStateHandle(_wheelNames[i], &_jointPosition[i], &_jointVelocity[i], &_jointEffort[i]);
        _jointStateInterface.registerHandle(jointStateHandle);
        ROS_INFO("[%s]: Registered joint state handle.",_wheelNames[i].c_str());

        JointHandle jointVelocityHandle(jointStateHandle, &_jointVelocityCommand[i]);
        JointLimits limits;
        SoftJointLimits softLimits;
        getJointLimits(_wheelNames[i], _nh, limits);
        getSoftJointLimits(_wheelNames[i], _nh, softLimits);
        joint_limits_interface::VelocityJointSoftLimitsHandle jointLimitsHandle(jointVelocityHandle, limits, softLimits);
        _velocityJointSoftLimitInterface.registerHandle(jointLimitsHandle);
        _velocityJointInterface.registerHandle(jointVelocityHandle);
        ROS_INFO("[%s]: Registered joint velocity handle.",_wheelNames[i].c_str());
    }

    registerInterface(&_jointStateInterface);
    ROS_INFO("Registered joint state interface");

    registerInterface(&_velocityJointInterface);
    ROS_INFO("Registered joint velocity interface");
    registerInterface(&_velocityJointSoftLimitInterface);
    ROS_INFO("Registered joint velocity limits interface");
}

void HardwareInterfaceArduino::update(const ros::TimerEvent& e) {

    _elapsedTime = ros::Duration(e.current_real - e.last_real);

    read(e);
    //ROS_INFO("Update [%d]: (%f,%f)",_elapsedTime.nsec,_jointEffortCommand[0],_jointEffortCommand[1]);
    _ctrlManager->update(ros::Time::now(), _elapsedTime);
    _velocityJointSoftLimitInterface.enforceLimits(_elapsedTime);
    write(e);



}

void HardwareInterfaceArduino::read(const ros::TimerEvent &e) {

    try{
        _serial.read(1);

    }catch(const SerialProtocol::ParseError& e)
    {
        ROS_ERROR("SerialProtocol::ParserError:: %s [%s]",e.what(),e._parsedMessage.c_str());

    }
    if ( ! _serial._messagesState.empty() )
    {
        std::shared_ptr<const SerialProtocol::MsgState> msg = _serial._messagesState[_serial._messagesState.size() - 1];
        _jointVelocity[0] = msg->_stateLeft.angularVelocity;
        _jointVelocity[1] = msg->_stateRight.angularVelocity;
        _jointPosition[0] = msg->_stateLeft.position;
        _jointPosition[1] = msg->_stateRight.position;
//        _jointPosition[0] += msg->_stateLeft.angularVelocity * _elapsedTime.toSec();
//        _jointPosition[1] += msg->_stateRight.angularVelocity * _elapsedTime.toSec();
        //ROS_INFO( "Message:\n %s", msg->str().c_str());
    }
    _serial.clear();

}

void HardwareInterfaceArduino::write(const ros::TimerEvent &e) {
    if (_jointVelocityCommand[0] != 0 || _jointVelocityCommand[1] != 0)
    {
        _serial.send( std::make_shared<SerialProtocol::MsgCmdVel>(_jointVelocityCommand[0],_jointVelocityCommand[1],e.current_real.toNSec() /1000));

    }
}




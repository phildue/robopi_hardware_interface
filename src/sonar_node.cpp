//
// Created by phil on 12.12.20.
//
// https://www.intorobotics.com/how-to-use-sensor_msgs-range-ros-for-multiple-sensors-with-rosserial/

#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>

//#include "robopi_drivers/robopi_drivers.h"


int main(int argc, char **argv)
{
/*
  create Kalman filter objects for the sensors.
   SimpleKalmanFilter(e_mea, e_est, q);
   e_mea: Measurement Uncertainty
   e_est: Estimation Uncertainty
   q: Process Noise
*/
//    SimpleKalmanFilter KF_Left(2, 2, 0.01);
//    SimpleKalmanFilter KF_Center(2, 2, 0.01);
//    SimpleKalmanFilter KF_Right(2, 2, 0.01);

    ros::init(argc, argv, "sonar");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(n.param("/robopi/sonar/publish_rate",50));
    ros::Publisher pub_range_left = n.advertise<sensor_msgs::Range>("/robopi/sensors/sonar/front", 10);


    sensor_msgs::Range range_left;
    range_left.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_left.header.frame_id = "sonar_front";
    range_left.field_of_view = 0.26;
    range_left.min_range = 0.0;
    range_left.max_range = 4.0;


    //robopi::SonarHcsr04 sonar(n.param("robopi/sonar/trigger",6),n.param("robopi/sonar/echo",12));

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        range_left.header.stamp = ros::Time::now();
   //     range_left.range = sonar.measure().distance();
        pub_range_left.publish(range_left);

    }

}





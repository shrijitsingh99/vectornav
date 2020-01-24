//
// Created by shrijitsingh99 on 9/14/19.
//

#include <iostream>
#include "ros/ros.h"
#include "vectornav/vectornav_interface.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "vectornav");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    auto pubIMU = n.advertise<sensor_msgs::Imu>("vectornav/IMU", 1000);
    auto pubGPS = n.advertise<sensor_msgs::NavSatFix>("vectornav/GPS", 1000);
    auto pubOdom = n.advertise<nav_msgs::Odometry>("vectornav/Odom", 1000);

    std::string sensor_port;
    int sensor_baud_rate;

    pn.param<std::string>("serial_port", sensor_port, "/dev/ttyUSB0");
    pn.param<int>("serial_baud", sensor_baud_rate, 115200);


    VectorNavInterface vn_interface(sensor_port, sensor_baud_rate);

    sensor_msgs::Imu imu_msg;
    sensor_msgs::NavSatFix gps_msg;
    nav_msgs::Odometry odom_msg;

    ros::Rate rate(30);
    while (ros::ok()) {
        bool imu_valid = true;
        bool gps_valid = true;
        bool odom_valid = true;

        if (imu_valid)
            pubIMU.publish(*vn_interface.imu_msg);
        if (gps_valid)
            pubGPS.publish(*vn_interface.gps1_msg);
        if (odom_valid)
            pubOdom.publish(*vn_interface.odom_msg);

        rate.sleep();

        ros::spinOnce();
    }
}
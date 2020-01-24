//
// Created by shrijitsingh99 on 9/14/19.
//

#ifndef VECTORNAV_VECTORNAV_INTERFACE_H
#define VECTORNAV_VECTORNAV_INTERFACE_H

#include "vn/ezasyncdata.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/LinearMath/Transform.h"

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

typedef enum data_source {
    INS,
    GPS1,
    GPS2
} gps_source;

class VectorNavInterface {
  public:
    VnSensor vs;
    CompositeData cd;

    tf2::Transform transform_odom2base_= tf2::Transform(tf2::Transform::getIdentity());
    tf2::Transform transform_odom2base_inverse_= tf2::Transform(tf2::Transform::getIdentity());
    tf2::Transform transform_utm2odom_= tf2::Transform(tf2::Transform::getIdentity());
    tf2::Transform transform_utm2odom_inverse_= tf2::Transform(tf2::Transform::getIdentity());
    tf2::Transform transform_utm2nav_;
    tf2::Transform transform_utm2nav_inverse_;

    vec3d orig_pos;
    vec4f orig_quat;
    bool orig_pos_set = false;
    bool remove_gravity_accel = true;

    sensor_msgs::Imu *imu_msg;
    sensor_msgs::NavSatFix *ins_msg;
    sensor_msgs::NavSatFix *gps1_msg;
    sensor_msgs::NavSatFix *gps2_msg;
    nav_msgs::Odometry *odom_msg;

    CommonGroup COMMONGROUP = COMMONGROUP_NONE | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES;
    TimeGroup TIMEGROUP = TIMEGROUP_NONE;
    ImuGroup IMUGROUP = IMUGROUP_NONE | IMUGROUP_TEMP | IMUGROUP_MAG | IMUGROUP_ANGULARRATE;
    GpsGroup GPSGROUP = GPSGROUP_NONE | GPSGROUP_POSLLA;
    AttitudeGroup ATTITUDEGROUP = ATTITUDEGROUP_NONE | ATTITUDEGROUP_YPRU | ATTITUDEGROUP_QUATERNION;
    InsGroup INSGROUP = INSGROUP_NONE | INSGROUP_INSSTATUS;
    GpsGroup GPS2GROUP = GPSGROUP_NONE | GPSGROUP_POSLLA;

    static void packet_received(void* userData, Packet& p, unsigned long index);
    void configure_binary_output();
    bool parse_imu_msg();
    bool parse_gps_msg(data_source src);
    bool parse_odometry_msg();

    VectorNavInterface(std::string port, uint32_t baud_rate);
    ~VectorNavInterface();


};


#endif //VECTORNAV_VECTORNAV_INTERFACE_H

//
// Created by shrijitsingh99 on 9/14/19.
//

#include "vectornav/vectornav_interface.h"
#include "vectornav/UTM.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

VectorNavInterface::VectorNavInterface(const std::string port, const uint32_t baud_rate) {
    vs.connect(port, baud_rate);
    configure_binary_output();
    vs.registerAsyncPacketReceivedHandler(this, &VectorNavInterface::packet_received);
    imu_msg = new sensor_msgs::Imu;
    gps1_msg = new sensor_msgs::NavSatFix;
    gps2_msg = new sensor_msgs::NavSatFix;
    ins_msg = new sensor_msgs::NavSatFix;
    odom_msg = new nav_msgs::Odometry;

}

VectorNavInterface::~VectorNavInterface() {
    vs.unregisterAsyncPacketReceivedHandler();
    vs.disconnect();
}

void VectorNavInterface::packet_received(void* userData, Packet& p, unsigned long index) {\
    if (p.type() != Packet::TYPE_BINARY)
        return;

    auto* vn = static_cast<VectorNavInterface*>(userData);
    vn->cd = CompositeData::parse(p);

    vn->parse_imu_msg();
    vn->parse_gps_msg(GPS1);
    vn->parse_odometry_msg();

}

void VectorNavInterface::configure_binary_output() {

    COMMONGROUP = COMMONGROUP_NONE;
    TIMEGROUP = TIMEGROUP_NONE;
    IMUGROUP = IMUGROUP_NONE | IMUGROUP_ANGULARRATE;
    GPSGROUP = GPSGROUP_NONE | GPSGROUP_POSLLA | GPSGROUP_POSU;
    ATTITUDEGROUP = ATTITUDEGROUP_NONE | ATTITUDEGROUP_YAWPITCHROLL | ATTITUDEGROUP_YPRU | ATTITUDEGROUP_QUATERNION | ATTITUDEGROUP_LINEARACCELBODY | ATTITUDEGROUP_LINEARACCELNED;
    INSGROUP = INSGROUP_INSSTATUS | INSGROUP_POSU | INSGROUP_POSLLA | INSGROUP_VELBODY | INSGROUP_VELBODY;

    GPS2GROUP = GPSGROUP_POSLLA | GPSGROUP_POSU | GPSGROUP_FIX;

    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            20,
            COMMONGROUP,
            TIMEGROUP,
            IMUGROUP,
            GPSGROUP,
            ATTITUDEGROUP,
            INSGROUP,
            GPSGROUP_NONE
    );

    vs.writeBinaryOutput1(bor);
}

bool VectorNavInterface::parse_imu_msg() {
    bool has_data = cd.hasQuaternion() && cd.hasAttitudeUncertainty() &&
            cd.hasAngularRate() &&
            (cd.hasAccelerationLinearBody() || cd.hasAccelerationLinearNed());


    if (has_data) {
        // Orientation
        vec4f q = cd.quaternion();
        imu_msg->orientation.x = q[0];
        imu_msg->orientation.y = q[1];
        imu_msg->orientation.z = q[2];
        imu_msg->orientation.w = q[3];

        // Orientation Covariance
        vec3f orientation_uncertainty = cd.attitudeUncertainty();
        imu_msg->orientation_covariance[0] = pow(orientation_uncertainty[0], 2) * M_PI / 180.0;
        imu_msg->orientation_covariance[1] = pow(orientation_uncertainty[1], 2) * M_PI / 180.0;
        imu_msg->orientation_covariance[2] = pow(orientation_uncertainty[2], 2) * M_PI / 180.0;

        // Angular Velocity
        vec3f angular_vel = cd.angularRate();
        imu_msg->angular_velocity.x = angular_vel[0];
        imu_msg->angular_velocity.y = angular_vel[1];
        imu_msg->angular_velocity.z = angular_vel[2];

        // Acceleration
        vec3f accel = remove_gravity_accel ? cd.accelerationLinearBody() : cd.accelerationNed();
        imu_msg->linear_acceleration.x = accel[0];
        imu_msg->linear_acceleration.y = accel[1];
        imu_msg->linear_acceleration.z = accel[2];

        // TODO (shrijitsingh99): Add angular velocity & acceleration uncertainty

        return true;
    }

    return false;
}

bool VectorNavInterface::parse_gps_msg(data_source src) {
    bool has_data = cd.hasAnyPosition();

    sensor_msgs::NavSatFix *gps_msg;

    sensor_msgs::NavSatStatus gps_status;

    uint8_t covariance_type = 0;
    vec3d pos;
    mat33f covariance;

    switch (src) {
        case INS:
            if ((has_data = has_data && cd.hasPositionEstimatedLla())) {
                pos = cd.positionEstimatedLla();
                covariance_type = 1;
                covariance.e00 = covariance.e11 = covariance.e22 = pow(cd.positionUncertaintyEstimated(), 2);
                gps_msg = ins_msg;
            }
            break;
        case GPS1:
            if ((has_data = has_data && cd.hasPositionEstimatedLla())) {
                pos = cd.positionGpsLla();
                covariance_type = 2;
//                covariance.e00 = pow(cd.positionUncertaintyGpsNed()[1], 2);
//                covariance.e11 = pow(cd.positionUncertaintyGpsNed()[0], 2);
//                covariance.e22 = pow(cd.positionUncertaintyGpsNed()[2], 2);
//                gps_status.status = cd.fix() < 2 ? -1 : 0;
                gps_msg = gps1_msg;
            }
            break;
        case GPS2:
            if ((has_data = has_data && cd.hasPositionGps2Lla() && cd.hasPositionUncertaintyGps2Ned())) {
                pos = cd.positionGps2Lla();
                covariance_type = 2;
                covariance.e00 = pow(cd.positionUncertaintyGpsNed()[1], 2);
                covariance.e11 = pow(cd.positionUncertaintyGpsNed()[0], 2);
                covariance.e22 = pow(cd.positionUncertaintyGpsNed()[2], 2);
                gps_status.status = cd.fix2() < 2 ? -1 : 0;
                gps_msg = gps2_msg;
            }
            break;
        default:
            has_data = false;
    }

    if (has_data) {
        gps_msg->latitude = pos[0];
        gps_msg->longitude = pos[1];
        gps_msg->altitude = pos[2];

        gps_msg->position_covariance_type = covariance_type;

        gps_msg->status = gps_status;

        return true;
    }

    return false;
}


bool VectorNavInterface::parse_odometry_msg() {
    bool has_data = cd.hasPositionGpsLla() && cd.hasYawPitchRoll() && cd.hasVelocityEstimatedBody()
            && cd.hasAngularRate();

    if (!has_data)
        return false;

    vec3d pos = cd.positionGpsLla();
    vec4f q = cd.quaternion();
    vec3f ypr = cd.yawPitchRoll();

    vec3f linear_vel = cd.velocityEstimatedBody();
    vec3f angular_vel = cd.angularRate();

//        q[0] = q.y;
//        q[1] = q.x;
//        q[2] = -q.z;
//        q[3] = q.w;

    if (!orig_pos_set) {
        LatLonToUTMXY(pos[0], pos[1], 43, orig_pos[1], orig_pos[0]);
        orig_pos[2] = pos[2];

        transform_utm2odom_.setOrigin(tf2::Vector3(orig_pos[0], orig_pos[1], orig_pos[2]));
        transform_utm2odom_.setRotation(tf2::Quaternion(q[0], q[1], q[2], q[3]));
        transform_utm2odom_inverse_ = transform_utm2odom_.inverse();

        orig_quat[0] = q.y;
        orig_quat[1] = q.x;
        orig_quat[2] = -q.z;
        orig_quat[3] = q.w;
        orig_pos_set = true;
    }




    vec3d new_pos;
    LatLonToUTMXY(pos[0], pos[1], 43, new_pos[0], new_pos[1]);
    new_pos[2] = pos[2];


    transform_utm2nav_.setOrigin(tf2::Vector3(new_pos[1], new_pos[0],
                                              new_pos[2]));
    transform_utm2nav_.setRotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    transform_utm2nav_inverse_=transform_utm2nav_.inverse();

    transform_odom2base_.mult(transform_utm2odom_inverse_,transform_utm2nav_);

    printf("%f\n", ypr[0]);
    tf2::toMsg(transform_odom2base_, odom_msg->pose.pose);


    tf2::Quaternion o_q(orig_quat[0], orig_quat[1], orig_quat[2], orig_quat[3]);
    tf2::Quaternion n_q(q[1], q[0], -q[2], q[3]);
    n_q = n_q * o_q.inverse();

    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
//    odom_msg->pose.pose.position.x = x;
//    odom_msg->pose.pose.position.y = y;
    odom_msg->pose.pose.position.y *= -1;
    odom_msg->pose.pose.position.z =  0.0;

    odom_msg->pose.pose.orientation.w = n_q[3];
    odom_msg->pose.pose.orientation.x = n_q[0];
    odom_msg->pose.pose.orientation.y = n_q[1];
    odom_msg->pose.pose.orientation.z = n_q[2];

    odom_msg->twist.twist.linear.x = linear_vel[0];
    odom_msg->twist.twist.linear.y = linear_vel[1];
    odom_msg->twist.twist.linear.z = linear_vel[2];

    odom_msg->twist.twist.angular.x = angular_vel[0];
    odom_msg->twist.twist.angular.y = angular_vel[1];
    odom_msg->twist.twist.angular.z = angular_vel[2];


    return true;
}
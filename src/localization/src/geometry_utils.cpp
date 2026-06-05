#include "localization/geometry_utils.hpp"

#include "tf2/LinearMath/Quaternion.h"

#include <cmath>

double normalizeAngle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::Quaternion msg;
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();
    msg.w = quaternion.w();
    return msg;
}


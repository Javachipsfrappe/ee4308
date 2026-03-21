
#include "rclcpp/rclcpp.hpp"

#pragma once

namespace ee4308::turtle2 {
    // Constrains angles to -pi (inclusive) and pi (exclusive) radians.
    double limitAngle(const double &angle)
    {
        double result = std::fmod(angle + M_PI, M_PI * 2); // fmod rounds remainders to zero. we want remainders to be +ve like mod() in matlab and % in python
        return result >= 0 ? result - M_PI : result + M_PI;
    }

    // Finds the yaw from a quaternion.
    double getYawFromQuaternion(const double &qx, const double &qy, const double &qz, const double &qw)
    {
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // Finds the yaw from a quaternion-like object (must have `x`, `y`, `z`, `w` properties)
    template <typename T>
    double getYawFromQuaternion(const T &quaternion)
    {
        return getYawFromQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    }

    template <typename T, typename U>
    rclcpp::Parameter getParameter(const U& node, const std::string &name, const T &default_value)
    {
        node->declare_parameter(name, default_value);
        rclcpp::Parameter parameter = node->get_parameter(name);
        RCLCPP_INFO_STREAM(node->get_logger(),
                           "[Parameter] " << name << ": " << parameter);
        return parameter;
    }
};
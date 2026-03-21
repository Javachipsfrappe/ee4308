#include <cmath>
#include "rclcpp/rclcpp.hpp"

#pragma once
namespace ee4308
{
    const double THRES = 1e-8; // for floating point calculations.

    // Finds the sign of a value
    template <typename T>
    T sgn(const T &value) { return (T(0) < value) - (value < T(0)); }

    // Constrains angles to -pi (inclusive) and pi (exclusive) radians.
    double limitAngle(const double &angle)
    {
        double result = std::fmod(angle + M_PI, M_PI * 2); // fmod rounds remainders to zero. we want remainders to be +ve like mod() in matlab and % in python
        return result >= 0 ? result - M_PI : result + M_PI;
    }

    // Gets the distance between two coordinates
    double getDistance(const double &x1, const double &y1, const double &x2, const double &y2)
    {
        return std::hypot(x2 - x1, y2  - y1);
    }

    template <typename T>
    double getDistance(const T& position1, const T&position2)
    {
        return ee4308::getDistance(position1.x, position1.y, position2.x, position2.y);
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
        return ee4308::getYawFromQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    }

    // Calculates the quaternion values from yaw, assuming zero roll and pitch.
    void getQuaternionFromYaw(const double &yaw, double &qx, double &qy, double &qz, double &qw)
    {
        qw = std::cos(yaw / 2);
        qx = 0;
        qy = 0;
        qz = std::sin(yaw / 2);
    }

    // Calculates the quaternion values from yaw, assuming zero roll and pitch.
    template <typename T>
    void getQuaternionFromYaw(const double &yaw, T &quaternion)
    {
        ee4308::getQuaternionFromYaw(yaw, quaternion.x, quaternion.y, quaternion.z, quaternion.w);
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
}

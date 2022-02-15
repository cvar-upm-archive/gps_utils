#pragma once

#include <iostream>

#include "GpsUtils.hpp"

#include "as2_core/node.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


class GpsTranslator : public as2::Node {
public:
    GpsTranslator();
    GpsTranslator(GpsUtils utils);
    GpsTranslator(double lat, double lon, double alt);
private:
    GpsUtils utils_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_fix_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_ecef_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pub_;

private:
    void translateCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
}; // GpsTranslator class
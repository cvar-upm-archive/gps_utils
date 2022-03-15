#pragma once

#include <iostream>

#include "GpsUtils.hpp"

#include "as2_core/node.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "as2_msgs/srv/set_origin.hpp"
#include "as2_msgs/srv/get_origin.hpp"
#include "as2_msgs/srv/geopath_to_path.hpp"
#include "as2_msgs/srv/path_to_geopath.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include <rclcpp/node_options.hpp>

namespace GpsTranslator {

class GpsTranslator : public as2::Node {
public:
    GpsTranslator(const rclcpp::NodeOptions & options);
    GpsTranslator(GpsUtils utils);
    GpsTranslator(double lat, double lon, double alt);
private:
    GpsUtils utils_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_fix_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_ecef_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pub_;

    rclcpp::Service<as2_msgs::srv::SetOrigin>::SharedPtr set_origin_srv_;
    rclcpp::Service<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

    rclcpp::Service<as2_msgs::srv::GeopathToPath>::SharedPtr geopath_to_path_srv_;
    rclcpp::Service<as2_msgs::srv::PathToGeopath>::SharedPtr path_to_geopath_srv_;

private:
    void translateCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void setOriginCb(const std::shared_ptr<as2_msgs::srv::SetOrigin::Request> request, 
                     std::shared_ptr<as2_msgs::srv::SetOrigin::Response> response);
    void getOriginCb(const std::shared_ptr<as2_msgs::srv::GetOrigin::Request> request, 
                     std::shared_ptr<as2_msgs::srv::GetOrigin::Response> response);

    void geopathToPathCb(const std::shared_ptr<as2_msgs::srv::GeopathToPath::Request> request, 
                     std::shared_ptr<as2_msgs::srv::GeopathToPath::Response> response);
    void pathToGeopathCb(const std::shared_ptr<as2_msgs::srv::PathToGeopath::Request> request, 
                     std::shared_ptr<as2_msgs::srv::PathToGeopath::Response> response);
}; // GpsTranslator class

} // namespace GpsTranslator
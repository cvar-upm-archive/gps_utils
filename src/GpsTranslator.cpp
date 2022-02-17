#include "GpsTranslator.hpp"

GpsTranslator::GpsTranslator(): as2::Node("gps_translator") {
    global_fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        this->generate_global_name("global_pose/fix"), 10, 
        std::bind(&GpsTranslator::translateCb, this, std::placeholders::_1)
    );
    global_ecef_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->generate_global_name("global_pose/ecef"), 10
    );
    local_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        this->generate_global_name("local_pose"), 10
    );
}

GpsTranslator::GpsTranslator(double lat, double lon, double alt): as2::Node("gps_translator") {
    GpsTranslator();
    this->utils_.SetOrigin(lat, lon, alt);
}

GpsTranslator::GpsTranslator(GpsUtils utils): as2::Node("gps_translator") { 
    double lat, lon, alt;
    utils.GetOrigin(lat, lon, alt);
    GpsTranslator(lat, lon, alt);
}

void GpsTranslator::translateCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped local_msg;
    geometry_msgs::msg::PoseStamped global_msg;

    try{
        utils_.LatLon2Local(*msg, local_msg);
        local_msg.header.stamp = msg->header.stamp;
        local_pub_->publish(local_msg);
    } catch (std::exception &e) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Local transformation not available since origin is not set.");
        // std::cout << e.what() << std::endl;
    }

    utils_.LatLon2Ecef(*msg, global_msg);
    global_msg.header.stamp = msg->header.stamp;
    global_ecef_pub_->publish(global_msg);
}



int main() { 
    std::cout << "hello" << std::endl;
}
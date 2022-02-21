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

    set_origin_srv_ = this->create_service<as2_msgs::srv::SetOrigin>(
        this->generate_global_name("set_origin"),
        std::bind(&GpsTranslator::setOriginCb, this, std::placeholders::_1, std::placeholders::_2));
    get_origin_srv_ = this->create_service<as2_msgs::srv::GetOrigin>(
        this->generate_global_name("get_origin"), 
        std::bind(&GpsTranslator::getOriginCb, this, std::placeholders::_1, std::placeholders::_2));
}

GpsTranslator::GpsTranslator(double lat, double lon, double alt): as2::Node("test") {
    this->utils_.SetOrigin(lat, lon, alt);

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

    set_origin_srv_ = this->create_service<as2_msgs::srv::SetOrigin>(
        this->generate_global_name("set_origin"),
        std::bind(&GpsTranslator::setOriginCb, this, std::placeholders::_1, std::placeholders::_2));
    get_origin_srv_ = this->create_service<as2_msgs::srv::GetOrigin>(
        this->generate_global_name("get_origin"), 
        std::bind(&GpsTranslator::getOriginCb, this, std::placeholders::_1, std::placeholders::_2));
}

GpsTranslator::GpsTranslator(GpsUtils utils): as2::Node("gps_translator") { 
    double lat, lon, alt;
    utils.GetOrigin(lat, lon, alt);
    this->utils_.SetOrigin(lat, lon, alt);

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

    set_origin_srv_ = this->create_service<as2_msgs::srv::SetOrigin>(
        this->generate_global_name("set_origin"),
        std::bind(&GpsTranslator::setOriginCb, this, std::placeholders::_1, std::placeholders::_2));
    get_origin_srv_ = this->create_service<as2_msgs::srv::GetOrigin>(
        this->generate_global_name("get_origin"), 
        std::bind(&GpsTranslator::getOriginCb, this, std::placeholders::_1, std::placeholders::_2));
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

void GpsTranslator::setOriginCb(const std::shared_ptr<as2_msgs::srv::SetOrigin::Request> request, 
                                std::shared_ptr<as2_msgs::srv::SetOrigin::Response> response) {
    try {
        utils_.SetOrigin(request->origin.latitude, request->origin.longitude, request->origin.altitude);
        response->success = true;
    } catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Origin already set.");
        response->success = false;
    }
}

void GpsTranslator::getOriginCb(const std::shared_ptr<as2_msgs::srv::GetOrigin::Request> request, 
                                std::shared_ptr<as2_msgs::srv::GetOrigin::Response> response) {
    try {
        geographic_msgs::msg::GeoPoint geo_point;
        utils_.GetOrigin(geo_point.latitude, geo_point.longitude, geo_point.altitude);
        response->origin = geo_point;
        response->success = true;
    } catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Unable to get origin since it's not set.");
        response->success = false;
    }   
}
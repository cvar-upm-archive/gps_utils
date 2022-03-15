#include "GpsTranslator.hpp"

GpsTranslator::GpsTranslator::GpsTranslator(const rclcpp::NodeOptions & options): as2::Node("gps_translator", options) {
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

    geopath_to_path_srv_ = this->create_service<as2_msgs::srv::GeopathToPath>(
        this->generate_global_name("geopath_to_path"),
        std::bind(&GpsTranslator::geopathToPathCb, this, std::placeholders::_1, std::placeholders::_2));
    path_to_geopath_srv_ = this->create_service<as2_msgs::srv::PathToGeopath>(
        this->generate_global_name("path_to_geopath"),
        std::bind(&GpsTranslator::pathToGeopathCb, this, std::placeholders::_1, std::placeholders::_2));
}

GpsTranslator::GpsTranslator::GpsTranslator(double lat, double lon, double alt): as2::Node("gps_translator") {
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

    geopath_to_path_srv_ = this->create_service<as2_msgs::srv::GeopathToPath>(
        this->generate_global_name("geopath_to_path"),
        std::bind(&GpsTranslator::geopathToPathCb, this, std::placeholders::_1, std::placeholders::_2));
    path_to_geopath_srv_ = this->create_service<as2_msgs::srv::PathToGeopath>(
        this->generate_global_name("path_to_geopath"),
        std::bind(&GpsTranslator::pathToGeopathCb, this, std::placeholders::_1, std::placeholders::_2));
}

GpsTranslator::GpsTranslator::GpsTranslator(GpsUtils utils): as2::Node("gps_translator") { 
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

    geopath_to_path_srv_ = this->create_service<as2_msgs::srv::GeopathToPath>(
        this->generate_global_name("geopath_to_path"),
        std::bind(&GpsTranslator::geopathToPathCb, this, std::placeholders::_1, std::placeholders::_2));
    path_to_geopath_srv_ = this->create_service<as2_msgs::srv::PathToGeopath>(
        this->generate_global_name("path_to_geopath"),
        std::bind(&GpsTranslator::pathToGeopathCb, this, std::placeholders::_1, std::placeholders::_2));
}

void GpsTranslator::GpsTranslator::translateCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
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

void GpsTranslator::GpsTranslator::setOriginCb(const std::shared_ptr<as2_msgs::srv::SetOrigin::Request> request, 
                                std::shared_ptr<as2_msgs::srv::SetOrigin::Response> response) {
    try {
        utils_.SetOrigin(request->origin.latitude, request->origin.longitude, request->origin.altitude);
        response->success = true;
        std::string info = "New origin: [ " + std::to_string(request->origin.latitude) + ", " 
                                            + std::to_string(request->origin.longitude) + ", " 
                                            + std::to_string(request->origin.altitude) + " ]";
        RCLCPP_INFO(this->get_logger(), info);
    } catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Origin already set.");
        response->success = false;
    }
}

void GpsTranslator::GpsTranslator::getOriginCb(const std::shared_ptr<as2_msgs::srv::GetOrigin::Request> request, 
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

void GpsTranslator::GpsTranslator::geopathToPathCb(const std::shared_ptr<as2_msgs::srv::GeopathToPath::Request> request, 
                                std::shared_ptr<as2_msgs::srv::GeopathToPath::Response> response) {
    try {
        response->path.header = request->geo_path.header;
        for(const geographic_msgs::msg::GeoPoseStamped &gps : request->geo_path.poses) {
            geometry_msgs::msg::PoseStamped pose;
            utils_.LatLon2Local(gps, pose);
            response->path.poses.push_back(pose);
        }
        response->success = true;
    } catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Local transformation not available since origin is not set.");
        response->success = false;
    }
}

void GpsTranslator::GpsTranslator::pathToGeopathCb(const std::shared_ptr<as2_msgs::srv::PathToGeopath::Request> request, 
                                std::shared_ptr<as2_msgs::srv::PathToGeopath::Response> response) {
    try {
        response->geo_path.header = request->path.header;
        for(const geometry_msgs::msg::PoseStamped &pose : request->path.poses) {
            geographic_msgs::msg::GeoPoseStamped gps;
            utils_.Local2LatLon(pose, gps);
            response->geo_path.poses.push_back(gps);
        }
        response->success = true;
    } catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Local transformation not available since origin is not set.");
        response->success = false;
    }
}

// COMPONENT
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(GpsTranslator::GpsTranslator)
#include "GpsUtils.hpp"

void GpsUtils::LatLon2Local(double lat, double lon, double h, double &rX, double &rY, double &rZ) {
    if (!this->isOriginSet) {
        throw OriginNonSet();
    }
    this->Forward(lat, lon, h, rX, rY, rZ);
}

void GpsUtils::LatLon2Local(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ) {
    this->LatLon2Local(fix.latitude, fix.longitude, fix.altitude, rX, rY, rZ);
}

void GpsUtils::LatLon2Local(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps) {
    double x, y, z;
    this->LatLon2Local(lat, lon, h, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = this->localFrame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsUtils::LatLon2Local(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps){
    this->LatLon2Local(fix.latitude, fix.longitude, fix.altitude, ps);
}

void GpsUtils::Local2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH) {
    if (!this->isOriginSet) {
        throw OriginNonSet();
    }
    this->Reverse(x, y, z, rLat, rLon, rH);
}

void GpsUtils::Local2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH) {
    this->Local2LatLon(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, rLat, rLon, rH);
}

void GpsUtils::SetOrigin(double lat0, double lon0, double h0) {
    if (!this->isOriginSet) {
        throw OriginAlreadySet();
    }
    this->Reset(lat0, lon0, h0);
    this->isOriginSet = true;
}

void GpsUtils::SetOrigin(sensor_msgs::msg::NavSatFix fix) {
    return this->SetOrigin(fix.latitude, fix.longitude, fix.altitude);
}

void GpsUtils::GetOrigin(double &rLat, double &rLon, double &rH) {
    rLat = this->LatitudeOrigin();
    rLon = this->LongitudeOrigin();
    rH = this->HeightOrigin();
}

void GpsUtils::LatLon2Ecef(double lat, double lon, double h, double &rX, double &rY, double &rZ) {
    earth.Forward(lat, lon, h, rX, rY, rZ);
}

void GpsUtils::LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ) {
    GpsUtils::LatLon2Ecef(fix.latitude, fix.longitude, fix.altitude, rX, rY, rZ);
}

void GpsUtils::LatLon2Ecef(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps){    
    double x, y, z;
    GpsUtils::LatLon2Ecef(lat, lon, h, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = globalFrame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsUtils::LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps) {
    GpsUtils::LatLon2Ecef(fix.latitude, fix.longitude, fix.altitude, ps);
}

void GpsUtils::Ecef2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH) {
    earth.Reverse(x, y, z, rLat, rLon, rH);
}

void GpsUtils::Ecef2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH) {
    GpsUtils::Ecef2LatLon(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, rLat, rLon, rH);
}

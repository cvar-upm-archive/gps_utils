#include "GpsUtils.hpp"

void GpsUtils::LatLon2Local(double lat, double lon, double h, double &rX, double &rY, double &rZ) {
    this->Forward(lat, lon, h, rX, rY, rZ);
}

void GpsUtils::LatLon2Local(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ) {
    this->Forward(fix.latitude, fix.longitude, fix.altitude, rX, rY, rZ);
}

void GpsUtils::LatLon2Local(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps) {
    double x, y, z;
    this->Forward(lat, lon, h, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = this->frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsUtils::LatLon2Local(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps){
    double x, y, z;
    this->Forward(fix.latitude, fix.longitude, fix.altitude, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = this->frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsUtils::Local2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH) {
    this->Reverse(x, y, z, rLat, rLon, rH);
}

void GpsUtils::Local2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH) {
    this->Reverse(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, rLat, rLon, rH);
}

void GpsUtils::SetOrigin(double lat0, double lon0) {
    this->Reset(lat0, lon0);
}

void GpsUtils::SetOrigin(double lat0, double lon0, double h0) {
    this->Reset(lat0, lon0, h0);
}

void GpsUtils::SetOrigin(sensor_msgs::msg::NavSatFix fix) {
    this->Reset(fix.latitude, fix.longitude, fix.altitude);
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
    earth.Forward(fix.latitude, fix.longitude, fix.altitude, rX, rY, rZ);
}

void GpsUtils::LatLon2Ecef(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps){    
    double x, y, z;
    earth.Forward(lat, lon, h, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = this->frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsUtils::LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps) {
    double x, y, z;
    earth.Forward(fix.latitude, fix.longitude, fix.altitude, x, y, z);
    // ps.header.stamp = 0;
    ps.header.frame_id = this->frame;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = z;
}

void GpsUtils::Ecef2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH) {
    earth.Reverse(x, y, z, rLat, rLon, rH);
}

void GpsUtils::Ecef2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH) {
    earth.Reverse(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z, rLat, rLon, rH);
}

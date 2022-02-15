/**
 * @file GpsUtils.hpp
 * @author Pedro Arias Perez
 * @brief Geodesic to Cartesian conversions
 * @version 0.1
 * @date 2022-02-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

const static GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
const static std::string globalFrame = "earth";  // wgs84 --> ROS REP105 Name Convention

class GpsUtils: private GeographicLib::LocalCartesian
{
private:
    const std::string localFrame = "map";  // local world fixed --> ROS REP105 Name Convention
    bool isOriginSet = false;
public:
    GpsUtils(): GeographicLib::LocalCartesian(earth) {};
    GpsUtils(double lat0, double lon0, double h0 = 0): GeographicLib::LocalCartesian(lat0, lon0, h0, earth) { this->isOriginSet = true; };

    // Geodesic Lat Lon to Local Cartesian 
    void LatLon2Local(double lat, double lon, double h, double &rX, double &rY, double &rZ);
    void LatLon2Local(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ);
    void LatLon2Local(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps);
    void LatLon2Local(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps);
    void Local2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH);
    void Local2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH);

    // Origin 
    void SetOrigin(double lat0, double lon0, double h0 = 0);
    void SetOrigin(sensor_msgs::msg::NavSatFix fix);
    void GetOrigin(double &rLat, double &rLon, double &rH);

    // Geodesic Lat Lon to Earth-Centered-Earth-Fixed
    static void LatLon2Ecef(double lat, double lon, double h, double &rX, double &rY, double &rZ);
    static void LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ);
    static void LatLon2Ecef(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps);
    static void LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps);
    static void Ecef2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH);
    static void Ecef2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH);
};  // GpsUtils

class OriginNonSet : public std::runtime_error
{
public:
    OriginNonSet(): std::runtime_error("origin is not set") {}
};

class OriginAlreadySet : public std::runtime_error
{
public:
    OriginAlreadySet(): std::runtime_error("origin can only be set once") {}
};
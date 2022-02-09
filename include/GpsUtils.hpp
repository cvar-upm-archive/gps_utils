#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

const static GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();

class GpsUtils: private GeographicLib::LocalCartesian
{
private:
    std::string frame = "wgs84";
public:
    GpsUtils(): GeographicLib::LocalCartesian(earth) {};
    GpsUtils(double lat0, double lon0): GeographicLib::LocalCartesian(lat0, lon0, 0, earth) {};
    GpsUtils(double lat0, double lon0, double h0): GeographicLib::LocalCartesian(lat0, lon0, h0, earth) {};

    // Geodesic Lat Lon to Local Cartesian 
    void LatLon2Local(double lat, double lon, double h, double &rX, double &rY, double &rZ);
    void LatLon2Local(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ);
    void LatLon2Local(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps);
    void LatLon2Local(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps);
    void Local2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH);
    void Local2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH);

    // Origin 
    void SetOrigin(double lat0, double lon0);
    void SetOrigin(double lat0, double lon0, double h0);
    void SetOrigin(sensor_msgs::msg::NavSatFix fix);
    void GetOrigin(double &rLat, double &rLon, double &rH);

    // Geodesic Lat Lon to Earth-Centered-Earth-Fixed
    void LatLon2Ecef(double lat, double lon, double h, double &rX, double &rY, double &rZ);
    void LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, double &rX, double &rY, double &rZ);
    void LatLon2Ecef(double lat, double lon, double h, geometry_msgs::msg::PoseStamped &ps);
    void LatLon2Ecef(sensor_msgs::msg::NavSatFix fix, geometry_msgs::msg::PoseStamped &ps);
    void Ecef2LatLon(double x, double y, double z, double &rLat, double &rLon, double &rH);
    void Ecef2LatLon(geometry_msgs::msg::PoseStamped ps, double &rLat, double &rLon, double &rH);
};  // GpsUtils
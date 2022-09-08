
#include <iostream>

#include "GpsUtils.hpp"

int main() {
    std::cout.precision(17);

    double lat0, lon0, alt0;
    lat0 = 28.14376;
    lon0 = -16.50235;
    alt0 = 0;
    GpsUtils utils(lat0, lon0, alt0);

    double lat, lon, alt;
    double x, y, z;
    utils.LatLon2Local(lat0, lon0, 0, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    utils.LatLon2Local(lat0, lon0, 20, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    utils.LatLon2Local(lat0, lon0, 50, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    utils.LatLon2Local(lat0, lon0, 100, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    std::cout << "----------" << std::endl;
    double lat1, lon1;
    lat1 = 28.143;
    lon1 = -16.502;
    utils.LatLon2Local(lat1, lon1, 0, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    utils.LatLon2Local(lat1, lon1, 50, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    utils.LatLon2Local(lat1, lon1, 100, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;
    utils.Local2LatLon(x, y, z, lat, lon, alt);
    std::cout << lat << " " << lon << " " << alt << std::endl;
    std::cout << std::endl;

    std::cout << "----------" << std::endl;
    geographic_msgs::msg::GeoPoseStamped gps;
    geometry_msgs::msg::PoseStamped ps;
    gps.pose.position.latitude = lat1;
    gps.pose.position.longitude = lon1;
    gps.pose.position.altitude = 0;

    utils.LatLon2Local(gps, ps);
    std::cout << "TEST: " << ps.pose.position.x << " " << ps.pose.position.y << " " << ps.pose.position.z << std::endl;

    return 0;
}
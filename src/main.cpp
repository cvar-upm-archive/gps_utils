/**
 * @file main.cpp
 * @author Pedro Arias Perez
 * @brief Testing GPS Utils
 * @version 0.1
 * @date 2022-02-03
 * 
 * @copyright Copyright (c) 2022
 */

#include <iostream>
#include "GpsUtils.hpp"

int main() {
    double lat, lon, h, x, y, z;
    lat = 40.43986720817431; lon = -3.6893830035942115; h = 0.0; // ETSII
    double lat1 = 40.43805993493903, lon1 = -3.6907017162961346, h1 = 0.0; // Gregorio Maranhon

    GpsUtils utils(lat, lon, h);
    utils.LatLon2Local(lat1, lon1, h1, x, y, z);
    std::cout << std::round(x) << " " << std::round(y) << " " << std::round(z) << std::endl;

    return 0;
}
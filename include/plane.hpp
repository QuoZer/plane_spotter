#pragma once
#include <string>
#include <vector>

#include <utils.hpp>
#include <parser.hpp>

#include <nlohmann/json.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <opencv2/opencv.hpp>

namespace plane_spotter
{
using json = nlohmann::json;
    

    
struct PlanePos {
    WGS_Pos wgs;
    ECEF_Pos ecef;
    ECEF_Pos pos_in_camera; 
    VecRot R;

    double gs;
    double timestamp;

    PlanePos(double ts, double lat, double lon, double alt);
    PlanePos(double ts, const WGS_Pos& pos);
    PlanePos(double ts, const AircraftData& data);
};


struct Plane {
    
    std::string flight_no;
    std::string hex;
    FixedSizeBuffer<PlanePos> pos_history;  // remember 10 last locations of a plane
    double last_ts; 

    const double K2MS =  0.514444; // knots to m/s
    
    Plane(): pos_history(10) {}; // TODO: this is generally not good 
    Plane(std::string flight): flight_no(flight), pos_history(10) {};

    void add_pos(PlanePos plane_pos);

    // get plane's moving direction from last two positions 
    cv::Vec3d dead_reckoning();
};

} // namespace plane_spotter
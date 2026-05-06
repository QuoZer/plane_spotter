#pragma once
#include <string>
#include <vector>

#include <utils.hpp>

#include <nlohmann/json.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <opencv2/opencv.hpp>

namespace plane_spotter
{
using json = nlohmann::json;
using Geo = GeographicLib::Geocentric;
    

struct WGS_Pos {
    double lat;
    double lon;
    double alt;
};

struct ECEF_Pos: public cv::Point3d {
    // cartesian point in ecef frame, support OpenCV's linal
    using cv::Point3d::Point3d;

    ECEF_Pos(const WGS_Pos& wgs);
};

struct VecRot {
    // Euler angle rotation 
    double roll;
    double pitch;
    double yaw;
};
    
struct PlanePos {
    WGS_Pos wgs;
    ECEF_Pos ecef;

    double head;
    double pitch;
    double timestamp;

    PlanePos(double lat, double lon, double alt);
    PlanePos(const WGS_Pos& pos);
};


struct Plane {
    
    std::string flight_no;
    std::string hex;
    FixedSizeBuffer<PlanePos> pos_history;  // remember 10 last locations of a plane
    
    Plane(): pos_history(10) {}; // TODO: this is generally not good 
    Plane(std::string flight): flight_no(flight), pos_history(10) {};

    void add_pos(PlanePos plane_pos);
    void add_pos(json aircraft_data);
};

} // namespace plane_spotter
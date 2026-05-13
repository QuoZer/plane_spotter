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
    // cartesian point in ecef frame, support OpenCV's lin-al
    using cv::Point3d::Point3d;

    ECEF_Pos(const WGS_Pos& wgs);

    ECEF_Pos& operator=(const cv::Point3d& p);
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
    ECEF_Pos pos_in_camera; 
    VecRot R;

    double gs;
    double timestamp;

    PlanePos(double ts, double lat, double lon, double alt);
    PlanePos(double ts, const WGS_Pos& pos);
    // PlanePos(const AircraftData& data);
};


struct Plane {
    
    std::string flight_no;
    std::string hex;
    FixedSizeBuffer<PlanePos> pos_history;  // remember 10 last locations of a plane
    double last_ts; 

    const double K2MS =  0.514444; // knots to m/s
    const double expected_delay = 1; // s, how stale is plane's position 
    
    Plane(): pos_history(10) {}; // TODO: this is generally not good 
    Plane(std::string flight): flight_no(flight), pos_history(10) {};

    void add_pos(PlanePos plane_pos);
    void add_pos(double ts, json aircraft_data);

    cv::Point3d dead_reckoning();
};

} // namespace plane_spotter
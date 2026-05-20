#pragma once
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


namespace gl = GeographicLib;
namespace plane_spotter {
using json = nlohmann::json;
using Geo = GeographicLib::Geocentric;

json read_json(std::string path);

// position in wgs coordinates 
struct WGS_Pos {
    double lat;
    double lon;
    double alt;
};

// position in ecef coordinates 
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

struct AircraftData {
    // Describes each entry in the message from dump1090

    WGS_Pos pos;
    double heading;
    double speed;

    std::string hex;
    std::string flight;

    AircraftData(const json& js_struct);
};

struct AircraftMsg {
    // Describes the messages coming from dump1090 

    double timestamp;
    std::vector<AircraftData> flights; 

    AircraftMsg(const json& js_struct);  
};


}
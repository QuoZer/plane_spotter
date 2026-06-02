#pragma once
#include <iostream>
#include <fstream>
#include <chrono>

#include <nlohmann/json.hpp>
#include <cpr/cpr.h>
#include <opencv2/opencv.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


namespace gl = GeographicLib;
namespace plane_spotter {
using json = nlohmann::json;
using Geo = GeographicLib::Geocentric;


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
    // Euler angle vector
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

    AircraftData(const std::string& flight_hex, const std::string& flight_no, double heading_, double groundspeed,
                 double lat, double lon, double alt): pos{lat, lon, alt}, heading(heading_), speed(groundspeed),
                 hex(flight_hex), flight(flight_no) {};
    AircraftData(const json& js_struct);
};

struct AircraftMsg {
    // Describes the messages coming from dump1090 
    double timestamp;
    std::vector<AircraftData> flights; 

    AircraftMsg() =default; 
    AircraftMsg(double ts, std::vector<AircraftData>& planes): timestamp(ts), flights(planes) {};  // or steal with &&? 
    AircraftMsg(const json& js_struct);  

    static AircraftMsg fromDump1090(const json& js_struct); 
    static AircraftMsg fromAPI(const json& js_struct); 
};

// reads a json file on path and returns a nlohmann::json object
json read_json(std::string path);

// get planes from FlightAware api within the coordinates (top left, bottom right)
AircraftMsg get_api_planes(std::string api_key, double min_lat, double min_lon, double max_lat, double max_lon);

}
#pragma once
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

#include <plane.hpp>


namespace plane_spotter {
using json = nlohmann::json;

json read_json(std::string path);

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
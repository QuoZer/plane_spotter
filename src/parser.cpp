#include <parser.hpp>


namespace plane_spotter
{
    json read_json(std::string path) {
        std::ifstream f(path);
        if (!f.is_open()) throw std::exception(); 
        json data = json::parse(f);

        // if (data.is_discarded()) throw json::parse_error('a');
        return data;
    }

    AircraftMsg::AircraftMsg(const json& js_struct) {
        try
        {
            timestamp = js_struct["now"];
            
            for (auto& flight: js_struct["aircraft"]) {
                flights.emplace_back(flight);
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in AircraftMsg construction: " << e.what() << '\n';
        }
    }

    AircraftData::AircraftData(const json& js_struct) {
        try
        {
            hex = js_struct["hex"];
            flight = js_struct["flight"];

            heading = js_struct["mag_heading"];
            speed = js_struct["gs"];

            pos = WGS_Pos{js_struct["lat"], js_struct["lon"], js_struct["alt_geom"]};
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in AircraftData construction: " << e.what() << '\n';
        }
        
    }

} // namespace plane_spotter

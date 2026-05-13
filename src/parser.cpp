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
                // skip partial readings 
                if (! (flight.contains("hex") && flight.contains("flight") && 
                        flight.contains("mag_heading") && flight.contains("gs") && 
                        flight.contains("lat") && flight.contains("lon") && flight.contains("alt_baro")) ) continue;

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
            hex = js_struct.at("hex");
            flight = js_struct.at("flight");

            heading = js_struct.at("mag_heading");
            speed = js_struct.at("gs");  // knots?

            double alt_m = static_cast<double>(js_struct.at("alt_baro"))*0.303;

            pos = WGS_Pos{js_struct.at("lat"), js_struct.at("lon"), alt_m};
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in AircraftData construction: " << e.what() << '\n';
        }
        
    }

} // namespace plane_spotter

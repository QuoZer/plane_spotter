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

    AircraftMsg get_api_planes(std::string api_key, double min_lat, double min_lon, double max_lat, double max_lon) {
        cpr::Response r = cpr::Get(
            cpr::Url{"https://aeroapi.flightaware.com/aeroapi/flights/search"},
            cpr::Header{{"x-apikey", api_key}},
            cpr::Parameters{{"query", "-latlong \"59.717185321041406  30.048275286082657 59.90572758595382 30.32474264337777\""}}  // example query param
        );
        
        if (r.status_code == 200) {
            auto json = nlohmann::json::parse(r.text);
            
            return AircraftMsg::fromAPI(json);
        }

        if (r.status_code == 400) {
            auto json = nlohmann::json::parse(r.text);
            std::cout << "failed to get API planes, reason: " << json["reason"] << std::endl;
        }

        return AircraftMsg(); 

    }


    ECEF_Pos::ECEF_Pos(const WGS_Pos& wgs) {
        const Geo& earth = Geo::WGS84();

        earth.Forward(wgs.lat, wgs.lon, wgs.alt, 
                      x, y, z);
    }

    ECEF_Pos& ECEF_Pos::operator=(const cv::Point3d& p) {
        cv::Point3d::operator=(p);
        return *this;
    }

    AircraftMsg::AircraftMsg(const json &js_struct)
    {
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

    AircraftMsg AircraftMsg::fromAPI(const json& js_struct) {
        std::vector<AircraftData> flights; 
        auto now = [](){ return std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() ).count() / 1000.0; }; 
        double ts = now();
        try{

            for (auto& flight: js_struct["flights"]) {
                // skip partial readings 
                if (! (flight.contains("ident") && flight.contains("ident_icao") && 
                       flight.contains("last_position")) ) continue;

                auto last_pos = flight["last_position"];
                // TODO: last pos 

                if (! (last_pos.contains("heading") && last_pos.contains("groundspeed") && 
                       last_pos.contains("latitude") && last_pos.contains("longitude") && 
                       last_pos.contains("altitude")) ) continue;

                flights.emplace_back(flight.at("ident"), flight.at("ident_icao"), last_pos.at("heading"), last_pos.at("groundspeed"),
                                     last_pos.at("latitude"), last_pos.at("longitude"), last_pos.at("altitude"));
            }

            return AircraftMsg(ts, flights);
        }
        catch(const std::exception& e) {
            std::cerr << "Error in AircraftMsg construction: " << e.what() << '\n';
        }
    }

    AircraftMsg AircraftMsg::fromDump1090(const json& js_struct) {
        std::vector<AircraftData> flights; 
        try
        {
            double ts = js_struct["now"];
            
            for (auto& flight: js_struct["aircraft"]) {
                // skip partial readings 
                if (! (flight.contains("hex") && flight.contains("flight") && 
                        flight.contains("mag_heading") && flight.contains("gs") && 
                        flight.contains("lat") && flight.contains("lon") && flight.contains("alt_baro")) ) continue;

                flights.emplace_back(flight);
            }

            return AircraftMsg(ts, flights);
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

            double ft2m = 0.3048;
            double alt_m = static_cast<double>(js_struct.at("alt_baro"))*ft2m;

            pos = WGS_Pos{js_struct.at("lat"), js_struct.at("lon"), alt_m};
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error in AircraftData construction: " << e.what() << '\n';
        }
        
    }

} // namespace plane_spotter

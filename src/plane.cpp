#include <plane.hpp>

namespace plane_spotter
{

    ECEF_Pos::ECEF_Pos(const WGS_Pos& wgs) {
        const Geo& earth = Geo::WGS84();

        earth.Forward(wgs.lat, wgs.lon, wgs.alt, 
                      x, y, z);
    }

    PlanePos::PlanePos(double lat, double lon, double alt): wgs{lat, lon, alt}, ecef(wgs) { }
    PlanePos::PlanePos(const WGS_Pos& pos): wgs{pos}, ecef(wgs) { }

    void Plane::add_pos(PlanePos plane_pos) {
        pos_history.push(plane_pos);
    }

    void Plane::add_pos(json aircraft_data) {
        PlanePos new_pos{aircraft_data["lat"], aircraft_data["lon"], aircraft_data["alt"]};
        add_pos(new_pos);
    }


}
#include <plane.hpp>

namespace plane_spotter
{

    ECEF_Pos::ECEF_Pos(const WGS_Pos& wgs) {
        const Geo& earth = Geo::WGS84();

        earth.Forward(wgs.lat, wgs.lon, wgs.alt, 
                      x, y, z);
    }

    ECEF_Pos& ECEF_Pos::operator=(const cv::Point3d& p) {
        cv::Point3d::operator=(p);
        return *this;
    }

    PlanePos::PlanePos(double ts, double lat, double lon, double alt): wgs{lat, lon, alt}, ecef(wgs), timestamp(ts) { }
    PlanePos::PlanePos(double ts, const WGS_Pos& pos): wgs{pos}, ecef(wgs), timestamp(ts) { }

    void Plane::add_pos(PlanePos plane_pos) {
        last_ts = plane_pos.timestamp;
        pos_history.push(plane_pos);
    }

    void Plane::add_pos(double ts, json aircraft_data) {
        PlanePos new_pos{ts, aircraft_data["lat"], aircraft_data["lon"], aircraft_data["alt"]};
        add_pos(new_pos);
    }

    cv::Point3d Plane::dead_reckoning() {
        PlanePos& last_pos = pos_history.back();
        if (pos_history.size() < 2) return last_pos.pos_in_camera; 
        PlanePos& prev_pos = pos_history[pos_history.size()-2];

        double dt = last_pos.timestamp - prev_pos.timestamp;
        if (dt > 10) return last_pos.pos_in_camera; 

        // positional difference 
        cv::Vec3d d_pos = last_pos.pos_in_camera - prev_pos.pos_in_camera; 
        cv::Vec3d direction = d_pos / cv::norm(d_pos);

        cv::Point3d predicted_pos = direction * last_pos.gs * K2MS * expected_delay;
        return predicted_pos;
    }


}
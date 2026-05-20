#include <plane.hpp>

namespace plane_spotter
{


    PlanePos::PlanePos(double ts, double lat, double lon, double alt): wgs{lat, lon, alt}, ecef(wgs), timestamp(ts) { }
    PlanePos::PlanePos(double ts, const WGS_Pos& pos): wgs{pos}, ecef(wgs), timestamp(ts) { }
    PlanePos::PlanePos(double ts, const AircraftData& data): wgs{data.pos.lat, data.pos.lon, data.pos.alt}, ecef(wgs), timestamp(ts) {
        gs = data.speed;
        R.yaw = data.heading;
    }

    void Plane::add_pos(PlanePos plane_pos) {
        last_ts = plane_pos.timestamp;
        pos_history.push(plane_pos);
    }

    cv::Vec3d Plane::dead_reckoning() {
        PlanePos& last_pos = pos_history.back();
        if (pos_history.size() < 2) return last_pos.pos_in_camera; 

        PlanePos& prev_pos = pos_history[pos_history.size()-2];

        double dt = last_pos.timestamp - prev_pos.timestamp;
        if (dt > 10) return last_pos.pos_in_camera; 

        // positional difference 
        cv::Vec3d d_pos = last_pos.pos_in_camera - prev_pos.pos_in_camera; 
        double pos_norm = cv::norm(d_pos);
        if (pos_norm < 0.05) return last_pos.pos_in_camera; 
        
        cv::Vec3d direction = d_pos / pos_norm;

        return direction * last_pos.gs * K2MS;
    }


}
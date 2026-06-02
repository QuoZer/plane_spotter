#pragma once
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#define gl GeographicLib

#include <plane.hpp>
#include <camera.hpp>
#include <parser.hpp>
#include <filters.hpp>

#include <chrono>
#include <optional>

namespace plane_spotter
{
    using json = nlohmann::json;

    struct AircraftView {
        Plane aircraft;
        std::vector<cv::Point2i> track; // pixel trail 
    };


    class Spotter {
    private:
        json camera_params_;
        json app_params_;

        ECEF_Pos cam_pos;
        VecRot cam_rot;
        gl::LocalCartesian camera_pos_projector; 

        std::unordered_map<std::string, AircraftView> history; // hex: PlaneObject

        Camera camera; 
        EKF<6> *filter;

        cv::Mat canvas; 

        const double PI = 3.141592653589;
        const int marker_decay = 20; // s, until we consider the plane's position outdated 

        bool filter_init = false;

    public:
        Spotter(json app_params, json camera_params);

        int predict(AircraftMsg new_msg);

        void display(cv::Mat& img);

        cv::Point3d transform_coords(WGS_Pos world_coords);

        Matrix<double, 3, 3> getRotationMatrix(double roll, double pitch, double yaw);

    private:

        void fuse(std::optional<cv::Point3d> adsb_pos, std::optional<cv::Point2d> cv_pos, double dt);

        void draw(cv::Mat& canvas, std::string name, cv::Point2d target, cv::Point2d prediction);

        void upd_history(const AircraftMsg& new_msg);

    };

    
    
} // namespace spotter
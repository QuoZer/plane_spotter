#include <spotter.hpp>

namespace plane_spotter
{

    Spotter::Spotter(json app_params, json camera_params):  app_params_(app_params) {
        cv::Size img_size(camera_params["width"], camera_params["height"]);

        camera.setIntrinsics(img_size, camera_params["fx"], camera_params["fy"]);

        cam_pos = ECEF_Pos(WGS_Pos{app_params["lat"], app_params["lon"], app_params["alt"]});
        cam_rot.yaw = static_cast<double>(app_params["heading"]) * PI / 180;
        cam_rot.roll = static_cast<double>(app_params["roll"]) * PI / 180;
        cam_rot.pitch = static_cast<double>(app_params["pitch"]) * PI / 180;

        camera_pos_projector = gl::LocalCartesian(app_params["lat"], app_params["lon"], app_params["alt"]);

        canvas = cv::Mat(img_size, CV_8UC3);
    }

    void Spotter::upd_history(const AircraftMsg& new_msg) {

        for (auto& aircraft: new_msg.flights) {

            if (history.find(aircraft.hex) == history.end()) {
                Plane new_plane(aircraft.hex);
                new_plane.add_pos(aircraft.pos);
                new_plane.flight_no = aircraft.flight;

                history.insert( {aircraft.hex, new_plane} ); 
            }
            else {
                history[aircraft.hex].add_pos(aircraft.pos); 
            }
        }
    }

    cv::Point3d Spotter::transform_coords(WGS_Pos world_coords) {
        // transforms any wgs position into local ENU coordinates relative to the camera 
        cv::Point3d local_point;
        camera_pos_projector.Forward(world_coords.lat, world_coords.lon, world_coords.alt,
                                     local_point.x, local_point.y, local_point.z);

        // ENU to opencv
        cv::Point3d camera_point{local_point.x, -local_point.z, local_point.y};

        return camera_point; 
    }

    bool Spotter::check_pose(cv::Point3d local_coords) {
        // TODO: implement
        
        return true;
    }

    int Spotter::predict(AircraftMsg new_msg) {
        // 1. upd history in relevant planes 
        upd_history(new_msg);

        // for each:
        for (auto& [k, v]: history) {
            // 2. find plane's location in the camera frame and filter 
            cv::Point3d local_pos = transform_coords(v.pos_history.back().wgs); 
            cv::Point3d cam_pos = camera.rotatePoint(local_pos, cam_rot.roll, -cam_rot.yaw, cam_rot.pitch);
            
            // check that local_pose falls inside frustrum 
            if (!check_pose(local_pos)) continue;
            
            // 3. project to camera 
            cv::Point2d pixel = camera.projectWorldToPixel(cam_pos);
            
            std::cout << k << std::endl;
            std::cout << "Located at: (cart): " << cam_pos.x << " | " << cam_pos.y << " | " << cam_pos.z << std::endl; 
            std::cout << "\t(pix): " << pixel.x << " | " << pixel.y << std::endl; 

            draw(canvas, k, pixel);
        }

        return 0;
    }

    void Spotter::draw(cv::Mat& canvas, std::string name, cv::Point2d target) {
        cv::circle(canvas, target, 20, cv::Scalar(0, 0, 255), 5);
        cv::putText(canvas, name, target+cv::Point2d(-10, -30), 0, 2, cv::Scalar(0, 0, 255), 3);
        // cv::addText(canvas, name, target+cv::Point2d(-10, 10), cv::fontQt("Times"));
    }

    void Spotter::display(cv::Mat& img) {
        cv::Mat grayOverlay, mask;
        cv::cvtColor(canvas, grayOverlay, cv::COLOR_BGR2GRAY);
        // Threshold to create a binary mask (pixels > 0 become 255)
        cv::threshold(grayOverlay, mask, 0, 255, cv::THRESH_BINARY);
        

        // Overlay non-zero pixels onto the base image
        canvas.copyTo(img, mask);
        
        cv::Mat out; 
        cv::resize(img, out, cv::Size(1600, 1200));
        cv::imshow("Result", out);
        cv::waitKey();
    }

    
} // namespace plane_spotter


#include <spotter.hpp>

namespace plane_spotter
{

    Spotter::Spotter(json app_params, json camera_params):  app_params_(app_params) {
        cv::Size img_size(camera_params["width"], camera_params["height"]);

        cam_pos = ECEF_Pos(WGS_Pos{app_params["lat"], app_params["lon"], app_params["alt"]});
        cam_rot.yaw = static_cast<double>(app_params["heading"]) * PI / 180;
        cam_rot.roll = static_cast<double>(app_params["roll"]) * PI / 180;
        cam_rot.pitch = static_cast<double>(app_params["pitch"]) * PI / 180;
        
        camera.setIntrinsics(img_size, camera_params["fx"], camera_params["fy"]);
        camera.setExtrinsics(cam_pos, cam_rot);
        camera_pos_projector = gl::LocalCartesian(app_params["lat"], app_params["lon"], app_params["alt"]);

        canvas = cv::Mat(img_size, CV_8UC3);
    }

    void Spotter::upd_history(const AircraftMsg& new_msg) {

        for (auto& aircraft: new_msg.flights) {
            // create the pos object and fill in-camera coords (TODO: move inside PlanePos?)
            PlanePos new_pos(new_msg.timestamp, aircraft.pos);
            new_pos.pos_in_camera = transform_coords(new_pos.wgs); 

            if (history.find(aircraft.hex) == history.end()) {

                Plane new_plane(aircraft.hex);
                new_plane.flight_no = aircraft.flight;
                new_plane.add_pos(new_pos);

                history.insert( {aircraft.hex, new_plane} ); 
            }
            else {
                history[aircraft.hex].add_pos(new_pos); 
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

    int Spotter::predict(AircraftMsg new_msg) {
        // 1. upd history in relevant planes and transform new positions to the camera's frame 
        upd_history(new_msg);

        // for each:
        for (auto& [k, v]: history) {
            cv::Point3d local_pos = v.pos_history.back().pos_in_camera; 
            cv::Point3d predicted_pos = v.dead_reckoning();        
            
            // 3. project to camera 
            cv::Point2d pixel = camera.projectWorldToPixel(local_pos);
            cv::Point2d pred_pixel = camera.projectWorldToPixel(predicted_pos);

            // 4. local search ?
            // pixel = enhance(pixel);
            
            std::cout << k << std::endl;
            std::cout << "Located at: (cart): " << local_pos.x << " | " << local_pos.y << " | " << local_pos.z << std::endl; 
            std::cout << "\t(pix): " << pixel.x << " | " << pixel.y << std::endl; 

            draw(canvas, k, pixel, pred_pixel);
        }

        // check for stale flights, cleanup 

        return 0;
    }

    void Spotter::draw(cv::Mat& canvas, std::string name, cv::Point2d target, cv::Point2d prediction) {
        // cv::line(canvas)
        cv::circle(canvas, target, 20, cv::Scalar(0, 0, 255), 5);
        cv::circle(canvas, prediction, 15, cv::Scalar(0, 0, 255), 3);
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
        cv::waitKey(300);
    }

    
} // namespace plane_spotter


#include <spotter.hpp>

namespace plane_spotter
{

    Spotter::Spotter(const json& app_params, const json& camera_params):  app_params_(app_params) {
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
            PlanePos new_pos(new_msg.timestamp, aircraft);
            new_pos.pos_in_camera = transform_coords(new_pos.wgs); 

            if (history.find(aircraft.hex) == history.end()) {

                Plane new_plane(aircraft.hex);
                new_plane.flight_no = aircraft.flight;
                new_plane.add_pos(new_pos);

                history.insert( {aircraft.hex, AircraftView{new_plane} } ); 
            }
            else {
                history[aircraft.hex].aircraft.add_pos(new_pos); 
            }
        }
    }

    cv::Point3d Spotter::transform_coords(WGS_Pos world_coords) {
        // transforms any wgs position into local ENU coordinates relative to the camera 
        cv::Point3d local_point;
        camera_pos_projector.Forward(world_coords.lat, world_coords.lon, world_coords.alt,
                                     local_point.x, local_point.y, local_point.z);

        // ENU to opencv
        cv::Point3d camera_point{local_point.x, local_point.y, local_point.z};

        return camera_point; 
    }

    Matrix<double, 3, 3> Spotter::getRotationMatrix(double roll, double pitch, double yaw)
    {
        // TODO: use eigen rotations instead? 
        Matrix<double, 3, 3> rotX{{1.0,        0.0,       0.0  },
                                  {0.0,  cos(pitch), sin(pitch)},
                                  {0.0, -sin(pitch), cos(pitch)}
                                };

        Matrix<double, 3, 3> rotY{ {cos(roll), 0.0, -sin(roll)},
                                   {0.0,      1.0,     0.0    },
                                   {sin(roll), 0.0, cos(roll) }
                                };

        Matrix<double, 3, 3> rotZ{  {cos(yaw), -sin(yaw),  0.0},
                                    {sin(yaw), cos(yaw),   0.0},
                                    {0.0,      0.0,        1.0}
                                };

        Matrix<double, 3, 3> R = rotZ * rotY * rotX;
        
        return R;         
    }

        
    void Spotter::fuse(std::optional<cv::Point3d> adsb_pos, std::optional<cv::Point2d> cv_pos, double dt) {
        // TODO: move constant matricies somewhere 
        // new_pos = pos + vel*dt
        Matrix<double, 6, 6> F = Matrix<double, 6, 6>::Identity();
        F.block<3,3>(3, 0) = Matrix<double, 3, 3>::Identity() * dt; 
        
        Matrix<double, 3, 3> Qads = Matrix<double, 3, 3>::Identity();
        Matrix<double, 3, 3> Rads = Matrix<double, 3, 3>::Identity();
        
        // ADS-B measurements 
        Matrix<double, 3, 6> Hads = Matrix<double, 3, 6>::Zero();
        Hads.block<3,3>(0, 0) = Matrix<double, 3, 3>::Identity(); 
        ADSBModel<3, 6> adsb_model(Hads, Qads, Rads);

        // CV measurements
        Matrix<double, 3, 3> R = getRotationMatrix(camera.R.roll, camera.R.pitch, camera.R.yaw); 
        Matrix<double, 3, 6> Hcv = R * Hads; // Hads is a selection matrix already

        Matrix<double, 2, 2> Qcv = Matrix<double, 2, 2>::Identity();
        Matrix<double, 2, 2> Rcv = Matrix<double, 2, 2>::Identity();
        CVModel<2, 6> cv_model(&camera, Hcv, Qcv, Rcv);
        
        if (filter_init)  filter->predict(); 

        if (adsb_pos && !filter_init) {
            Vector<double, 6> x; // unpack adsb_pos + vel 
            Matrix<double, 6, 6> P = Matrix<double, 6, 6>::Identity();
            Matrix<double, 6, 6> Q = Matrix<double, 6, 6>::Identity();
            filter = new EKF<6> (F, Q, x, P); 
            filter_init = true;
        }
        else if (adsb_pos) {
            Eigen::Vector3d eigen_adsb_pos; 
            eigen_adsb_pos[0] = adsb_pos->x; eigen_adsb_pos[1] = adsb_pos->y; eigen_adsb_pos[2] = adsb_pos->z;
            filter->update(adsb_model, eigen_adsb_pos);
        }
        else if (cv_pos) {
            Eigen::Vector2d eigen_cv_pos; 
            eigen_cv_pos[0] = cv_pos->x; eigen_cv_pos[1] = cv_pos->y; 
            filter->update(cv_model, eigen_cv_pos);
        }

    }

    int Spotter::predict(AircraftMsg new_msg) {
        // 1. upd history in relevant planes and transform new positions to the camera's frame 
        upd_history(new_msg);

        auto now = [](){ return std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() ).count() / 1000.0; }; 

        // for each:
        for (auto& [hex, view]: history) {
            if (new_msg.timestamp - view.aircraft.last_ts > marker_decay) continue;
            // 2. predict actual pos 
            PlanePos& last_pos = view.aircraft.pos_history.back(); 
            cv::Vec3d lead_vec = view.aircraft.dead_reckoning();     
            
            cv::Point3d predicted_pos = last_pos.pos_in_camera + cv::Point3d(lead_vec * (now() - last_pos.timestamp));
            
            // 3. project to camera 
            cv::Point2d pixel = camera.projectWorldToPixel(last_pos.pos_in_camera);
            cv::Point2d pred_pixel = camera.projectWorldToPixel(predicted_pos);

            view.track.push_back(pixel);

            // 4. local search ?
            // pixel = enhance(pixel);
            
            std::cout << hex << std::endl;
            std::cout << "Located at: (cart): " << last_pos.pos_in_camera.x << " | " << last_pos.pos_in_camera.y << " | " << last_pos.pos_in_camera.z << std::endl; 
            std::cout << "\t(pix): " << pixel.x << " | " << pixel.y << std::endl; 

            draw(canvas, hex, pixel, pred_pixel);
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


#include <parser.hpp>
#include <spotter.hpp>

#include <opencv2/opencv.hpp>
#include <string>

using namespace plane_spotter;

int main(int argc, char* argv[]) {
    // args parser 
    std::string app_config = argv[1];
    std::cerr << "INPUT DIR: " << app_config << std::endl; 
    json app_params = read_json(app_config);

    std::string camera_config = app_params["camera_params_path"];
    std::string test_coords = app_params["test_coords"];
    std::string img_path = app_params["test_image"];

    // parse camera params
    json camera_params = read_json(camera_config);
    // parse input coords (=>fetch from api)
    json js_input = read_json(test_coords);
    AircraftMsg input(js_input);

    // read image (=> stream from camera)
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cout << "Error: Could not read the image." << std::endl;
        // return -1;
        img = cv::Mat3d(cv::Size(4032, 3024), 0);
    }

    // compute pixel positions 
    Spotter spotter(app_params, camera_params);
    spotter.predict(input);

    // display
    spotter.display(img);

    return 0;
}
#include <parser.hpp>
#include <spotter.hpp>

#include <opencv2/opencv.hpp>
#include <string>
#include <csignal>
#include <atomic>
#include <unistd.h>


using namespace plane_spotter;

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCaught Ctrl+C! Cleaning up..." << std::endl;
        keep_running = false; // Set flag to exit loop gracefully
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    // args parser 
    std::string app_config = argv[1];
    std::cerr << "INPUT DIR: " << app_config << std::endl; 
    json app_params = read_json(app_config);

    std::string camera_config = app_params["camera_params_path"];
    std::string test_coords = app_params["aircraft_json"];
    std::string img_path = app_params["test_image"];

    // parse camera params
    json camera_params = read_json(camera_config);

    Spotter spotter(app_params, camera_params);

    while (keep_running) {
        // parse input coords (=>fetch from api)
        // json js_input = read_json(test_coords);
        // AircraftMsg input(js_input);

        AircraftMsg input = get_api_planes(app_params["api_key"], 59.717185321041406, 30.048275286082657, 
                                                                  59.90572758595382,  30.32474264337777);

        std::cout << "tracking: " << input.flights.size() << " aircraft." << std::endl;
        
        // read image (TODO: stream from camera)
        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cout << "Error: Could not read the image." << std::endl;
            // return -1;
            img = cv::Mat3d(cv::Size(4032, 3024), 0);
        }
        
        // compute pixel positions 
        spotter.predict(input);
        // display
        spotter.display(img);
        // sleep(5);
        std::cout << "LOOP DONE" << std::endl;
    }

    cv::destroyAllWindows();

    return 0;
}

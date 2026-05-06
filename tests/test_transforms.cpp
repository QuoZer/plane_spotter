#include <gtest/gtest.h>

// #include "camera.hpp"
// #include "plane.hpp"
#include <spotter.hpp>

namespace plane_spotter {

double TEST_LAT = 59.9606739;       // spb
double TEST_LON = 30.1586551;
double TEST_ALT = 10;


// Fixture for tests that share a common camera position.
// SetUp() runs before each TEST_F in this suite.
class TransformTest : public ::testing::Test {
protected:
    Spotter* spotter; 

    void SetUp() override { 
        json app_params = { //59.9606739,30.1586551 TEST
                        {"lat", TEST_LAT},
                        {"lon", TEST_LON},
                        {"alt", TEST_ALT},
                        {"heading", 0}
                    };
        json camera_params = {
                        {"width", 4032},
                        {"height", 3024},
                        {"fx", 4400},
                        {"fy", 4400}
                    };

        spotter = new Spotter(app_params, camera_params);
    }

    void TearDown() override {
        delete spotter; 
    }

};

//  a point directly north at distance d, same altitude as camera,
// should produce ENU coords (0, d, 0). Use this as your first test.
TEST_F(TransformTest, PointDueNorthHasZeroEasting) {

    WGS_Pos north_point{TEST_LAT + 0.0001, TEST_LON, TEST_ALT}; // ~11m
    
    cv::Point3d local = spotter->transform_coords(north_point); 

    // std::cout << local.x << " | " << local.y << " | " << local.z << std::endl;

    ASSERT_NEAR(local.x, 0.0, 1e-05);
    ASSERT_NEAR(local.z, 0.0, 1e-05);
    ASSERT_GT(local.y, 10);
}

// a point directly east should produce ENU coords (d, 0, 0).
TEST_F(TransformTest, PointDueEastHasZeroNorthing) {

    WGS_Pos east_point{TEST_LAT, TEST_LON + 0.0001, TEST_ALT}; // ~5m
    
    cv::Point3d local = spotter->transform_coords(east_point); 

    // std::cout << local.x << " | " << local.y << " | " << local.z << std::endl;

    ASSERT_NEAR(local.y, 0.0, 1e-05);
    ASSERT_NEAR(local.z, 0.0, 1e-05);
    ASSERT_GT(local.x, 5);
}

//  a plane due north should land
// at the image centre column (pixel.x == width/2).
TEST_F(TransformTest, PlaneAheadProjectsToImageCentre) {
    GTEST_SKIP();
    // WGS_Pos north_point{TEST_LAT + 0.0001, TEST_LON, TEST_ALT};

    // cv::Point3d local = spotter->transform_coords(north_point); 
    
    // cv::Point2d pixel = spotter->camera.projectWorldToPixel(local);
}

TEST_F(TransformTest, CameraProjectionCenter) {
    GTEST_SKIP();
}

} // namespace plane_spotter

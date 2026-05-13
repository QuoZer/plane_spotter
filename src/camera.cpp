#include "camera.hpp"

namespace plane_spotter
{

Camera::Camera()
{
    
}

void Camera::toCenter(cv::Point& cornerPixel, cv::Size imagesize)
{
	cornerPixel.x = cornerPixel.x - imagesize.width / 2;
	cornerPixel.y = -cornerPixel.y + imagesize.height / 2;
}

void Camera::toCorner(cv::Point& centerPixel, cv::Size imagesize)
{
	centerPixel.x = centerPixel.x + imagesize.width / 2;
	centerPixel.y = centerPixel.y + imagesize.height / 2;
}

cv::Point3d Camera::rotatePoint(const cv::Point3d& worldPoint, double roll, double pitch, double yaw)
{
    cv::Mat in_point(1, 3, CV_64F, double(0));
    in_point.at<double>(0) = worldPoint.x;
    in_point.at<double>(1) = worldPoint.y;
    in_point.at<double>(2) = worldPoint.z;

    cv::Mat rotZ(cv::Matx33d(1, 0, 0,
                            0, cos(yaw), sin(yaw),
                            0, -sin(yaw), cos(yaw)));
    cv::Mat rotX(cv::Matx33d(cos(pitch), 0, -sin(pitch),
                            0, 1, 0,
                            sin(pitch), 0, cos(pitch)));
    cv::Mat rotY(cv::Matx33d(cos(roll), -sin(roll), 0,
                            sin(roll), cos(roll), 0,
                            0, 0, 1));
    cv::Mat new_point = in_point * rotY * rotZ * rotX;
    
    return cv::Point3d(new_point);         // opencv calib3d/utils  proposes this order
}

void Camera::setExtrinsics(cv::Vec3d pos, VecRot rot)
{
	T = pos;
	R = rot;
}

void Camera::setIntrinsics(cv::Size newsize, double xF, double yF)
{
    newSize = newsize;
    xFov = xF;
    yFov = yF;           // * 9/16 - vertical fov
}

void Camera::setModelName(std::string modelName)
{
	this->modelName = modelName;
}


bool Camera::check_pose(cv::Point3d local_coords) {
    // TODO: implement

    if (local_coords.z < 0) return false;
    
    return true;
}

cv::Point2i Camera::projectWorldToPixel(cv::Point3d worldPoint)
{
    // correct for camera rotation 
    cv::Point3d cam_point = rotatePoint(worldPoint, R.roll, -R.yaw, R.pitch);
    
    if (!check_pose(cam_point)) return {-1, -1};

    double xPinholeFocus = xFov;
    double yPinholeFocus = yFov;

    double cx = cam_point.x;
    double cy = cam_point.y;
    double cz = cam_point.z;

    cv::Point2i pinholePoint;
    pinholePoint.x = xPinholeFocus * cx / cz;
    pinholePoint.y = yPinholeFocus * cy / cz;

    toCorner(pinholePoint, newSize);

    return pinholePoint;
}

cv::Point3d Camera::projectPixelToWorld(cv::Point2i pixel)
{
    toCenter(pixel, newSize);
    //std::cout << pixel << " | " << newSize << std::endl;

    float cz = 10;                                    // doesnt really affect much
    double xFovRad = xFov * M_PI / 180;
    double yFovRad = yFov * M_PI / 180;
    double xPinholeFocus = newSize.width / (2 * tan(xFovRad / 2));
    double yPinholeFocus = newSize.height / (2 * tan(yFovRad / 2));

    cv::Point3d cameraCoords;
    cameraCoords.x = pixel.x * cz / xPinholeFocus;
    cameraCoords.y = pixel.y * cz / yPinholeFocus;
    cameraCoords.z = cz;
    //std::cout << xPinholeFocus << " | " << yPinholeFocus << std::endl;
    return cameraCoords;
}

}
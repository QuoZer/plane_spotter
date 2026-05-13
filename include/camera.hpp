#pragma once
#define _USE_MATH_DEFINES
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <plane.hpp>


namespace plane_spotter
{
	
/// <summary>
/// A parent class for all the camera models. 
/// </summary>
class Camera
{
public:	///* Parameters *///
	float xFov;							
	float yFov;								
	cv::Size newSize;					
	std::string modelName;
    
	cv::Vec3d T;
	VecRot R;

	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;

public:	///* Internal functions *///
	/* Tools */


	/// <summary>
	/// Transforms a pixel from opencv corner reference frame to the central one
	/// </summary>
	/// <param name="cornerPixel"> Pixel that needs to be transformed </param>
	/// <param name="imagesize"> Image size </param>
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize);

	/// <summary>
	/// Transforms a pixel from central reference frame to the opencv one
	/// </summary>
	/// <param name="centerPixel"> Pixel that needs to be transformed </param>
	/// <param name="imagesize"> Image size </param>
	void toCorner(cv::Point& centerPixel, cv::Size imagesize);

	cv::Point3d rotatePoint(const cv::Point3d& worldPoint, double roll, double pitch, double yaw);

	bool check_pose(cv::Point3d local_coords);


public:		///* Settings *///
	Camera();

	/// <summary>
	/// Set camera position and orientation on your robot/setup. Keep in mind that all cameras should use common reference frame
	/// </summary>
	/// <param name="pos"> Position in xyz coordinates </param>
	/// <param name="rot"> Orientation in quaternion </param>
	void setExtrinsics(cv::Vec3d pos, VecRot rot);

	/// <summary>
	/// Sets an identifier for the camera 
	/// </summary>
	/// <param name="modelName"> Camera name </param>
	void setModelName(std::string modelName);

	// trying to use one agnostic function to set parameters no matter the parameters
	void setIntrinsics(cv::Size newsize, double xF, double yF);

	/* Projection functions */

	/// <summary>
	/// Takes a 3D point in camera coordinates and projects it into fisheye image coordinates
	/// </summary>
	/// <param name="worldPoint"> 3D point in camera coordinates </param>
	/// <returns> 2D point in image plane </returns>
	cv::Point2i projectWorldToPixel(cv::Point3d worldPoint); 		

	/// <summary>
	///  Takes a point in image (central coordinates) and projects it into camera coordinates. 
	/// </summary>
	/// <param name="pixel"> 2D point in image plane </param>
	/// <returns> 3D point in camera coordinates </returns>
	cv::Point3d projectPixelToWorld(cv::Point2i pixel);

};


} // namespace plane_spotter

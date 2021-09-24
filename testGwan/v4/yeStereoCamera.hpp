#ifndef YESTEREOCAMERA_HPP
#define YESTEREOCAMERA_HPP

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>

namespace SYE {

class YeStereoCamera {
private:
	
protected:
	virtual bool doCalibration(const char *pPath) = 0;
	virtual bool doCalibration(std::vector<std::string> &imgList) = 0;
public:
	YeStereoCamera();
	//virtual ~YeStereoCamera();
};

class Calibration : public YeStereoCamera 
{

private:
	int CHECKERBOARD[2]{ 6, 9 };
	
	std::vector<cv::String> images;

	
	std::vector<cv::Point3f> objp;
	std::vector<cv::Point2f> corner_pts;
	
	std::vector<std::vector<cv::Point3f>> objpoints_right;	
	std::vector<std::vector<cv::Point3f>> objpoints_left;	
	std::vector<std::vector<cv::Point2f>> imgpoints_right;			
	std::vector<std::vector<cv::Point2f>> imgpoints_left;
		
		
	cv::Mat frame, gray, lrImage[2];
	bool success;						       
	char buf[256];	
	
	cv::Mat cameraMatrix_left, distCoeffs_left;	
	cv::Mat cameraMatrix_right, distCoeffs_right;
	cv::Mat R, T, E, F;
	cv::Size imgsize;
	
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	
	
public:
	bool doCalibration(std::vector<std::string> &imgList);
	bool doCalibration(const char *pPath);
};

}


#endif

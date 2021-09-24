#include "yeStereoCamera.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <iostream>
#include <vector>

using namespace std;

class hwangCalibration: public SYE::YeStereoCamera{
	int CHECKERBOARD[2]{6,9};

	vector<vector<cv::Point3f>> objpoints_left;
	vector<vector<cv::Point3f>> objpoints_right;
	
	vector<vector<cv::Point2f>> imgpoints_left;
	vector<vector<cv::Point2f>> imgpoints_right;

	vector<cv::Point3f> objp;

	cv::Mat frame, gray, pic[2];

	vector<cv::Point2f> corner_pts;

	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

	cv::Mat R, T, E, F;

	cv::Size imgsize;

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

public:
	virtual bool doCalibration(const char *pPath);
	virtual bool doCalibration(vector<string> &imgList);
};

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

#include "yeStereoCamera.hpp"

using namespace cv;
using namespace std;
using namespace SYE;

class stereo: public YeStereoCamera
{
private:
	int CHECKERBOARD[2]{6, 9};

	/* Creating vector to store vector of 3D points for each checkerboard image */
	vector<vector<cv::Point3f> > objpoints_right;
	vector<vector<cv::Point3f> > objpoints_left;

	/* Creating vector to store vectors of 2D points for each checkerboard image */
	vector<vector<cv::Point2f> > imgpoints_right;
	vector<vector<cv::Point2f> > imgpoints_left;

	/* Defining the world coordinates for 3D points */
	vector<cv::Point3f> objp;									// 월드 좌표계 선언
	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left; // 파라미터를 구하기 위한 Mat 객체 선언
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

	// stereo camera calibration result
	cv::Mat R, T, E, F;

	// stereoRectify
	cv::Mat R1, R2, P1, P2, Q;

	cv::Rect validRoi[2];

public:
	bool doCalibration(const char *pPath);
	bool doCalibration(std::vector<std::string> &imgList);

};
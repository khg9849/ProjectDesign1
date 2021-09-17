#pragma warning(disable:4996);
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/opencv_modules.hpp>
//#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main()
{
	vector<cv::String> images;
	string path = "./linkFolder/s1625714490-90";
	
	cv::glob(path, images);
	
	
	cout << "로드한 이미지 개수 : " << images.size() << endl;
	if (images.size() == 0)
		cout << "이미지가 존재하지 않음! \n" << endl;
		
	cv::Mat frame;
	frame = cv::imread(images[0]);
	
	//image read
	cv::Mat lrImg[2];
	lrImg[0] = frame(Range::all(), Range(0, frame.cols/2));
	lrImg[1] = frame(Range::all(), Range(frame.cols/2, frame.cols));
	
	
	//xml read
	cv::FileStorage fsFrontRead("CameraParams.xml", cv::FileStorage::READ);
	
	
	//camera parameters
	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;	
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

	fsFrontRead["Left_CameraMatrix"] >> cameraMatrix_left;
	fsFrontRead["Left_DistCoeffs"] >> distCoeffs_left;
	fsFrontRead["Left_Rotation_Vector"] >> R_left;
	fsFrontRead["Left_Translation_Vector"] >> T_left;

	fsFrontRead["Right_CameraMatrix"] >> cameraMatrix_right;
	fsFrontRead["Right_DistCoeffs"] >> distCoeffs_right;
	fsFrontRead["Right_Rotation_Vector"] >> R_right;
	fsFrontRead["Right_Translation_Vector"] >> T_right;
	
	//stereo camera parameters
	cv::Mat R, T, E, F;
	cv::Mat R1, R2, P1, P2, Q;
	
	
	fsFrontRead["Rotation_Matrix"] >> R;
	fsFrontRead["Translation_Vector"] >> T;
	fsFrontRead["Essential_Matrix"] >> E;
	fsFrontRead["Fundamental_Matrix"] >> F;
	
	fsFrontRead["R1"] >> R1;
	fsFrontRead["R2"] >> R2;
	fsFrontRead["P1"] >> P1;
	fsFrontRead["P2"] >> P2;
	fsFrontRead["Q"] >> Q;
	
	fsFrontRead.release();
	
	//image undistort
	Mat imageUndistorted[2];
	undistort(lrImg[0], imageUndistorted[0], cameraMatrix_left, distCoeffs_left);
	undistort(lrImg[1], imageUndistorted[1], cameraMatrix_left, distCoeffs_left);
	
	imshow("lrImg[0]", lrImg[0]);
	imshow("imageUndistorted0", imageUndistorted[0]);
	imshow("lrImg[1]", lrImg[1]);
	imshow("imageUndistorted1", imageUndistorted[1]);
	waitKey();
	
	return 0;
}
	
	
	
	
	
	

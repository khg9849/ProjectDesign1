#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <iostream>
#include <vector>
using namespace std;

int CHECKERBOARD[2]{6,9};

int main(){
	vector<cv::String> images;
	string path = "./image";
	// image file 

	cv::glob(path, images);

	vector<vector<cv::Point3f>> objpoint;
	vector<vector<cv::Point2f>> imgpoint;

	vector<cv::Point3f> objp;

	for(int i = 0; i < CHECKERBOARD[1]; i++){
		for(int j = 0; j < CHECKERBOARD[0]; j++){
			objp.push_back(cv::Point3f(j*28, i*28, 0));
		}
	}

	cv::Mat frame, gray;

	vector<cv::Point2f> corner_pts;
	bool success;
	char buf[256];
	int index = 0;

	for(int i = 0; i < images.size(); i++){
		frame = cv::imread(images[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	
		success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if(success){
			cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

			cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

			cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

			objpoint.push_back(objp);
			imgpoint.push_back(corner_pts);
		}
	}

	cv::destroyAllWindows();

	cv::Mat cameraMatrix, distCoeffs, rotation, transition;

	cv::calibrateCamera(objpoint, imgpoint, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, rotation, transition);

	cout << "[Camera Parameters]" << endl;
	cout << "CameraMatrix" << endl << cameraMatrix << endl << endl;
	cout << "DistCoeffs" << endl << distCoeffs << endl << endl;
	cout << "Rotation Vector" << endl << rotation << endl << endl;
	cout << "Translation Vector" << endl << transition << endl << endl << endl;

	cv::Mat new_frame;
	frame = cv::imread(images[0]);
	cv::undistort(frame, new_frame, cameraMatrix, distCoeffs);
	imshow("Test", frame);
	cv::waitKey(0);
	imshow("Test", new_frame);
	cv::waitKey(0);

	return 0;
}

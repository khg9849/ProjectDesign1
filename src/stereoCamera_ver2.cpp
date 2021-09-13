#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <iostream>
#include <vector>
using namespace std;

int CHECKERBOARD[2]{6,9};

int main(){
	vector<cv::String> images;

	string path = "./sCalibration-90";

	cv::glob(path, images);

	vector<vector<cv::Point3f>> objpoints_left;
	vector<vector<cv::Point3f>> objpoints_right;

	vector<vector<cv::Point2f>> imgpoints_left;
	vector<vector<cv::Point2f>> imgpoints_right;

	vector<cv::Point3f> objp;

	for(int i = 0; i < CHECKERBOARD[1]; i++){
		for(int j = 0; j < CHECKERBOARD[0]; j++){
			objp.push_back(cv::Point3f(j*23.9, i*23.9, 0));
		}
	}

	cv::Mat frame, gray, pic[2];

	vector<cv::Point2f> corner_pts;

	for(int i = 0; i < images.size(); i++){
		frame = cv::imread(images[i]);

		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

		pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		pic[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));

		for(int j = 0; j < 2; j++){
			if(cv::findChessboardCorners(pic[j], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE)){

				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

				cv::cornerSubPix(pic[j], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
		
				if(!j){
					objpoints_left.push_back(objp);
					imgpoints_left.push_back(corner_pts);
				}
				else{
					objpoints_right.push_back(objp);
					imgpoints_right.push_back(corner_pts);
				}
			}
		}
	}

	cv::Mat cameraMatrix_left, distCoeffs_left;
	cv::Mat cameraMatrix_right, distCoeffs_right;
	cv::Mat R, T, E, F;

	cv::Size imgsize = pic[0].size();
	
	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_PRINCIPAL_POINT, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

	cout << "[Stereo Camera parameters]\n";
	cout << "Rotation Matrix\n" << R << "\n\n";
	cout << "Translation Vector\n" << T << "\n\n";
	cout << "Essential Matrix\n" << E << "\n\n";
	cout << "Fundamental Matrix\n" << F << "\n\n\n";
	
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	
	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
	
	cout << "[Stereo Rectify parameters]\n";
	cout << "R1\n" << R1 << "\n\n";
	cout << "R2\n" << R2 << "\n\n";
	cout << "P1\n" << P1 << "\n\n";
	cout << "P2\n" << P2 << "\n\n";
	cout << "Q\n" << Q << "\n\n\n";

	return 0;
}

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
			objp.push_back(cv::Point3f(j*23.7, i*23.7, 0));
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

	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(pic[0].rows, pic[0].cols), cameraMatrix_left, distCoeffs_left, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(pic[1].rows, pic[1].cols), cameraMatrix_right, distCoeffs_right, R_right, T_right);

	cv::Mat R, T, E, F;

	cv::Size imgsize = pic[0].size();
	
	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	
	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
	
	cv::FileStorage fsw("calibrationData_90.xml", cv::FileStorage::WRITE);
	fsw << "leftCameraMatrix" << cameraMatrix_left;
	fsw << "leftCameraDistCoeff" << distCoeffs_left;
	fsw << "rightCameraMatrix" << cameraMatrix_right;
	fsw << "rightCameraDistCoeff" << distCoeffs_right;
	fsw << "rotationMatrix" << R;
	fsw << "translationVertor" << T;
	fsw << "essentialMatrix" << E;
	fsw << "fundamentalMatrix" << F;
	fsw << "rectifyR1" << R1;
	fsw << "rectifyR2" << R2;
	fsw << "rectifyP1" << P1;
	fsw << "rectifyP2" << P2;
	fsw << "rectifyQ" << Q;

	return 0;
}

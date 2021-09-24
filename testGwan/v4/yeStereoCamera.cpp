#include "yeStereoCamera.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <vector>
#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}



bool Calibration::doCalibration(std::vector<std::string> &imgList){
	if(imaList.size() == 0){
		std::cout << "이미지가 존재하지 않음! \n" << std::endl;
		return false;	
	}
	else
		std::cout << "로드한 이미지 개수 : " << imaList.size() << std::endl;
	
	//set checkerboard

	for(int i = 0; i < CHECKERBOARD[1]; i++){
		for(int j = 0; j < CHECKERBOARD[0]; j++){
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));
		}
	}
	
	//find chess corner
	
	
	for(int i = 0; i < imaList.size(); i++){
		frame = cv::imread(imaList[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		
		for(int l = 0; l < 2; l++){
			
			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
				
				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	
				
				 

				if ((l % 2) == 0)
				{
					objpoints_left.push_back(objp);
					imgpoints_left.push_back(corner_pts);		
				}
				else
				{
					objpoints_right.push_back(objp);
					imgpoints_right.push_back(corner_pts);		
				}

			}
			else
				return false;
		}
		
	}
	
	//stereo calibration
	
	imgsize  = lrImage[0].size();

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_PRINCIPAL_POINT, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	
	//save XLM file
	
	cv::FileStorage fs("calibration.xml", cv::FileStorage::WRITE);
	if(fs.isOpened()){
		fs << "image_Size" << imgsize;
		fs << "CameraMatrix_LEFT" << cameraMatrix_left << "DistCoefficient_LEFT" << distCoeffs_left << "CameraMatrix_RIGHT" << cameraMatrix_right << "DistCoefficient_RIGHT" << distCoeffs_right;
		fs << "RotationMatrix" << R << "TranslationVector" << T << "EssentialMatrix" << E << "FundamentalMatrix" << F;
		fs << "R1" << R1 << "R2" << R2 << "P1" << "P2" << "Q" << Q;
	}	
	else{
		std::cout << "Error : can not save the extrinsic parameters" << std::endl;
		fs.release();
		return false;
	}
	fs.release();
	
	
	return true;
}
bool Calibration::doCalibration(const char *pPath){
	
	
	
	//load images
	

	cv::glob(pPath, imaList);
	
	if(imaList.size() == 0){
		std::cout << "이미지가 존재하지 않음! \n" << std::endl;
		return false;	
	}
	else
		std::cout << "로드한 이미지 개수 : " << imaList.size() << std::endl;
	
	//set checkerboard

	for(int i = 0; i < CHECKERBOARD[1]; i++){
		for(int j = 0; j < CHECKERBOARD[0]; j++){
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));
		}
	}
	
	//find chess corner
	
	
	for(int i = 0; i < imaList.size(); i++){
		frame = cv::imread(imaList[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		
		for(int l = 0; l < 2; l++){
			
			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
				
				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	
				
				 

				if ((l % 2) == 0)
				{
					objpoints_left.push_back(objp);
					imgpoints_left.push_back(corner_pts);		
				}
				else
				{
					objpoints_right.push_back(objp);
					imgpoints_right.push_back(corner_pts);		
				}

			}
			else
				return false;
		}
		
	}
	
	//stereo calibration
	
	imgsize  = lrImage[0].size();

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_PRINCIPAL_POINT, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	
	//save XLM file
	
	cv::FileStorage fs("calibration.xml", cv::FileStorage::WRITE);
	if(fs.isOpened()){
		fs << "image_Size" << imgsize;
		fs << "CameraMatrix_LEFT" << cameraMatrix_left << "DistCoefficient_LEFT" << distCoeffs_left << "CameraMatrix_RIGHT" << cameraMatrix_right << "DistCoefficient_RIGHT" << distCoeffs_right;
		fs << "RotationMatrix" << R << "TranslationVector" << T << "EssentialMatrix" << E << "FundamentalMatrix" << F;
		fs << "R1" << R1 << "R2" << R2 << "P1" << "P2" << "Q" << Q;
	}	
	else{
		std::cout << "Error : can not save the extrinsic parameters" << std::endl;
		fs.release();
		return false;
	}
	fs.release();
	
	
	return true;
}


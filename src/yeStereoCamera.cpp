#include "yeStereoCamera.hpp"

#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}
YeStereoCamera::~YeStereoCamera() {
}

//경로 내부의 이미지 파일을 읽어서 켈리브레이션 실시.
bool YeStereoCamera::doCalibration(const char* pPath, const char* ext = "*.jpg", const char* xmlName) {
	
	int CHECKERBOARD[2]{ 6, 9 };

	/*		Load Images		*/

	vector<cv::String> images;
	cv::glob(pPath + ext, images);
	std::cout << "Number of Loaded images : " << images.size() << std::endl;
	if (images.size() == 0) {
		std::cout << "Error: No images!\n" << std::endl;
		return false;
	}

	/*   Calibration 시작   */

	vector<vector<cv::Point3f> > objpoints_right;			// 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언
	vector<vector<cv::Point3f> > objpoints_left;			// 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언

	vector<vector<cv::Point2f> > imgpoints_right;			// 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언
	vector<vector<cv::Point2f> > imgpoints_left;			// 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언

	vector<cv::Point3f> objp;								// 월드 좌표계 선언


	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	// 체스보드 한 칸의 길이 조정해야 함
			
		}
	}

	cv::Mat frame, gray, lrImage[2];

	vector<cv::Point2f> corner_pts;	
	bool success;
	char buf[256];
	int index = 0;


	for (int i = 0; i < images.size(); i++) {
		frame = cv::imread(images[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

	
		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols / 2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols / 2, gray.cols));
		
		for (int l = 0; l < 2; l++) {
		
			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
				
				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	// 주어진 2D point에 대해 더 정제시켜 업데이트
			
				if ((l % 2) == 0) {
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
				}
				else {
					for (int k = 0; k < corner_pts.size(); k++)
					{
						corner_pts[k].x += lrImage[l].cols;
					}
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

				}
				

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
		}
		cv::imshow("Image", frame);//image 띄우기

		index++;

		cv::waitKey(0);	
	}
	cv::destroyAllWindows(); // 윈도우 창 강제종료, imshow 빼면 같이 빼기


	cv::Mat R_left, T_left, R_right, T_right;

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), matCamMat1, matDistCoffs1, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), matCamMat2, matDistCoffs2, R_right, T_right);


	cv::Size imgsize(gray.rows, gray.cols / 2);

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize,
		matR, matT, matE, matF, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

	cv::FileStorage fs(xmlName, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "Size" << imgsize;
		fs << "matCamMat1" << matCamMat1 << "matDistCoffs1" << matDistCoffs1 << "matCamMat2" << matCamMat2 << "matDistCoffs2" << matDistCoffs2;
		fs << "matR" << matR << "matT" << matT;
		fs.release();
	}
	else {
		std::cout << "Error : can not save the extrinsic parameters" << std::endl;
		return false;
	}

	return true;

}
bool YeStereoCamera::doCalibration(std::vector<std::string>& imgList, const char* xmlName) {
	
	int CHECKERBOARD[2]{ 6, 9 };


	/*   Calibration 시작   */

	vector<vector<cv::Point3f> > objpoints_right;			// 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언
	vector<vector<cv::Point3f> > objpoints_left;			// 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언

	vector<vector<cv::Point2f> > imgpoints_right;			// 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언
	vector<vector<cv::Point2f> > imgpoints_left;			// 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언

	vector<cv::Point3f> objp;								// 월드 좌표계 선언


	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	// 체스보드 한 칸의 길이 조정해야 함

		}
	}

	cv::Mat frame, gray, lrImage[2];

	vector<cv::Point2f> corner_pts;
	bool success;
	char buf[256];
	int index = 0;


	for (int i = 0; i < imgList.size(); i++){
		frame = cv::imread(imgList[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);


		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols / 2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols / 2, gray.cols));

		for (int l = 0; l < 2; l++) {

			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	// 주어진 2D point에 대해 더 정제시켜 업데이트

				if ((l % 2) == 0) {
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
				}
				else {
					for (int k = 0; k < corner_pts.size(); k++)
					{
						corner_pts[k].x += lrImage[l].cols;
					}
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

				}


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
		}
		cv::imshow("Image", frame);

		index++;

		cv::waitKey(0);
	}
	cv::destroyAllWindows();


	cv::Mat R_left, T_left, R_right, T_right;

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), matCamMat1, matDistCoffs1, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), matCamMat2, matDistCoffs2, R_right, T_right);


	cv::Size imgsize(gray.rows, gray.cols / 2);

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize,
		matR, matT, matE, matF, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

	cv::FileStorage fs(xmlName, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "Size" << imgsize;
		fs << "matCamMat1" << matCamMat1 << "matDistCoffs1" << matDistCoffs1 << "matCamMat2" << matCamMat2 << "matDistCoffs2" << matDistCoffs2;
		fs << "matR" << matR << "matT" << matT;
		fs.release();
	}
	else {
		std::cout << "Error : can not save the extrinsic parameters" << std::endl;
		return false;
	}

	return true;
}



// Yolo를 이용하여 특정 이름의 영역을 추출.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, bbox_t* pObjRect) 
{

}



//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, std::vector<YePos3D>& features) {

}


// 추춘된 특정 영역만 SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 팀 화 이 팅 ! ! ! ,,----------------------------------------------------*/
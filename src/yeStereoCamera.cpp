#include "yeStereoCamera.hpp"
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
using namespace SYE;
using std::vector;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}
YeStereoCamera::~YeStereoCamera() {
}

//��� ������ �̹��� ������ �о �̸��극�̼� �ǽ�.
bool YeStereoCamera::doCalibration(const char* pPath, const char* xmlName, const char* ext) {
	
	int CHECKERBOARD[2]{ 6, 9 };

	/*		Load Images		*/
	
	vector<cv::String> images;
	cv::glob(pPath, images);
	std::cout << "Number of Loaded images : " << images.size() << std::endl;
	if (images.size() == 0) {
		std::cout << "Error: No images!\n" << std::endl;
		return false;
	}

	/*   Calibration ����   */

	vector<vector<cv::Point3f> > objpoints_right;			// �� �̹����� ���� 3D ��ǥ�� �����ϴ� ���� ����
	vector<vector<cv::Point3f> > objpoints_left;			// �� �̹����� ���� 3D ��ǥ�� �����ϴ� ���� ����

	vector<vector<cv::Point2f> > imgpoints_right;			// �� �̹����� ���� 2D ��ǥ�� �����ϴ� ���� ����
	vector<vector<cv::Point2f> > imgpoints_left;			// �� �̹����� ���� 2D ��ǥ�� �����ϴ� ���� ����

	vector<cv::Point3f> objp;								// ���� ��ǥ�� ����


	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	// ü������ �� ĭ�� ���� �����ؾ� ��
			
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
				
				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	// �־��� 2D point�� ���� �� �������� ������Ʈ
			
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
		cv::imshow("Image", frame);//image ����

		index++;

		cv::waitKey(0);	
	}
	cv::destroyAllWindows(); // ������ â ��������, imshow ���� ���� ����


	cv::Mat R_left, T_left, R_right, T_right;
	cv::Mat matR, matT, matE, matF, matCamMat1, matCamMat2, matDistCoffs1, matDistCoffs2;

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
bool YeStereoCamera::doCalibration(vector<std::string>& imgList, const char* xmlName) {
	
	int CHECKERBOARD[2]{ 6, 9 };


	/*   Calibration ����   */

	vector<vector<cv::Point3f> > objpoints_right;			// �� �̹����� ���� 3D ��ǥ�� �����ϴ� ���� ����
	vector<vector<cv::Point3f> > objpoints_left;			// �� �̹����� ���� 3D ��ǥ�� �����ϴ� ���� ����

	vector<vector<cv::Point2f> > imgpoints_right;			// �� �̹����� ���� 2D ��ǥ�� �����ϴ� ���� ����
	vector<vector<cv::Point2f> > imgpoints_left;			// �� �̹����� ���� 2D ��ǥ�� �����ϴ� ���� ����

	vector<cv::Point3f> objp;								// ���� ��ǥ�� ����


	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	// ü������ �� ĭ�� ���� �����ؾ� ��

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

				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	// �־��� 2D point�� ���� �� �������� ������Ʈ

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



// Yolo�� �̿��Ͽ� Ư�� �̸��� ������ ����.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, bbox_t* pObjRect) 
{

}

//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, std::vector<YePos3D>& features) {

}

// ����� Ư�� ������ SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 �� ȭ �� �� ! ! ! ,,----------------------------------------------------*/
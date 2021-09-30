#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

#include "stereo.hpp"

using namespace cv;
using namespace std;
using namespace SYE;

bool stereo::doCalibration(const char *pPath)
{
	/* Extracting path of individual image stored in a given directory */
	vector<String> images;
	vector<string> imgList;
	cv::glob(pPath, images, false);
	for (int i = 0; i < images.size(); i++)
		imgList.push_back(images[i]);
	return doCalibration(imgList);
}
bool stereo::doCalibration(std::vector<std::string> &imgList)
{

	cout << "로드한 이미지 개수 : " << imgList.size() << endl;
	if (imgList.size() == 0)
	{
		cout << "이미지가 존재하지 않음! \n";
		return false;
	}

	//setObjp
	for (int i = 0; i < CHECKERBOARD[1]; i++)	  // CHECKERBOARD[0] = 6
		for (int j = 0; j < CHECKERBOARD[0]; j++) // CHECKERBOARD[1] = 9
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));
	// z는 0 이고 (x, y ,z)로 담기니까 (j, i , 0)으로 벡터에 push_back으로 값 담기

	cv::Mat frame, gray;

	/* vector to store the pixel coordinates of detected checker board corners */
	vector<cv::Point2f> corner_pts; // 검출된 check board corner 포인트(2D 좌표)를 담을 벡터 선언
	bool success;					// findChessboardCorners 되었는지 안 되었는지를 확인하기 위한 용도
	char buf[256];
	int index = 0;

	cv::Mat lrImg[2];

	// get objpoints and imgpoints
	for (int i = 0; i < imgList.size(); i++)
	{
		frame = cv::imread(imgList[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
		lrImg[0] = gray(Range::all(), Range(0, gray.cols / 2));
		lrImg[1] = gray(Range::all(), Range(gray.cols / 2, gray.cols));

		for (int ii = 0; ii < 2; ii++)
		{
			success = cv::findChessboardCorners(lrImg[ii], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
												CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.001);

				/* refining pixel coordinates for given 2d points. */
				cv::cornerSubPix(lrImg[ii], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria); // 주어진 2D point에 대해 더 정제시켜 업데이트

				if ((ii % 2) == 0)
				{
					objpoints_left.push_back(objp);		  // 해당 번째의 objp의 값을 objpoints에 추가
					imgpoints_left.push_back(corner_pts); // 해당 번째의 corner_pts의 값을 imgopoints에 추가
				}
				else
				{
					for (int k = 0; k < corner_pts.size(); k++)
						corner_pts[k].x += lrImg[ii].cols;
					objpoints_right.push_back(objp);	   // 해당 번째의 objp의 값을 objpoints에 추가
					imgpoints_right.push_back(corner_pts); // 해당 번째의 corner_pts의 값을 imgopoints에 추가
				}

				/* Displaying the detected corner points on the checker board */
				/*
					if ((ii % 2) == 0){
						cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
						imshow("chessboard",frame);
						waitKey();
					}
					else
						cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
					*/
			}
		}
	}

	// 위에서 저장했던 object point와 image point를 이용하여 parameter 구하기
	// get cameraMatrix, distCoeffs, R, T
	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(gray.rows, gray.cols / 2), cameraMatrix_left, distCoeffs_left, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(gray.rows, gray.cols / 2), cameraMatrix_right, distCoeffs_right, R_right, T_right);

	/*      Stereo Calibration 시작     */
	cv::Size imgsize(gray.rows, gray.cols / 2);
	// get E, F
	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
						cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize,
						R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	// get R1, R2, P1, P2, Q
	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize,
					  R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);

	FileStorage fs("calibration_result.xml", cv::FileStorage::WRITE);
	if (!fs.isOpened())
	{
		cerr << "OUTPUT FILE cannot be opened" << '\n';
		return false;
	}

	fs << "cameraMatrix_left" << cameraMatrix_left;
	fs << "distCoeffs_left" << distCoeffs_left;
	fs << "cameraMatrix_right" << cameraMatrix_right;
	fs << "distCoeffs_right" << distCoeffs_right;

	fs << "Rotation_Matrix" << R;
	fs << "Translation_Vector" << T;
	fs << "Translation_Vector" << E;
	fs << "Translation_Vector" << F;

	fs << "Rotation_Matrix_1" << R1;
	fs << "Rotation_Matrix_2" << R2;
	fs << "Projection_Matrix_1" << P1;
	fs << "Projection_Matrix_2" << P2;
	fs << "disparity-to-depth_mapping_matrix" << Q;

	fs.release();
	cout << "calibration_result.xml is saved\n";

	return true;
}
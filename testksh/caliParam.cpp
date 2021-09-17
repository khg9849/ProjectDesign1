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

int CHECKERBOARD[2]{ 6, 9 };

int main()
{
	vector<cv::String> images;
	string path = "./linkFolder/sCalibration-90";

	cv::glob(path, images);
	
	
	cout << "로드한 이미지 개수 : " << images.size() << endl;
	if (images.size() == 0)
		cout << "이미지가 존재하지 않음! \n" << endl;


	vector<vector<cv::Point3f> > objpoints_right;	
	vector<vector<cv::Point3f> > objpoints_left;

	vector<vector<cv::Point2f> > imgpoints_right;		
	vector<vector<cv::Point2f> > imgpoints_left;		


	vector<cv::Point3f> objp;		

	for (int i = 0; i < CHECKERBOARD[1]; i++)		
		for (int j = 0; j < CHECKERBOARD[0]; j++)	
			objp.push_back(cv::Point3f(j * 10, i * 10, 0));	



	cv::Mat frame, gray;
	vector<cv::Point2f> corner_pts;	
		
	bool success;		
	char buf[256];
	int index = 0;
	
	cv::Mat lrImg[2];
	for (int i = 0; i < images.size(); i++)			
	{
		frame = cv::imread(images[i]);	
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);	
		
		lrImg[0] = gray(Range::all(), Range(0, gray.cols/2));
		lrImg[1] = gray(Range::all(), Range(gray.cols/2, gray.cols));

		for(int ii = 0; ii < 2; ii++)
		{
		
			success = cv::findChessboardCorners(lrImg[ii], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
							CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.001);
				
				cv::cornerSubPix(lrImg[ii], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
				
				
				if((ii % 2) != 0)
					for(int k = 0; k < corner_pts.size(); k++)
						corner_pts[k].x += lrImg[ii].cols;
						
				cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

				if ((ii % 2) == 0)
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
/*
		cv::imshow("Image", frame);			

		sprintf(buf, "%d_Image.jpg", index);
		cv::imwrite(buf, frame);
		index++;
	
		cv::waitKey(0);	
		*/				
	}

	cv::destroyAllWindows();			

	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;	
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

	// cameraMatrix는 내부 파라미터
	// distCoeffs는 왜곡 파라미터
	// R, T 는 외부 파라미터

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(gray.rows, gray.cols/2), cameraMatrix_left, distCoeffs_left, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(gray.rows, gray.cols/2), cameraMatrix_right, distCoeffs_right, R_right, T_right);
	
	cv::FileStorage fsFront("CameraParams.xml", cv::FileStorage::WRITE);
	fsFront << "Left_CameraMatrix" << cameraMatrix_left;
	fsFront << "Left_DistCoeffs" << distCoeffs_left;
	fsFront << "Left_Rotation_Vector" << R_left;
	fsFront << "Left_Translation_Vector" << T_left;

	fsFront << "Right_CameraMatrix" << cameraMatrix_right;
	fsFront << "Right_DistCoeffs" << distCoeffs_right;
	fsFront << "Right_Rotation_Vector" << R_right;
	fsFront << "Right_Translation_Vector" << T_right;
	
	cv::Mat R, T, E, F;
	cv::Size imgsize(gray.rows, gray.cols/2);

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize,
		R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));


	fsFront << "Rotation_Matrix" << R;
	fsFront << "Translation_Vector" << T;
	fsFront << "Essential_Matrix" << E;
	fsFront << "Fundamental_Matrix" << F;

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize,
		R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);

	fsFront << "R1" << R1;
	fsFront << "R2" << R2;
	fsFront << "P1" << P1;
	fsFront << "P2" << P2;
	fsFront << "Q" << Q;
	
	fsFront.release();
	/*
	Mat imageUndistorted[2];
	undistort(lrImg[0], imageUndistorted[0], cameraMatrix_left, distCoeffs_left);
	undistort(lrImg[1], imageUndistorted[1], cameraMatrix_left, distCoeffs_left);
	
	imshow("lrImg[0]", lrImg[0]);
	imshow("imageUndistorted0", imageUndistorted[0]);
	imshow("lrImg[1]", lrImg[1]);
	imshow("imageUndistorted1", imageUndistorted[1]);
	waitKey();
	*/
	
	return 0;
}


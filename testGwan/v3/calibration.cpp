#include <opencv2/opencv.hpp>

#include <opencv2/calib3d/calib3d_c.h>

#include <iostream>
#include <vector>


using namespace std;

int CHECKERBOARD[2]{ 6, 9 };


int main()
{
	
	vector<cv::String> images;

	string path = "../image/90chess";

	cv::glob(path, images);		
	cout << "로드한 이미지 개수 : " << images.size() << endl;
	if (images.size() == 0)
		cout << "이미지가 존재하지 않음! \n" << endl;

	
	vector<vector<cv::Point3f> > objpoints_right;	
	vector<vector<cv::Point3f> > objpoints_left;	

	vector<vector<cv::Point2f> > imgpoints_right;			
	vector<vector<cv::Point2f> > imgpoints_left;	


	
	vector<cv::Point3f> objp;
								
	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	
		}
	}

	

	cv::Mat frame, gray, lrImage[2];
	
	
	vector<cv::Point2f> corner_pts;		
	bool success;						       
	char buf[256];

	
	for (int i = 0; i < images.size(); i++)				
	{
		frame = cv::imread(images[i]);	
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
		}
		
	}


	cv::Mat cameraMatrix_left, distCoeffs_left;	
	cv::Mat cameraMatrix_right, distCoeffs_right;
	cv::Mat R, T, E, F;
	// cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
	//cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), cameraMatrix_left, distCoeffs_left, R_left, T_left);
	//cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), cameraMatrix_right, distCoeffs_right, R_right, T_right);

	

	
	cv::Size imgsize = lrImage[0].size();

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_PRINCIPAL_POINT, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	
	
	//cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	
	cout << "[Stereo Camera parameters]\n";
	cout << "Rotation Matrix\n" << R << "\n\n";
	cout << "Translation Vector\n" << T << "\n\n";
	cout << "Essential Matrix\n" << E << "\n\n";
	cout << "Fundamental Matrix\n" << F << "\n\n\n";
	
	cv::FileStorage fs("90d_calibration_withDrawChessBoard.xml", cv::FileStorage::WRITE);
	if(fs.isOpened()){
		fs << "Size" << imgsize;
		fs << "C_LEFT" << cameraMatrix_left << "D_LEFT" << distCoeffs_left << "C_RIGHT" << cameraMatrix_right << "D_RIGHT" << distCoeffs_right;
		fs << "R" << R << "T" << T << "E" << E << "F" << F;
	}	
	else{
		cout << "Error : can not save the extrinsic parameters" << endl;
	}
	
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];
	
	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);

	

	
	if(fs.isOpened()){
		fs << "R1" << R1 << "R2" << R2 << "P1" << "P2" << "Q" << Q;
		fs.release();
	}	
	else{
		cout << "Error : can not save the extrinsic parameters" << endl;
	}
	

	
	
	
	return 0;
}

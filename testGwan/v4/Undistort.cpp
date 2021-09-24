#include <vector>
#include <string>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>


using std::cout;
using std::cin;
using std::endl;

class Undistort{
	private:
		cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;
		cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;
		cv::Size imgsize;
		cv::Mat R, T;
		cv::Mat frame;
		cv::Mat lrImage[2];

		vector<cv::String> normalimages;
		cv::Mat undistortnormalImage[2]; // normal image's left and right
		char buf[256];
	pubilc:
		bool doUndistortion(const char* xmlName, const char* pPath){

			cv::FileStorage fs(xmlName, cv::FileStorage::READ);
			if(fs.isOpened()){
				
				
				fs["CameraMatrix_LEFT"] >> cameraMatrix_left;
				fs["DistCoefficient_LEFT"] >> distCoeffs_left;
				fs["CameraMatrix_RIGHT"] >> cameraMatrix_right;
				fs["DistCoefficient_RIGHT"] >> distCoeffs_right;
				fs["RotationMatrix"] >> R;
				fs["TranslationVector"] >> T;
				
			}	
			else{
				cout << "Error : can not save the extrinsic parameters" << endl;
			}
			
			cout<< cameraMatrix_left << "\n\n";
			cout<< cameraMatrix_right << "\n\n";
			cout<< distCoeffs_left << "\n\n";
			cout<< distCoeffs_right << "\n\n";
			cout<< R << "\n\n";
			cout<< T << "\n\n";
			

			cv::glob(pPath, normalimages);
			
			
			for (int i = 0; i < normalimages.size(); i++){
				frame = cv::imread(normalimages[i]);
				lrImage[0] = frame(cv::Range::all(), cv::Range(0, frame.cols/2));
				lrImage[1] = frame(cv::Range::all(), cv::Range(frame.cols/2, frame.cols));
				undistort(lrImage[0], undistortnormalImage[0], cameraMatrix_left ,distCoeffs_left);
				undistort(lrImage[1], undistortnormalImage[1], cameraMatrix_left ,distCoeffs_left);
				cv::imshow("image1", undistortnormalImage[0]);
				cv::imshow("image2", undistortnormalImage[1]);
				
				
				
				cv::waitKey(0);
				sprintf(buf, "%d_Limage.jpg", i);
				cv::imwrite(buf, undistortnormalImage[0]);
				sprintf(buf, "%d_Rimage.jpg", i);
				cv::imwrite(buf, undistortnormalImage[1]);
				
			}
				
			fs.release();

			}
}



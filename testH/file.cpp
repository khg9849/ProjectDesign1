#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

int main(){
	FileStorage fsr("calibration.xml", cv::FileStorage::READ);
	ofstream fp;

	Mat distCoeff_left, cameraMatrix_left;
	Mat distCoeff_right, cameraMatrix_right;
	Mat T;

	fsr["translationVector"] >> T;
	fsr["leftCameraMatrix"] >> cameraMatrix_left;
	fsr["rightCameraMatrix"] >> cameraMatrix_right;

	fp.open("depthfile.bin", ios::binary);

	int temp;
 
	Mat l_rev, r_rev;

	l_rev = cameraMatrix_left.inv();
	r_rev = cameraMatrix_right.inv();

	ofstream ofp;
	ofp.open("a.txt");
	for(int left = 1; left < 1280; left++){
		for(int right = 0; right < left; right++){
			temp = -T.at<double>(0,0)/(l_rev.at<double>(0,0)*left+l_rev.at<double>(0,2)-r_rev.at<double>(0,0)*right-r_rev.at<double>(0,2));
			
			ofp<<temp<<'\n';
			fp.write((char*)&temp, sizeof(int));
		}
	}

	return 0;
}

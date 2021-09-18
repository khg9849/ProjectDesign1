#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <vector>
using namespace std;

int main(){
	
	cv::FileStorage fsr("calibrationData_90.xml", cv::FileStorage::READ);

	cv::Mat distCoeff_left, cameraMatrix_left;
	cv::Mat distCoeff_right, cameraMatrix_right;

	fsr["leftCameraDistCoeff"] >> distCoeff_left;
	fsr["leftCameraMatrix"] >> cameraMatrix_left;
	fsr["rightCameraDistCoeff"] >> distCoeff_right;
	fsr["rightCameraMatrix"] >> cameraMatrix_right;

	string path = "../s1625714490-90/s1625714490-90";
	vector<cv::String> images;
	cv::glob(path, images);

	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher_orb = cv::BFMatcher::create(cv::NORM_HAMMING);

	cv::Mat img, gray, dis_pic[2], pic[2], dc[2], output;
	vector<cv::KeyPoint> kp[2];
	vector<vector<cv::DMatch>> match;
	
	for(int i = 0; i < 5; i++){
		img = cv::imread(images[i]);
		cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

		dis_pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		cv::undistort(dis_pic[0], pic[0], cameraMatrix_left, distCoeff_left);
		dis_pic[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		cv::undistort(dis_pic[1], pic[1], cameraMatrix_right, distCoeff_right);

		
		for(int j = 0; j < 2; j++){
			fast->detect(pic[j], kp[j]);
			brief->compute(pic[j], kp[j], dc[j]);
		}
		matcher_orb->radiusMatch(dc[0], dc[1], match, 15);
		cv::drawMatches(pic[0], kp[0], pic[1], kp[1], match, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<vector<char>>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		cv::imshow("test", output);
		cv::waitKey(0);
	}

	return 0;
}

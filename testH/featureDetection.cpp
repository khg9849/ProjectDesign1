#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <vector>
using namespace std;

int main(){
	
	cv::FileStorage fsr("calibration.xml", cv::FileStorage::READ);

	cv::Mat distCoeff_left, cameraMatrix_left;
	cv::Mat distCoeff_right, cameraMatrix_right;

	fsr["leftCameraDistCoeff"] >> distCoeff_left;
	fsr["leftCameraMatrix"] >> cameraMatrix_left;
	cout<<cameraMatrix_left<<'\n';
	fsr["rightCameraDistCoeff"] >> distCoeff_right;
	fsr["rightCameraMatrix"] >> cameraMatrix_right;
	cout<<cameraMatrix_right<<'\n';

	string path = "../resources/cube/50cm";
	vector<cv::String> images;
	cv::glob(path, images);

	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher_orb = cv::BFMatcher::create(cv::NORM_HAMMING);
	cv::Ptr<cv::Feature2D> orb = cv::ORB::create();
	
	cv::Mat img, gray, dis_pic[2], pic[2], dc[2], output, dc1[2];
	vector<cv::KeyPoint> kp[2], kp1[2];
	vector<vector<cv::DMatch>> match1, match2;
	
	for(int i = 0; i < images.size(); i++){
		img = cv::imread(images[i]);
		cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

		cout<<gray.rows<<'\n'<<gray.cols<<'\n';
		dis_pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		cv::undistort(dis_pic[0], pic[0], cameraMatrix_left, distCoeff_left);
		dis_pic[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		cv::undistort(dis_pic[1], pic[1], cameraMatrix_right, distCoeff_right);

		
		for(int j = 0; j < 2; j++){
			fast->detect(pic[j], kp[j]);
			brief->compute(pic[j], kp[j], dc[j]);
			orb->detectAndCompute(pic[j], cv::Mat(), kp1[j], dc1[j]);
		}
		matcher_orb->radiusMatch(dc[0], dc[1], match1, 10);
		matcher_orb->radiusMatch(dc1[0], dc1[1], match2, 30);

		//cv::drawMatches(pic[0], kp[0], pic[1], kp[1], match1, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<vector<char>>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		//cv::imshow("1", output);
		//cv::drawMatches(pic[0], kp1[0], pic[1], kp1[1], match2, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<vector<char>>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		/*
		if(match1.size()){
			for(int k = 0; k < match2.size(); k++){
				if(match2.at(k).size()){
					cv::drawMatches(pic[0], kp1[0], pic[1], kp1[1], match2.at(k), output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
					cout<<match2.at(k).at(0).queryIdx<<' '<<match2.at(k).at(0).trainIdx<<'\n';
					//cout<<match1.at(k).size()<<'\n';
					//cout<<match1.at(k).at(0).queryIdx<<'\n';
					//break;
					cv::imshow("1", output);
					cv::waitKey(0);
				}
			}
		}*/
		//	if(match1.at(0).size()){
		//		cout<<match1.at(0).at(0).queryIdx << '\n';
		//	}
		cv::Point2f a = kp1[0].at(72).pt, b = kp1[1].at(143).pt;
		cout<<"a.x : " << a.x << '\n' << "a.y : " << a.y << '\n';
		cout<<"b.x : " << b.x << '\n' << "b.y : " << b.y << '\n';

		//cv::imshow("test", output);
		//cv::waitKey(0);
	}

	return 0;
}

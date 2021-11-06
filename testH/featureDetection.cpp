#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <vector>
#include <fstream>
//#include <darknet.h>
using namespace std;
using namespace cv::dnn::dnn4_v20210608;

int main(){
	
	cv::FileStorage fsr("calibration.xml", cv::FileStorage::READ);
	
	cv::Mat distCoeff_left, cameraMatrix_left;
	cv::Mat distCoeff_right, cameraMatrix_right;

	cv::Mat T;
	fsr["translationVector"] >> T;
	fsr["leftCameraDistCoeff"] >> distCoeff_left;
	fsr["leftCameraMatrix"] >> cameraMatrix_left;
	fsr["rightCameraDistCoeff"] >> distCoeff_right;
	fsr["rightCameraMatrix"] >> cameraMatrix_right;

	ofstream fp;
	//fp.open("40cm_right.txt");

	string path = "../resources/a";
	vector<cv::String> images;
	cv::glob(path, images);

	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);
	
	cv::Mat img, gray, pic[2], dc[2], output;
	vector<cv::KeyPoint> kp[2];
	vector<cv::DMatch> matches;

	cv::Mat mat1, mat2;
	cv::Mat mat3(960, 1280, CV_8UC1);
	mat1 = cameraMatrix_left.inv();
	mat2 = cameraMatrix_right.inv();

	cv::Ptr<cv::StereoMatcher> matcher1 = cv::StereoBM::create(128);
	cv::Mat mat5;

	for(int i = 0; i < images.size(); i++){
		img = cv::imread(images[i]);
		cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
		//cout<<img.type()<<'\n';

		pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		pic[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		//cout<<pic[0].at<uchar>(100,100)<<'\n';
		//cout<<pic[0].type()<<'\n';
		//cout<<mat3.type()<<'\n';
		matcher1->compute(pic[0], pic[1], mat5);
		cv::imshow("1", mat5);
		cv::waitKey(0);

		for(int j = 0; j < 2; j++){
			fast->detect(pic[j], kp[j]);
			brief->compute(pic[j], kp[j], dc[j]);
		}
		matcher->match(dc[0], dc[1], matches);

		vector<cv::DMatch> ma;
		
		for(int k = 0; k < matches.size(); k++){
			//if(kp[1][matches[k].trainIdx].pt.x < kp[0][matches[k].queryIdx].pt.x){
			
				//mat3.at<uchar>(kp[1][matches[k].trainIdx].pt.y, kp[1][matches[k].trainIdx].pt.x) = 255;
					mat3.at<uchar>(kp[1][matches[k].trainIdx].pt.y, kp[1][matches[k].trainIdx].pt.x) = T.at<double>(0,0)/(-mat2.at<double>(0,0)*(int)(kp[1][matches[k].trainIdx].pt.x)-mat2.at<double>(0,2)+mat1.at<double>(0,0)*(int)(kp[0][matches[k].queryIdx].pt.x)+mat1.at<double>(0,2))/2650*255+255;
				ma.push_back(matches[k]);
			//}
			/*
			cout<<matches[k].queryIdx<<' '<<matches[k].trainIdx<<'\n';
			vector<cv::DMatch> ma;
			ma.push_back(matches[k]);
			cv::drawMatches(pic[0], kp[0], pic[1], kp[1], ma, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			cv::imshow("test", output);
			cv::waitKey(0);
			*/
		}
		cv::drawMatches(pic[0], kp[0], pic[1], kp[1], ma, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		//cout<<-T.at<double>(0,0)/(mat1.at<double>(0,0)*(int)(1279)+mat1.at<double>(0,2)-mat2.at<double>(0,0)*(int)(1278)-mat2.at<double>(0,2))<<'\n';
		cv::imshow("test", mat3);
		cv::waitKey(0);
	}

	return 0;
}

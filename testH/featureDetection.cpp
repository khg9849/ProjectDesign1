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

	string path = "../resources/cube/40cm";
	vector<cv::String> images;
	cv::glob(path, images);

	cv::Ptr<cv::Feature2D> orb = cv::ORB::create();
	cv::Ptr<cv::DescriptorMatcher> matcher_orb = cv::BFMatcher::create(cv::NORM_HAMMING);
	
	cv::Mat img, gray, pic[2], dc[2], output;
	vector<cv::KeyPoint> kp[2];
	vector<vector<cv::DMatch>> matches;

	cv::Mat mat1, mat2;
	mat1 = cameraMatrix_left.inv();
	mat2 = cameraMatrix_right.inv();
	
	for(int i = 0; i < images.size(); i++){
		img = cv::imread(images[i]);
		cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

		Net net = readNetFromDarknet("darknet/cfg/yolov4.cfg", "darknet/yolov4.weights");
		net.setPreferableBackend(DNN_BACKEND_OPENCV);
		net.setPreferableTarget(DNN_TARGET_CPU);
		string outputFile = "yolo_out_cpp.avi";
		string str = parser.get<String>(img);
		ifstream ifile(str);
		//Detector yoloDetector("./darknet/cfg/yolov4.cfg", "./darknet/yolov4.weights");
		//yoloDetector.detect(img, 0.7);

		pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		pic[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		
		for(int j = 0; j < 2; j++){
			orb->detectAndCompute(pic[j], cv::Mat(), kp[j], dc[j]);
		}
		matcher_orb->radiusMatch(dc[0], dc[1], matches, 25);
		/*
		if(matches.size()){
			for(int k = 0; k < matches.size(); k++){
				for(int m = 0; m < matches.at(k).size(); m++){
					vector<cv::DMatch> temp;
					temp.push_back(matches.at(k).at(m));
					cv::drawMatches(pic[0], kp[0], pic[1], kp[1], temp, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
					cout<<"pixel difference : " << (int)kp[0].at(temp.at(0).queryIdx).pt.x-(int)kp[1].at(temp.at(0).trainIdx).pt.x << ", z value : ";
					
					cout<< -T.at<double>(0,0)/(-mat2.at<double>(0,0)*(int)(kp[1].at(temp.at(0).trainIdx).pt.x)-mat2.at<double>(0,2)+mat1.at<double>(0,0)*(int)(kp[0].at(temp.at(0).queryIdx).pt.x)+mat1.at<double>(0,2))<<'\n';

					cv::imshow("test", output);
					cv::waitKey(0);
				}
			}
		}*/
	}

	return 0;
}

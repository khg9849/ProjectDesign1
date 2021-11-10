#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
using namespace std;
using namespace cv;

int main(int argc, char** argv){

	FileStorage fsr("calibration.xml", cv::FileStorage::READ);
	
	Mat distCoeff_left, cameraMatrix_left;
	Mat distCoeff_right, cameraMatrix_right;
	Mat T;

	fsr["translationVector"] >> T;
	fsr["leftCameraDistCoeff"] >> distCoeff_left;
	fsr["leftCameraMatrix"] >> cameraMatrix_left;
	fsr["rightCameraDistCoeff"] >> distCoeff_right;
	fsr["rightCameraMatrix"] >> cameraMatrix_right;

	Ptr<Feature2D> fast = FastFeatureDetector::create();
	Ptr<Feature2D> brief = xfeatures2d::BriefDescriptorExtractor::create();
	Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING, true);
	
	Mat img, gray, pic[2], dc[2], output;
	vector<KeyPoint> kp[2];
	vector<DMatch> matches;

	Mat mat1, mat2;
	mat1 = cameraMatrix_left.inv();
	mat2 = cameraMatrix_right.inv();

	img = cv::imread(argv[1]);
	cvtColor(img, gray, cv::COLOR_RGB2GRAY);
	
	pic[0] = gray(Range(127, 520), Range(151, 520));
	pic[1] = gray(Range(127, 520), Range(1431, 1800));

	ifstream fp;
	fp.open("depthfile.bin", ios::binary);

	clock_t a, b;

	int sum = 0;

	int arr[1280];
	//arr = new int[1280];
	for(int i = 0; i < 1280; i++){
		arr[i] = 100;
	}

	for(int t = 0; t < 10000; t++){

	fast->detect(pic[0], kp[0], pic[1]);
	brief->compute(pic[0], kp[0], dc[0]);
	fast->detect(pic[1], kp[1], pic[0]);
	brief->compute(pic[1], kp[1], dc[1]);
	matcher->match(dc[0], dc[1], matches);
/*
	int depth;
	a = clock();
	for(int k = 0; k < matches.size(); k++){
		depth = -T.at<double>(0,0)/(mat2.at<double>(0,0)*(int)(kp[1][matches[k].trainIdx].pt.x)+mat2.at<double>(0,2)-mat1.at<double>(0,0)*(int)(kp[0][matches[k].queryIdx].pt.x)-mat1.at<double>(0,2));
	}

	b = clock();
*/	
	int depth;

	a = clock();
	for(int k = 0; k < matches.size(); k++){
		depth = arr[(int)kp[1][matches[k].trainIdx].pt.x-(int)kp[0][matches[k].queryIdx].pt.x];
	}

	b = clock();

	sum += (b-a);
	}

	cout<<(double)sum/10000<<'\n';
//	cout<<CLOCKS_PER_SEC<<'\n';

	//cv::imwrite("output_one.jpg", mat3);
	//cv::imshow("test", mat3);
	//cv::waitKey(0);

	return 0;
}

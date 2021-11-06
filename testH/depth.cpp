#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <vector>
using namespace std;

int main(int argc, char** argv){
	
	if(argc!=2){
		cout<<"./depth jpgfilepath/*.jpg\n";
		
		return 1;
	}

	cv::FileStorage fsr("calibration.xml", cv::FileStorage::READ);
	
	cv::Mat distCoeff_left, cameraMatrix_left;
	cv::Mat distCoeff_right, cameraMatrix_right;
	cv::Mat T;

	fsr["translationVector"] >> T;
	fsr["leftCameraDistCoeff"] >> distCoeff_left;
	fsr["leftCameraMatrix"] >> cameraMatrix_left;
	fsr["rightCameraDistCoeff"] >> distCoeff_right;
	fsr["rightCameraMatrix"] >> cameraMatrix_right;

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

	img = cv::imread(argv[1]);
	cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
	
	pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
	pic[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));

	fast->detect(pic[0], kp[0], pic[1]);
	brief->compute(pic[0], kp[0], dc[0]);
	fast->detect(pic[1], kp[1], pic[0]);
	brief->compute(pic[1], kp[1], dc[1]);
	matcher->match(dc[0], dc[1], matches);

	cv::drawMatches(pic[0], kp[0], pic[1], kp[1], matches, output, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	cv::imshow("1", output);
	cv::waitKey(0);

	for(int k = 0; k < matches.size(); k++){
		//cout<<kp[1][matches[k].trainIdx].pt.x<<' '<<kp[0][matches[k].queryIdx].pt.x<<'\n';
		if(kp[1][matches[k].trainIdx].pt.x > kp[0][matches[k].queryIdx].pt.x){
			int col_r = kp[1][matches[k].trainIdx].pt.x;
			int row_r = kp[1][matches[k].trainIdx].pt.y;
		/*mat3.at<uchar>(row_r-1, col_r) = 255;
			mat3.at<uchar>(row_r+1, col_r) = 255;
			mat3.at<uchar>(row_r, col_r-1) = 255;
			mat3.at<uchar>(row_r, col_r+1) = 255;
			mat3.at<uchar>(row_r+2, col_r) = 255;
			mat3.at<uchar>(row_r-2, col_r) = 255;
			mat3.at<uchar>(row_r, col_r+2) = 255;
			mat3.at<uchar>(row_r, col_r-2) = 255;
			mat3.at<uchar>(row_r+1, col_r+1) = 255;
			mat3.at<uchar>(row_r-1, col_r-1) = 255;
			mat3.at<uchar>(row_r+1, col_r-1) = 255;
			mat3.at<uchar>(row_r-1, col_r+1) = 255;
			mat3.at<uchar>(kp[1][matches[k].trainIdx].pt.y, kp[1][matches[k].trainIdx].pt.x) = */
			int temp = 255;//T.at<double>(0,0)/(-mat2.at<double>(0,0)*(int)(kp[1][matches[k].trainIdx].pt.x)-mat2.at<double>(0,2)+mat1.at<double>(0,0)*(int)(kp[0][matches[k].queryIdx].pt.x)+mat1.at<double>(0,2))/2650*255+255;
			/*mat3.at<uchar>(row_r-1, col_r) = temp;
			mat3.at<uchar>(row_r+1, col_r) = temp;
			mat3.at<uchar>(row_r, col_r-1) = temp;
			mat3.at<uchar>(row_r, col_r+1) = temp;
			mat3.at<uchar>(row_r+2, col_r) = temp;
			mat3.at<uchar>(row_r-2, col_r) = temp;
			mat3.at<uchar>(row_r, col_r+2) = temp;
			mat3.at<uchar>(row_r, col_r-2) = temp;
			mat3.at<uchar>(row_r+1, col_r+1) = temp;
			mat3.at<uchar>(row_r-1, col_r-1) = temp;
			mat3.at<uchar>(row_r+1, col_r-1) = temp;
			mat3.at<uchar>(row_r-1, col_r+1) = temp;*/
			mat3.at<uchar>(kp[1][matches[k].trainIdx].pt.y, kp[1][matches[k].trainIdx].pt.x) = temp;
		}		
	}

	cv::imwrite("output_one.jpg", mat3);
	cv::imshow("test", mat3);
	cv::waitKey(0);

	return 0;
}

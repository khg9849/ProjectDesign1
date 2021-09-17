#pragma warning(disable:4996);
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>
//#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main()
{
	Mat src;
	
	src = imread("./2_Image.jpg");
	if(src.empty()){
		cerr << "Image load failed!" << endl;
		return 0;
	}
	
	//Mat gray;
	//cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
	
	//ref resrc : wiserloner.tistory.com/879
	
	vector<KeyPoint> keypoints;
	//void FAST(InputArray image, std::vector<KeyPoint>& keypoints, int threshold, bool nonmaxSuppression = true);
	FAST(src, keypoints, 300, true);
	
	Mat dst;
	cvtColor(src,dst, COLOR_BGR2GRAY);
	
	for(KeyPoint kp : keypoints){
		Point pt(cvRound(kp.pt.x), cvRound(kp.pt.y));
		circle(dst, pt, 5, Scalar(0,0,255), 2);
	}
	
	imshow("src", src);
	imshow("dst", dst);
	
	waitKey(0);
	destroyAllWindows();
	
	
	return 0;
}

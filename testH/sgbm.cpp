#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/ximgproc.hpp>
#include <fstream>
#include <iostream>
#include <vector>
using namespace std;
using namespace cv;

int main(int argc, char** argv){
/*
	if(argc!=2){
		cout<<"./prog file\n";

		return 1;
	}
*/
	ifstream fp;
	fp.open("data");
	
	int a[10];
	for(int i = 0; i < 10; i++){
		fp>>a[i];
	}

	Ptr<StereoSGBM> st_matcher = StereoSGBM::create(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], StereoSGBM::MODE_HH);
	Ptr<StereoMatcher> st_matcher1 = ximgproc::createRightMatcher(st_matcher);// = StereoSGBM::create();

	Mat img = imread(argv[1]);

	Mat left1  = img(Range::all(), Range(0, img.cols/2));
	cout<<left1.type();
	imwrite("aloeL.jpg", left1);
    Mat right1 = img(Range::all(), Range(img.cols/2, img.cols));
    
	Mat left, right;
	cvtColor(left1, left, COLOR_BGR2GRAY);
	cvtColor(right1, right, COLOR_BGR2GRAY);

	Mat left_re, right_re;
	st_matcher->compute(left, right, left_re);
//	st_matcher1->compute(right, left, right_re);

	     //   wls_filter->setLambda(lambda);
        //wls_filter->setSigmaColor(sigma);

		Mat res;

        
		ximgproc::getDisparityVis(left_re,res);

	imshow("left", res);
	waitKey(0);

	return 0;
}

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include "yeStereoCamera.hpp"
#define OPENCV

using namespace std;
using namespace cv;
using namespace SYE;

/*
./depthTest ../resources/cake1/a.jpg
*/
int main(int argc, char** argv)
{
    FileStorage fs("calibration.xml", cv::FileStorage::READ);
	
	Mat matCamMat1, matDistCoffs1;
	Mat matCamMat2, matDistCoffs2;
	Mat matR,matT;

	fs["matCamMat1"] >> matCamMat1;
    fs["matDistCoffs1"] >> matDistCoffs1;
    fs["matCamMat2"] >> matCamMat2;
    fs["matDistCoffs2"] >> matDistCoffs2;
    fs["matR"] >> matR;
    fs["matT"] >> matT;


	Ptr<Feature2D> fast = FastFeatureDetector::create();
	Ptr<Feature2D> brief = xfeatures2d::BriefDescriptorExtractor::create();
	Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING, true);
	
	Mat mat, gray, pic[2], dc[2], output;
	vector<KeyPoint> kp[2];
	vector<DMatch> matches;

    //initMatrix
	fast = cv::FastFeatureDetector::create();
	brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);


    //getAbsoluteLength 
    vector<YePos3D> features;
    vector<bbox_t> pos;

    mat=cv::imread(argv[1]);
    // imshow("Mat",mat);
    // waitKey(0);
    cvtColor(mat, gray, cv::COLOR_RGB2GRAY);
    pic[0] = gray(cv::Range::all(), cv::Range(0, gray.cols / 2));
	pic[1] = gray(cv::Range::all(), cv::Range(gray.cols / 2, gray.cols));

    double baseline = matT.at<double>(0)/10;  //Distance between the cameras [cm]
	double f =  matCamMat1.at<double>(0,0); //focal length [mm]
	double FOV = 83; //화각
	double PI = 3.14159265358979;
    double f_pixel = (mat.cols*0.5*0.5)/ tan(FOV * 0.5 * PI/180);
    //double f_pixel =  matCamMat1.at<double>(0,0); //focal length [px]

    fast->detect(pic[0], kp[0], pic[1]);
    brief->compute(pic[0], kp[0], dc[0]);
    fast->detect(pic[1], kp[1], pic[0]);
    brief->compute(pic[1], kp[1], dc[1]);
    matcher->match(dc[0], dc[1], matches);

    bool findPos = false;
    
    YePos3D temp[2];
    for(int j = 0; j < matches.size() && !findPos; j++){
        int x1=(int)kp[0][matches[j].queryIdx].pt.x;
        int y1=(int)kp[0][matches[j].queryIdx].pt.y;
        int x2=(int)kp[1][matches[j].trainIdx].pt.x;
        int y2=(int)kp[1][matches[j].trainIdx].pt.y;

        int dif=x1-x2;
        double zDepth =fabs((baseline*f_pixel)/dif);

        if(abs(y1-y2)<50&&x1>=200&&y1>=600&&y2<=800){
            printf("point1:(%d,%d),point2:(%d,%d), dif is %d, zDepth is %lf\n",x1,y1,x2,y2,dif, zDepth);
            
            cv::Mat res=mat.clone();
            cv::Point point1,point2;
            point2=cv::Point(x1, y1);
            point1=cv::Point(x2+mat.cols/2,y2);

            cv::line(res,point1,point2,cv::Scalar(0, 255, 0),5,1);
            cv::line(res,point1,point1,cv::Scalar(0, 0, 255),5,3);
            cv::line(res,point2,point2,cv::Scalar(0, 0, 255),5,3);

            imshow("res",res);
            waitKey(0);
        }
        
    
        temp[0].z=zDepth;
        features.push_back(temp[0]);

        bbox_t posTemp;
        posTemp.x = (int)kp[0][matches[j].queryIdx].pt.x;
        posTemp.y = (int)kp[0][matches[j].trainIdx].pt.y;

        pos.push_back(posTemp);
		}


    return 0;
}


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"
#include <opencv2/videoio.hpp>
#define OPENCV

#define DEBUG

using namespace SYE;
using namespace std;

void initTrackWindow();
void onChange(int pos, void* userdata);
sgbmParam param;

int main()
{
    YeStereoCamera *temp = new YeStereoCamera();
    const char *filepath = "../resources/calib_data";
    const char *objName = "cup";

    if (!temp->initCalibData("calibration.xml")) {
        if ((temp->doCalibration(filepath, "calibration.xml")) == false) {
            std::cout << "doCalibration failed\n";
            exit(1);
        }
    }

    cv::Mat mat;
    
	cv::VideoCapture video("../resources/home1.mp4");

    if (!video.isOpened()) {
        cout << "Can't open the video" << endl;
        return 0;
    }

	temp->getcfg_file("../darknet/cfg/yolov4.cfg");
    temp->getWeight_file("../darknet/yolov4.weights");
    // temp->getcfg_file("../darknet/cfg/yolov4-tiny.cfg");
    // temp->getWeight_file("../darknet/yolov4-tiny.weights");
	temp->getObjNames_file("../darknet/data/coco.names");
	temp->initMatrix();
	
    // if intitTrackbar() makes error, comment out it and uncomment section below.
    initTrackWindow();
    /* section
    param.wsize=1;
    param.max_disp=3*16;
    param.preFilterCap=27;
    param.lambda=20000;
    param.sigma=38/10.0;
    param.vis_mult=20/10.0;
    */ 

    double start, end, sum = 0;
    cv::namedWindow("window",CV_WINDOW_NORMAL);
    bool first=true;
	video>>mat;
    int i=0;
    cv::Mat rtn;

	while(!mat.empty()){
        start = cv::getTickCount();

        vector<bbox_t> pObject;
        if ((temp->findImage(mat, objName, pObject)) == false) {
            std::cout << "findImage failed\n";
            video>>mat;
            continue;
        }
        if(pObject.size()==0){
            std::cout <<"pObject.size() is 0\n";
            exit(1);
        }
#ifdef DEBUG
        for (int i = 0; i < pObject.size(); i++) {
            std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
        }
#endif
		if(pObject.size()){
        	
  	    	if ((temp->getSgbmInRect(mat, pObject[0], rtn, param)) == false) {
    	    	std::cout << "getSgbmInRect failed\n";
            	exit(1);
        	}
       

        	YePos3D feature;
        	bbox_t pos;
         	if ((temp->getAbsoluteLengthInRect(mat, pObject[0], feature, pos)) == false) {
            	std::cout << "getAbsoluteLengthInRect failed\n";
            	exit(1);
        	}
            temp->showResult(mat,rtn,pObject[0],feature,pos);
        }
        cv::putText(rtn, std::to_string(i++), cv::Point(100,100), 1, 5, cv::Scalar(0, 0, 255), 3, 8);
        	       
        end = cv::getTickCount();
        double temp;
        temp = end-start;
        cv::imshow("result",rtn);
        if(!first){
            printf("%d\t%lf\n",i-1,temp/cv::getTickFrequency());
            cv::waitKey(0);
            sum += (temp);
       
        }
        else{   
            cv::waitKey(0);
            first=false;
        }  
        video>>mat;
    }
    printf("sum\t%lf\n",sum/cv::getTickFrequency());
    return 0;
}

void initTrackWindow(){
    cv::destroyAllWindows();
    cv::namedWindow("window");

    cv::createTrackbar("wsize","window",0,20,onChange);
    cv::createTrackbar("max_disp","window",0,20,onChange);
    cv::createTrackbar("preFilterCap","window",0,100,onChange);
    cv::createTrackbar("lambda","window",0,20000,onChange);
    cv::createTrackbar("sigma","window",0,200,onChange);
    cv::createTrackbar("vis_mult","window",0,20,onChange);
    
    cv::createTrackbar("minDisparity","window",0,100,onChange);
    cv::createTrackbar("disp12MaxDiff","window",0,1000000,onChange);
    cv::createTrackbar("uniquenessRatio","window",0,100,onChange);
    cv::createTrackbar("speckleWindowSize","window",0,100,onChange);

     cv::setTrackbarPos("wsize","window",1);
     cv::setTrackbarPos("max_disp","window",3);
     cv::setTrackbarPos("preFilterCap","window",27);
     cv::setTrackbarPos("lambda","window",20000);
    cv::setTrackbarPos("sigma","window",38);
    cv::setTrackbarPos("vis_mult","window",20);
      
    cv::setTrackbarPos("minDisparity","window",0);
    cv::setTrackbarPos("disp12MaxDiff","window",2);
    cv::setTrackbarPos("uniquenessRatio","window",5);
    cv::setTrackbarPos("speckleWindowSize","window",5);

    param.wsize=cv::getTrackbarPos("wsize","window");
    param.max_disp=cv::getTrackbarPos("max_disp","window")*16;
    param.preFilterCap=cv::getTrackbarPos("preFilterCap","window");
     param.lambda=cv::getTrackbarPos("lambda","window");
    param.sigma=cv::getTrackbarPos("sigma","window")/10.0;
    param.vis_mult=cv::getTrackbarPos("vis_mult","window")/10.0;
    
    param.minDisparity=cv::getTrackbarPos("minDisparity","window");
    param.disp12MaxDiff=cv::getTrackbarPos("disp12MaxDiff","window");
    param.uniquenessRatio=cv::getTrackbarPos("uniquenessRatio","window");
    param.speckleWindowSize=cv::getTrackbarPos("speckleWindowSize","window");

}

void onChange(int pos, void* userdata){
    param.wsize=cv::getTrackbarPos("wsize","window");
    param.max_disp=cv::getTrackbarPos("max_disp","window")*16;
    param.preFilterCap=cv::getTrackbarPos("preFilterCap","window");
    param.lambda=cv::getTrackbarPos("lambda","window");
    param.sigma=cv::getTrackbarPos("sigma","window")/10.0;
    param.vis_mult=cv::getTrackbarPos("vis_mult","window")/10.0;

    param.minDisparity=cv::getTrackbarPos("minDisparity","window");
    param.disp12MaxDiff=cv::getTrackbarPos("disp12MaxDiff","window");
    param.uniquenessRatio=cv::getTrackbarPos("uniquenessRatio","window");
    param.speckleWindowSize=cv::getTrackbarPos("speckleWindowSize","window");
}

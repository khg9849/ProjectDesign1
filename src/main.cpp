#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"
#include <opencv2/videoio.hpp>
#define OPENCV

using namespace SYE;
using namespace std;

/*how to useage

    ./yeStereoCamera 

*/

void initTrackWindow();
void onChange(int pos, void* userdata);
sgbmParam param;

int main()
{
    YeStereoCamera *temp = new YeStereoCamera();
    const char *filepath = "../resources/calib_data";
    const char *objName = "clock";


    if (!temp->initCalibData("calibration.xml")) {
        if ((temp->doCalibration(filepath, "calibration.xml")) == false) {
            std::cout << "doCalibration failed\n";
            exit(1);
        }
    }

    cv::Mat mat;
    
	cv::VideoCapture video("../resources/1/sample.mp4");
    
    if (!video.isOpened()) {
        cout << "Can't open the video" << endl;
        return 0;
    }

	temp->getcfg_file("../darknet/cfg/yolov4-tiny.cfg");
    temp->getWeight_file("../darknet/yolov4-tiny.weights");
	temp->getObjNames_file("../darknet/data/coco.names");
	temp->initMatrix();

	video>>mat;
	while(!mat.empty()){
        vector<bbox_t> pObject;
        if ((temp->findImage(mat, objName, pObject)) == false) {
            std::cout << "findImage failed\n";
            exit(1);
        }

        for (int i = 0; i < pObject.size(); i++) {
            std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
        }

		if(pObject.size()){
        	cv::Mat rtn;
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
       
			video>>mat;
		}
    }

    return 0;
}

void initTrackWindow(){
    cv::namedWindow("window");
    cv::createTrackbar("vis_mult","window",0,100,onChange);
    cv::createTrackbar("max_disp","window",0,100,onChange);
    cv::createTrackbar("lambda","window",0,10000,onChange);
    cv::createTrackbar("sigma","window",0,20,onChange);
    cv::createTrackbar("wsize","window",0,20,onChange);
    cv::createTrackbar("minDisparity","window",0,100,onChange);
    cv::createTrackbar("disp12MaxDiff","window",0,1000000,onChange);
    cv::createTrackbar("preFilterCap","window",0,100,onChange);
    cv::createTrackbar("uniquenessRatio","window",0,100,onChange);
    cv::createTrackbar("speckleWindowSize","window",0,100,onChange);

    cv::setTrackbarPos("vis_mult","window",8);
    cv::setTrackbarPos("max_disp","window",16);
    cv::setTrackbarPos("lambda","window",8000);
    cv::setTrackbarPos("sigma","window",1.5);
    cv::setTrackbarPos("wsize","window",3);
    cv::setTrackbarPos("minDisparity","window",0);
    cv::setTrackbarPos("disp12MaxDiff","window",1000000);
    cv::setTrackbarPos("preFilterCap","window",63);
    cv::setTrackbarPos("uniquenessRatio","window",0);
    cv::setTrackbarPos("speckleWindowSize","window",0);

    param.vis_mult=cv::getTrackbarPos("vis_mult","window");
    param.max_disp=cv::getTrackbarPos("max_disp","window");
    param.lambda=cv::getTrackbarPos("lambda","window");
    param.sigma=cv::getTrackbarPos("sigma","window");
    param.wsize=cv::getTrackbarPos("wsize","window");
    param.minDisparity=cv::getTrackbarPos("minDisparity","window");
    param.disp12MaxDiff=cv::getTrackbarPos("disp12MaxDiff","window");
    param.preFilterCap=cv::getTrackbarPos("preFilterCap","window");
    param.uniquenessRatio=cv::getTrackbarPos("uniquenessRatio","window");
    param.speckleWindowSize=cv::getTrackbarPos("speckleWindowSize","window");

}

void onChange(int pos, void* userdata){
    param.vis_mult=cv::getTrackbarPos("vis_mult","window");
    param.max_disp=cv::getTrackbarPos("max_disp","window");
    param.lambda=cv::getTrackbarPos("lambda","window");
    param.sigma=cv::getTrackbarPos("sigma","window");
    param.wsize=cv::getTrackbarPos("wsize","window");
    param.minDisparity=cv::getTrackbarPos("minDisparity","window");
    param.disp12MaxDiff=cv::getTrackbarPos("disp12MaxDiff","window");
    param.preFilterCap=cv::getTrackbarPos("preFilterCap","window");
    param.uniquenessRatio=cv::getTrackbarPos("uniquenessRatio","window");
    param.speckleWindowSize=cv::getTrackbarPos("speckleWindowSize","window");
}


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"
#include <opencv2/videoio.hpp>
#define OPENCV

using namespace SYE;
using namespace std;
using namespace cv;

/*

./reconTest ../resources/1117dataset/cup.jpg cup
./reconTest ../resources/cake1/a.jpg cake
./reconTest ../resources/1115Dataset/WIN_20211115_17_09_34_Pro.jpg 
*/
void initTrackWindow();
void onChange(int pos, void* userdata);
sgbmParam param;

int main(int argc, char** argv)
{
    YeStereoCamera *temp = new YeStereoCamera();

    cv::Mat  mat=cv::imread(argv[1]);
    cv::Mat rtn;

    const char *filepath = "../resources/calib_data";
    if (!temp->initCalibData("calibration.xml")) {
        if ((temp->doCalibration(filepath, "calibration.xml")) == false) {
            std::cout << "doCalibration failed\n";
            exit(1);
        }
    }
    temp->getcfg_file("../darknet/cfg/yolov4-tiny.cfg");
    temp->getWeight_file("../darknet/yolov4-tiny.weights");
	temp->getObjNames_file("../darknet/data/coco.names");
	temp->initMatrix();

   cv::imshow("mat",mat);
    cv::waitKey(0);

    cv::Mat mat1,mat2,dist1,dist2;
    mat1=temp->getMatCamMat1();
    mat2=temp->getMatCamMat2();
    dist1=temp->getMatDistCoffs1();
    dist2=temp->getMatDistCoffs2();
    cv::Mat lrImage[2];
    cv::Mat lrImage2[2];
    lrImage[0] = mat(cv::Range::all(), cv::Range(0, mat.cols/2));
	lrImage[1] = mat(cv::Range::all(), cv::Range(mat.cols/2, mat.cols));
    undistort(lrImage[0], lrImage2[0], mat1 ,dist1);
	undistort(lrImage[1], lrImage2[1], mat2 ,dist2);
    cv::hconcat(lrImage2[0],lrImage2[1],mat);
    cv::imshow("Mat_undistorted",mat);
    cv::waitKey(0);

 

    initTrackWindow();

    vector<bbox_t> pObject;
    if ((temp->findImage(mat, argv[2], pObject)) == false) {
        std::cout << "findImage failed\n";
        exit(1);
    }

    for (int i = 0; i < pObject.size(); i++) {
        std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
    }

    while(true){
        // if ((temp->getSgbmInRect(mat, pObject[1],rtn,param)) == false) {
        //         std::cout << "getSgbmInRect failed\n";
        //         exit(1);
        //     }
 if ((temp->getSgbm(mat, rtn,param)) == false) {
                std::cout << "getSgbmInRect failed\n";
                exit(1);
            }
            cv::imshow("result", rtn);
            cv::imwrite("temp.jpg",rtn);
            cv::waitKey(0);
    }

    destroyAllWindows();
    return 0;
}

void initTrackWindow(){
    cv::namedWindow("window");
    cv::createTrackbar("vis_mult","window",0,100,onChange);
    cv::createTrackbar("max_disp","window",0,10,onChange); // *16
    cv::createTrackbar("lambda","window",0,10000,onChange);
    cv::createTrackbar("sigma","window",0,20,onChange);
    cv::createTrackbar("wsize","window",0,20,onChange); // *2-1
    cv::createTrackbar("minDisparity","window",0,100,onChange);
    cv::createTrackbar("disp12MaxDiff","window",0,1000000,onChange);
    cv::createTrackbar("preFilterCap","window",0,1000,onChange);
    cv::createTrackbar("uniquenessRatio","window",0,100,onChange);
    cv::createTrackbar("speckleWindowSize","window",0,100,onChange);

    cv::setTrackbarPos("vis_mult","window",8);
    cv::setTrackbarPos("max_disp","window",1);
    cv::setTrackbarPos("lambda","window",8000);
    cv::setTrackbarPos("sigma","window",1.5);
    cv::setTrackbarPos("wsize","window",1);
    cv::setTrackbarPos("minDisparity","window",0);
    cv::setTrackbarPos("disp12MaxDiff","window",1000000);
    cv::setTrackbarPos("preFilterCap","window",63);
    cv::setTrackbarPos("uniquenessRatio","window",0);
    cv::setTrackbarPos("speckleWindowSize","window",0);

    param.vis_mult=cv::getTrackbarPos("vis_mult","window");
    param.max_disp=cv::getTrackbarPos("max_disp","window")*16;
    param.lambda=cv::getTrackbarPos("lambda","window");
    param.sigma=cv::getTrackbarPos("sigma","window");
    param.wsize=cv::getTrackbarPos("wsize","window")*2-1;
    param.minDisparity=cv::getTrackbarPos("minDisparity","window");
    param.disp12MaxDiff=cv::getTrackbarPos("disp12MaxDiff","window");
    param.preFilterCap=cv::getTrackbarPos("preFilterCap","window");
    param.uniquenessRatio=cv::getTrackbarPos("uniquenessRatio","window");
    param.speckleWindowSize=cv::getTrackbarPos("speckleWindowSize","window");

}


void onChange(int pos, void* userdata){
    param.vis_mult=cv::getTrackbarPos("vis_mult","window");
    param.max_disp=cv::getTrackbarPos("max_disp","window")*16;
    param.lambda=cv::getTrackbarPos("lambda","window");
    param.sigma=cv::getTrackbarPos("sigma","window");
    param.wsize=cv::getTrackbarPos("wsize","window")*2-1;
    param.minDisparity=cv::getTrackbarPos("minDisparity","window");
    param.disp12MaxDiff=cv::getTrackbarPos("disp12MaxDiff","window");
    param.preFilterCap=cv::getTrackbarPos("preFilterCap","window");
    param.uniquenessRatio=cv::getTrackbarPos("uniquenessRatio","window");
    param.speckleWindowSize=cv::getTrackbarPos("speckleWindowSize","window");
}

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"
#include <opencv2/videoio.hpp>
#define OPENCV

using namespace SYE;
using namespace std;

/*

./test ../resources/1117dataset/cup.jpg cup
./test ../resources/1115Dataset/WIN_20211115_17_09_34_Pro.jpg clock
*/
int main(int argc, char** argv)
{
    cv::Mat  mat=cv::imread(argv[1]);
    cv::imshow("mat",mat);
    cv::waitKey(0);

    YeStereoCamera *temp = new YeStereoCamera();
    const char *filepath = "../resources/calib_data";
    const char *objName = argv[2];

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

    vector<bbox_t> pObject;
    if ((temp->findImage(mat, objName, pObject)) == false) {
        std::cout << "findImage failed\n";
        exit(1);
    }

    for (int i = 0; i < pObject.size(); i++) {
        std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
    }

    // fix pObject 
    bbox_t pos;
    int stride=mat.cols/2;
    bbox_t bbox1=pObject[0];
    bbox_t bbox2=pObject[1];
    pos.x=std::min(bbox1.x,bbox2.x-stride);
    pos.y=std::min(bbox1.y,bbox2.y);
    pos.w=std::max(bbox1.x+bbox1.w,bbox2.x-stride+bbox2.w)-pos.x;
    pos.h=std::max(bbox1.y+bbox1.h,bbox2.y+bbox2.h-pos.y)-pos.y;
    if(pos.x+pos.w>=stride) pos.w=stride-pos.x;
    if(pos.y+pos.h>=mat.rows) pos.h=mat.rows-pos.y;

    pObject[0]=pos;
    pObject[1]=pos;
    pObject[1].x+=stride;

    std::cout<<"fixed pObject\n";
    for (int i = 0; i < pObject.size(); i++) {
        std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
    }
    
    cv::Mat rtn;
    if ((temp->getSgbmInRect(mat, pObject, rtn,param)) == false) {
        std::cout << "getSgbmInRect failed\n";
        exit(1);
    }

    cv::imshow("result", rtn);
    cv::waitKey(0);
    
    // vector<YePos3D> feature;
    // vector<bbox_t> pos;
    //     if ((temp->getAbsoluteLengthInRect(mat, pObject, feature, pos)) == false) {
    //     std::cout << "getAbsoluteLengthInRect failed\n";
    //     exit(1);
    // }

    // for (int i = 0; i < feature.size(); i++) {
    //     std::cout << feature[i].x << ' ' << feature[i].y << ' ' << feature[i].z << '\n';
    // }

    // temp->showResult(mat,rtn,rtnPos,feature,pos);

    return 0;
}


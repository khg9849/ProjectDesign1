
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
./detectTest ../resources/cake1/a.jpg cake
./detectTest ../resources/1117dataset/cup.jpg cup
./detectTest ../resources/1115Dataset/WIN_20211115_17_09_34_Pro.jpg clock
*/
int main(int argc, char** argv)
{
    cv::Mat  mat=cv::imread(argv[1]);
    cv::imshow("mat",mat);
    cv::waitKey(0);
    char* objName = argv[2];

    YeStereoCamera *temp = new YeStereoCamera();
	temp->getcfg_file("../darknet/cfg/yolov4-tiny.cfg");
    temp->getWeight_file("../darknet/yolov4-tiny.weights");
	temp->getObjNames_file("../darknet/data/coco.names");
    const char *filepath = "../resources/calib_data";
    if (!temp->initCalibData("calibration.xml")) {
        if ((temp->doCalibration(filepath, "calibration.xml")) == false) {
            std::cout << "doCalibration failed\n";
            exit(1);
        }
    }

    vector<bbox_t> pObject;
    if ((temp->findImage(mat, objName, pObject)) == false) {
        std::cout << "findImage failed\n";
        exit(1);
    }


    for(int i=0;i<pObject.size();i++)
        cv::rectangle(mat, cv::Rect(pObject[i].x,pObject[i].y,pObject[i].w,pObject[i].h), cv::Scalar(0, 0, 255), 3, 8, 0);
	
    cv::imshow("res",mat);
    cv::waitKey(0);

    return 0;
}


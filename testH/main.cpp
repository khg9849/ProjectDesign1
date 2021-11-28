#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"
#define OPENCV

using namespace SYE;
using namespace std;

/*how to useage

    ./yeStereoCamera 

*/
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

	temp->initMatrix();
	temp->getcfg_file("../darknet/cfg/yolov4.cfg");
    temp->getWeight_file("../darknet/yolov4.weights");
	temp->getObjNames_file("../darknet/data/coco.names");

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

        vector<cv::Mat> rtn;
		vector<bbox_t> rtnPos;
        if ((temp->getSgbmInRect(mat, pObject, rtn, rtnPos)) == false) {
            std::cout << "getSgbmInRect failed\n";
            exit(1);
        }

        for (cv::Mat res : rtn) {
            cv::imshow("result", res);
            cv::waitKey(0);
        }

        vector<YePos3D> feature;
		vector<bbox_t> pos;
        if ((temp->getAbsoluteLengthInRect(mat, pObject, feature, pos)) == false) {
            std::cout << "getAbsoluteLengthInRect failed\n";
            exit(1);
        }

		temp->showResult(mat,rtn,rtnPos,feature,pos);
        //for (int i = 0; i < feature.size(); i++) {
        //    std::cout << feature[0].x << ' ' << feature[0].y << ' ' << feature[0].z << '\n';
        //}

		video>>mat;
    }

    return 0;
}


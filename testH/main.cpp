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
    const char *objName = "chair";


    if (!temp->initCalibData("calibration.xml")) {
        if ((temp->doCalibration(filepath, "calibration.xml")) == false) {
            std::cout << "doCalibration failed\n";
            exit(1);
        }
    }

    cv::Mat mat;
	mat = cv::imread("../resources/cube/new/20cm.jpg");
    
	/*cv::VideoCapture video("../resources/1/sample2.mp4");
    if (!video.isOpened()) {
        cout << "Can't open the video" << endl;
        return 0;
    }*/

	temp->getcfg_file("../darknet/cfg/yolov4.cfg");
    temp->getWeight_file("../darknet/yolov4.weights");
	temp->getObjNames_file("../darknet/data/coco.names");

	//video>>mat;
	//while(!mat.empty()){
        vector<bbox_t> pObject;
        if ((temp->findImage(mat, objName, pObject)) == false) {
            std::cout << "findImage failed\n";
            //exit(1);
        }

        for (int i = 0; i < pObject.size(); i++) {
            std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
        }

		bbox_t a, b;
		a.x=0;a.y=0;a.w=1280;a.h=960;
		b.x=1280;b.y=0;b.w=1280;b.h=960;
		pObject.push_back(a);
		pObject.push_back(b);
        vector<cv::Mat> rtn;
        if ((temp->getSgbmInRect(mat, pObject, &rtn)) == false) {
            std::cout << "getSgbmInRect failed\n";
            //exit(1);
        }

        for (cv::Mat res : rtn) {
            cv::imshow("result", res);
            cv::waitKey(0);
        }

        vector<vector<YePos3D>> feature;
        if ((temp->getAbsoluteLengthInRect(mat, pObject, feature)) == false) {
            std::cout << "getAbsoluteLengthInRect failed\n";
            //exit(1);
        }

		if(pObject.size()){
        for (int i = 0; i < feature[0].size(); i++) {
            std::cout << feature[0][i].x << ' ' << feature[0][i].y << ' ' << feature[0][i].z << '\n';
        }
		}

	//	video>>mat;
    //}

    return 0;
}

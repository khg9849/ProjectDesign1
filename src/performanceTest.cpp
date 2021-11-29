#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include "yeStereoCamera.hpp"
#include <opencv2/videoio.hpp>
#define OPENCV

using namespace SYE;
using namespace std;


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
    
	//cv::VideoCapture video("../resources/1/sample2.mp4");
    cv::VideoCapture video("../resources/WIN_20211115_17_10_04_Pro.mp4");
    if (!video.isOpened()) {
        cout << "Can't open the video" << endl;
        return 0;
    }

   temp->getcfg_file("../darknet/cfg/yolov4.cfg");
   temp->getWeight_file("../darknet/yolov4.weights");
	// temp->getcfg_file("../darknet/cfg/yolov4-tiny.cfg");
    // temp->getWeight_file("../darknet/yolov4-tiny.weights");
    temp->initMatrix();
	temp->getObjNames_file("../darknet/data/coco.names");

    double duration1=0,duration2=0;
    struct timeval bgn,end;
    double diff1,diff2;

    int cnt=0;

	video>>mat;
	while(!mat.empty()){
        
        // cv::imshow("mat",mat);
        // cv::waitKey(0);

        gettimeofday(&bgn,NULL);
       
        vector<bbox_t> pObject;
        if ((temp->findImage(mat, objName, pObject)) == false) {
            std::cout << "findImage failed\n";
            exit(1);
        }

        vector<cv::Mat> rtn;
        std::vector<bbox_t> rtnPos;
        if ((temp->getSgbmInRect(mat, pObject, rtn,rtnPos)) == false) {
            std::cout << "getSgbmInRect failed\n";
            exit(1);
        }
        gettimeofday(&end,NULL);
        diff1=(end.tv_sec+end.tv_usec/1000000.0)-(bgn.tv_sec+bgn.tv_usec/1000000.0);
        duration1+=diff1;

        for (cv::Mat res:rtn){
           // printf("rtnPos(%d,%d,%d,%d)\n",rtnPos[0].x,rtnPos[0].y,rtnPos[0].w,rtnPos[0].h);
            cv::imshow("partial",res);
            cv::waitKey(0);
        }


        vector<YePos3D> feature;
		vector<bbox_t> pos;
        // cout<<"getZ is started\n";
        if ((temp->getAbsoluteLengthInRect(mat, pObject, feature, pos)) == false) {
            std::cout << "getAbsoluteLengthInRect failed\n";
            exit(1);
        }
        // for (int i = 0; i < feature.size(); i++) {
        //     std::cout << feature[i].x << ' ' << feature[i].y << ' ' << feature[i].z << '\n';
        // }
        // cout<<"getZ is finished\n";
        // cout<<"showRes is started\n";
		temp->showResult(mat,rtn,rtnPos,feature,pos);
        // cout<<"showRes is finished\n";

        gettimeofday(&bgn,NULL);
        cv::Mat full;
        temp->getSgbm(mat,full);
        gettimeofday(&end,NULL);
        diff2=(end.tv_sec+end.tv_usec/1000000.0)-(bgn.tv_sec+bgn.tv_usec/1000000.0);
        duration2+=diff2;
        //cout<<"diff2: "<<diff<<'\n';
        imshow("full",full);
        cv::waitKey(0);
        cout<<cnt<<'\t'<<diff1<<'\t'<<diff2<<'\n';
        if(++cnt==100) break;
		video>>mat;
    }
    cout<<"duration1: "<<duration1<<'\n';
    cout<<"duration2: "<<duration2<<'\n';



    return 0;
}
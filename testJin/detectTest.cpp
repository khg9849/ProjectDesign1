
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

    vector<bbox_t> pObject;
    if ((temp->findImage(mat, objName, pObject)) == false) {
        std::cout << "findImage failed\n";
        exit(1);
    }
    if(pObject.size()<2){
        std::cout << "findImage failed\n";
        exit(1);
    }
    for (int i = 0; i < pObject.size(); i++) {
        std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
    }
    for(int i=0;i<2;i++)
        cv::rectangle(mat, cv::Rect(pObject[i].x,pObject[i].y,pObject[i].w,pObject[i].h), cv::Scalar(0, 255, 0), 3, 8, 0);
	

    // fix pObject 
    bbox_t pos;
    int stride=mat.cols/2;
    bbox_t bbox1=pObject[0];
    bbox_t bbox2=pObject[1];
    pos.x=std::min(bbox1.x,bbox2.x-stride);
    pos.y=std::min(bbox1.y,bbox2.y);
    pos.w=std::max(bbox1.x+bbox1.w,bbox2.x-stride+bbox2.w)-pos.x;
    pos.h=std::max(bbox1.y+bbox1.h,bbox2.y+bbox2.h)-pos.y;
    if(pos.x+pos.w>=stride) pos.w=stride-pos.x;
    if(pos.y+pos.h>=mat.rows) pos.h=mat.rows-pos.y;

    pObject[0]=pos;
    pObject[1]=pos;
    pObject[1].x+=stride;

    std::cout<<"fixed pObject\n";
    for (int i = 0; i < pObject.size(); i++) {
        std::cout << "x y w h id : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << "\n";
    }
    for(int i=0;i<2;i++)
        cv::rectangle(mat, cv::Rect(pObject[i].x,pObject[i].y,pObject[i].w,pObject[i].h), cv::Scalar(0, 0, 255), 3, 8, 0);
	
    cv::imshow("res",mat);
    cv::waitKey(0);

    return 0;
}


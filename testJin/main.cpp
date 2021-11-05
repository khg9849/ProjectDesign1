#include "darknet.h"
#include<opencv2/opencv.hpp>
#include<iostream>

#include "CallYolo.h"

#define CHAIR 56
#define MONITER 62

void callYolo(cv::Mat image);
void display3DReCon(cv::Mat image);

int main(int argc, char **argv){
	
	//TODO : Load image.
	//cv::Mat mat;
	//cv::imshow("test", mat);
	//

	// Call Yolo( detection )
//	std::string photopath="../resources/dataset2/135cm/WIN_20211014_15_41_13_Pro.jpg";
	std::string photopath="../resources/dataset2/90cm/WIN_20211014_15_42_09_Pro.jpg";
	
	cv::Mat res=cv::imread(photopath);

	CallYolo* yolo=new CallYolo();

	yolo->init("../darknet/cfg/yolov4.cfg", "../darknet/yolov4.weights");
	yolo->setPhoto(photopath);
	
	bbox_t_container cont;
	size_t contsize;
	cont=yolo->getCont();
	contsize=yolo->getContSize();

	cv::Mat src = res.clone();
	cv::Mat mask = cv::Mat::zeros(res.size(), CV_8U);  
	cv::Mat masked = cv::Mat::zeros(res.size(), CV_8U); 

	for(int i=0;i<contsize;i++){
		int x=cont.candidates[i].x;
		int y=cont.candidates[i].y;
		int w=cont.candidates[i].w;
		int h=cont.candidates[i].h;
		std::cout<<"obj_id is "<<cont.candidates[i].obj_id<<"\n";
		
		cv::rectangle(res,cv::Point(x,y), cv::Point(x+w,x+h),1,8,0);
		if(cont.candidates[i].obj_id==CHAIR){
			cv::rectangle(mask,cv::Rect(x,y,w,h),cv::Scalar(255),-1,8,0);
		}
			
	}

	src.copyTo(masked,mask);
	cv::imshow("result",res);
	cv::imshow("masked",masked);
	cv::waitKey(0);

	cv::imwrite("masked.jpg",masked);
//	detection = detector->detect(photopath);
	// Feature detecting & matching.

	// 3D reconstruction.


	//cv::imshow("test", mat);
	//cv::waitKey(0);


	return 0;
}

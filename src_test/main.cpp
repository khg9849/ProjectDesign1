#include "darknet.h"
#include<opencv2/opencv.hpp>
#include<iostream>

void callYolo(cv::Mat image);
void display3DReCon(cv::Mat image);

int main(int argc, char **argv){
	
	//TODO : Load image.
	cv::Mat mat;
	cv::imshow("test", mat);

	// Call Yolo( detection )


	// Feature detecting & matching.

	// 3D reconstruction.


	cv::imshow("test", mat);
	cv::waitKey(0);


	return 0;
}

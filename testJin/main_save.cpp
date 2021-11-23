#include "darknet.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "CallYolo.h"
#include "Display3DReCon.h"

#include "calibration.h"


cv::Mat cropDetection(bbox_t bbox1,cv::Mat img1,bbox_t bbox2,cv::Mat img2);
bbox_t getBbox(bbox_t_container cont, int contSize,int obj_id, double threshold);

// bool saveCont(char *filepath, bbox_t_container cont, int contsize);
// bbox_t_container readCont(char *filepath, int *contsize);

#define DEBUG 1

const String keys =
    "{help h usage ? |                  | print this message                                                }"
    "{@left          |./data/aloeL.jpg  | left view of the stereopair                                       }"
    "{@right         |./data/aloeR.jpg  | right view of the stereopair                                      }"
    "{GT             |./aloeGT.png| optional ground-truth disparity (MPI-Sintel or Middlebury format) }"
    "{dst_path       |None              | optional path to save the resulting filtered disparity map        }"
    "{dst_raw_path   |None              | optional path to save raw disparity map before filtering          }"
    "{algorithm      |sgbm                | stereo matching method (bm or sgbm)                               }"
    "{filter         |wls_conf          | used post-filtering (wls_conf or wls_no_conf or fbs_conf)         }"
    "{no-display     |                  | don't display results                                             }"
    "{no-downscale   |1                 | force stereo matching on full-sized views to improve quality      }"
    "{dst_conf_path  |None              | optional path to save the confidence map used in filtering        }"
    "{vis_mult       |8.0               | coefficient used to scale disparity map visualizations            }"
    "{max_disparity  |16                | parameter of stereo matching                                      }"
    "{window_size    |-1                | parameter of stereo matching                                      }"
    "{wls_lambda     |8000.0            | parameter of wls post-filtering                                   }"
    "{wls_sigma      |1.5               | parameter of wls post-filtering                                   }"
    "{fbs_spatial    |16.0              | parameter of fbs post-filtering                                   }"
    "{fbs_luma       |8.0               | parameter of fbs post-filtering                                   }"
    "{fbs_chroma     |8.0               | parameter of fbs post-filtering                                   }"
    "{fbs_lambda     |128.0             | parameter of fbs post-filtering                                   }"
    ;

/*
BICYCLE 1
CUP 41
CHAIR 56
MONITER 62
CONTROLLER 67
MICROWAVE 68
CLOCK 74
SCISSORS 76
*/
// ./myProject ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/135cm/135cm_1.jpg 56 0.99
// ./myProject ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/testImg_1.jpg 1 0.5
// ./myProject ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/cup.jpg 41 0.5
// ./myProject ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/microwave.jpg 68 0.5
// ./myProject ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/1115 Dataset/WIN_20211115_17_09_34_Pro.jpg 41 0.5

// ./myProject ../darknet/cfg/yolov4-tiny.cfg ../darknet/yolov4-tiny.weights ../resources/135cm/135cm_1.jpg 56 0.5

int main(int argc, char **argv)
{
	if(argc!=6){
		std::cout<<"usage: ./myProject cfg_path weights_path src_path obj_id threshold"<<'\n';
		exit(1);
	}

	std::string cfgpath = argv[1];
	std::string weightspath = argv[2];
	std::string photopath = argv[3];
	
	int obj_id;
	std::stringstream ssInt(argv[4]);
	ssInt>>obj_id;

	double threshold;
	std::stringstream ssDouble(argv[5]);
	ssDouble>>threshold;

	CallYolo *yolo = new CallYolo();
	yolo->init(cfgpath, weightspath);
	cv::Mat srcImg = cv::imread(photopath);

	CommandLineParser parser(argc,argv,keys);
    Display3DReCon* dp=new Display3DReCon();
    dp->init(parser);

	bbox_t_container cont1,cont2;
	int contsize1,contsize2;

	cv::Mat img1=srcImg(cv::Rect(0,0,srcImg.cols/2,srcImg.rows));
	cv::Mat img2=srcImg(cv::Rect(srcImg.cols/2,0,srcImg.cols/2,srcImg.rows));
	std::string photopath1 = "left.jpg";
	std::string photopath2 = "right.jpg";
	imwrite(photopath1,img1);
	imwrite(photopath2,img2);

	//printf("\telapsed1\tdetecting_time\telpased2\tdetecting_time+elpased2\tdiff\n");

	double begin = (double)getTickCount();

	yolo->setPhoto(photopath1);
	cont1 = yolo->getCont();
	contsize1=yolo->getContSize();

	yolo->setPhoto(photopath2);
	cont2 = yolo->getCont();
	contsize2=yolo->getContSize();

	double end=(double)getTickCount();
	double detecting_time = (end-begin)/getTickFrequency();
	
	bbox_t bbox1,bbox2;
	bbox1=getBbox( cont1,contsize1,obj_id, threshold);
	bbox2=getBbox( cont2,contsize2,obj_id, threshold);
	
	cv::Mat detectedImg = cropDetection(bbox1,img1,bbox2,img2);

	imshow("srcImg",srcImg);
	imshow("detectedImg",detectedImg);
	waitKey(0);

	double elapsed1=dp->test3dReCon(srcImg);
	double elapsed2=dp->test3dReCon(detectedImg);
	double diff=elapsed1-(detecting_time+elapsed2);
	printf("difference: %lf\n",diff);
	//printf("\t%lf\t%lf\t%lf\t%lf\t%lf\n",elapsed1,detecting_time,elapsed2,detecting_time+elapsed2,diff);
	return 0;
}

cv::Mat cropDetection(bbox_t bbox1,cv::Mat img1,bbox_t bbox2,cv::Mat img2)
{
	cv::Mat detected;
	int nx,ny,nw,nh;

	nx=min(bbox1.x,bbox2.x);
	ny=min(bbox1.y,bbox2.y);
	nw=max(bbox1.x+bbox1.w,bbox2.x+bbox2.w)-nx;
	nh=max(bbox1.y+bbox1.h,bbox2.y+bbox2.h)-ny;

	hconcat(img1(cv::Rect(nx,ny,nw,nh)), img2(cv::Rect(nx,ny,nw,nh)),detected);
	return detected;
}

bbox_t getBbox(bbox_t_container cont, int contSize,int obj_id, double threshold){
	bbox_t bbox;
	bool flag=false;

	for(int i=0;i<contSize;i++){
		printf("cont[%d]: obj_id %d, (%d, %d, %d, %d), prob: %lf\n",i,cont.candidates[i].obj_id,cont.candidates[i].x,cont.candidates[i].y,cont.candidates[i].w,cont.candidates[i].h,cont.candidates[i].prob);
	
		if(cont.candidates[i].obj_id==obj_id&&cont.candidates[i].prob>=threshold){
			if(!flag){
				bbox=cont.candidates[i];
				flag=true;
				printf("bbox: (%d, %d, %d, %d)\n",bbox.x,bbox.y,bbox.w,bbox.h);
			}
			else{
				std::cout<<"There are more than one object("<<obj_id<<")in the container."<<'\n';
				exit(1);
			}
		}
	}
	return bbox;
}

/*
bool saveCont(char *filepath, bbox_t_container cont, int contsize)
{
	FILE *fp = fopen(filepath, "w");
	if (!fp)
	{
		printf("OUTPUT FILE cannot be opened\n");
		return false;
	}
	fprintf(fp, "%d\n", contsize);

	for (int i = 0; i < contsize; i++)
	{
		fprintf(fp, "%d %d %d %d %d %f\n",
				(int)cont.candidates[i].obj_id,
				(int)cont.candidates[i].x,
				(int)cont.candidates[i].y,
				(int)cont.candidates[i].w,
				(int)cont.candidates[i].h,
				cont.candidates[i].prob);
	}
	fclose(fp);
	printf("%s is saved\n", filepath);
	return true;
}

bbox_t_container readCont(char *filepath, int *contsize)
{
	bbox_t_container cont;

	FILE *fp = fopen(filepath, "r");
	if (!fp)
	{
		printf("INPUT FILE cannot be opened\n");
		return cont;
	}
	fscanf(fp, "%d\n", contsize);
	for (int i = 0; i < *contsize; i++)
	{
		fscanf(fp, "%d %d %d %d %d %f\n",
			   &cont.candidates[i].obj_id, &cont.candidates[i].x, &cont.candidates[i].y, &cont.candidates[i].w, &cont.candidates[i].h, &cont.candidates[i].prob);
	}
	fclose(fp);
	printf("%s is read\n", filepath);
	return cont;
}
*/
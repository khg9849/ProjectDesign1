#include "darknet.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#include "CallYolo.h"
#include "Display3DReCon.h"

#define CHAIR 56
#define MONITER 62
#define CONTROLLER 67
#define THRESHOLD 0.9

#define EPOCH 10

//void callYolo(cv::Mat image);
//void display3DReCon(cv::Mat image);

cv::Mat cropDetection(bbox_t_container cont, int contsize, cv::Mat src);
bool saveCont(char *filepath, bbox_t_container cont, int contsize);
bbox_t_container readCont(char *filepath, int *contsize);

#define DEBUG 1

const String keys =
    "{help h usage ? |                  | print this message                                                }"
    "{@left          |./data/aloeL.jpg  | left view of the stereopair                                       }"
    "{@right         |./data/aloeR.jpg  | right view of the stereopair                                      }"
    "{GT             |./aloeGT.png| optional ground-truth disparity (MPI-Sintel or Middlebury format) }"
    "{dst_path       |None              | optional path to save the resulting filtered disparity map        }"
    "{dst_raw_path   |None              | optional path to save raw disparity map before filtering          }"
    "{algorithm      |bm                | stereo matching method (bm or sgbm)                               }"
    "{filter         |wls_conf          | used post-filtering (wls_conf or wls_no_conf or fbs_conf)         }"
    "{no-display     |                  | don't display results                                             }"
    "{no-downscale   |                  | force stereo matching on full-sized views to improve quality      }"
    "{dst_conf_path  |None              | optional path to save the confidence map used in filtering        }"
    "{vis_mult       |1.0               | coefficient used to scale disparity map visualizations            }"
    "{max_disparity  |16                | parameter of stereo matching                                      }"
    "{window_size    |-1                | parameter of stereo matching                                      }"
    "{wls_lambda     |8000.0            | parameter of wls post-filtering                                   }"
    "{wls_sigma      |1.5               | parameter of wls post-filtering                                   }"
    "{fbs_spatial    |16.0              | parameter of fbs post-filtering                                   }"
    "{fbs_luma       |8.0               | parameter of fbs post-filtering                                   }"
    "{fbs_chroma     |8.0               | parameter of fbs post-filtering                                   }"
    "{fbs_lambda     |128.0             | parameter of fbs post-filtering                                   }"
    ;


int main(int argc, char **argv)
{
	std::string cfgpath = "../darknet/cfg/yolov4.cfg";
	std::string weightspath = "../darknet/yolov4.weights";
	bbox_t_container cont;
	int contsize;

	std::string photopath = "../resources/135cm/135cm_1.jpg";
	std::string detectedpath="../resources/135cm/135cm_detected.jpg";
	char *contpath = "../cont135.txt";

	// CallYolo *yolo = new CallYolo();
	// yolo->init(cfgpath, weightspath);
	// yolo->setPhoto(photopath);
	// cont = yolo->getCont();
	// contsize = (int)yolo->getContSize();
	// bool isContSaved = saveCont(contpath, cont, contsize);
	
	cont=readCont(contpath,&contsize);
	
	cv::Mat srcImg = cv::imread(photopath);
	cv::Mat detectedImg = cropDetection(cont, contsize, srcImg);
	cv::imshow("srcImg",srcImg);
	cv::imshow("detectedImg",detectedImg);
	cv::imwrite(detectedpath, detectedImg);
	cv::waitKey(0);
	
	CommandLineParser parser(argc,argv,keys);
    Display3DReCon* dp=new Display3DReCon();
    dp->init(parser);

    double elapsed1,elapsed2,diff;
    double sum=0,avg;

    for(int i = 0;i < EPOCH;i++){
        elapsed1=dp->test3dReCon(srcImg);
        elapsed2=dp->test3dReCon(detectedImg);
        diff=elapsed1-elapsed2;
        sum+=(diff);
		printf("[%d] difference: %lf\n",i,diff);
    }
    avg=sum/EPOCH;
    printf("avg is %lf\n",avg);
	
	return 0;
}

cv::Mat cropDetection(bbox_t_container cont, int contsize, cv::Mat src)
{
	cv::Mat res = src.clone();
	cv::Mat detected;
	cv::Mat croptedImg[2];

	int wmax = 0, hmax = 0;
	int lx = src.cols, ly = 0;
	int rx = 0, ry = 0;


	cv::Mat mask = cv::Mat::zeros(res.size(), CV_8U);  
	cv::Mat masked = cv::Mat::zeros(res.size(), CV_8U); 

	for (int i = 0; i < contsize; i++)
	{
		int cx = cont.candidates[i].x;
		int cy = cont.candidates[i].y;
		int cw = cont.candidates[i].w;
		int ch = cont.candidates[i].h;
		cv::rectangle(res, cv::Point(cx, cy), cv::Point(cx + cw, cy + ch), 1, 8, 0);
		
#if DEBUG
		printf("obj_id, cx, cy, cw, ch, prob = (%d, %d,%d,%d,%d,%lf)\n", cont.candidates[i].obj_id, cx, cy, cw, ch, cont.candidates[i].prob);
		cv::imshow("making boxes...",res);
		cv::waitKey(0);
#endif
		if (cont.candidates[i].obj_id == CHAIR && cont.candidates[i].prob >= THRESHOLD)
		{
			if (cx < lx)
			{
				lx = cx;
				ly = cy;
			}
			if (cx > rx)
			{
				rx = cx;
				ry = cy;
			}
			wmax = std::max(wmax, cw);
			hmax = std::max(hmax, ch);
		}
	}

	croptedImg[0] = src(cv::Rect(lx, ly, wmax, hmax));
	croptedImg[1] = src(cv::Rect(rx, ry, wmax, hmax));
	hconcat(croptedImg[0], croptedImg[1], detected);
	
	return detected;
}

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

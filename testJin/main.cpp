#include "darknet.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "CallYolo.h"
#include "Display3DReCon.h"

#include "calibration.h"

#define BICYCLE 1
#define CHAIR 56
#define MONITER 62
#define CONTROLLER 67
#define THRESHOLD 0.5

#define EPOCH 100

//void callYolo(cv::Mat image);
//void display3DReCon(cv::Mat image);

cv::Mat cropDetection(bbox_t_container cont, int contsize,cv::Mat src, int object);
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
    "{algorithm      |sgbm                | stereo matching method (bm or sgbm)                               }"
    "{filter         |wls_conf          | used post-filtering (wls_conf or wls_no_conf or fbs_conf)         }"
    "{no-display     |1                  | don't display results                                             }"
    "{no-downscale   |1                 | force stereo matching on full-sized views to improve quality      }"
    "{dst_conf_path  |None              | optional path to save the confidence map used in filtering        }"
    "{vis_mult       |10.0               | coefficient used to scale disparity map visualizations            }"
    "{max_disparity  |32                | parameter of stereo matching                                      }"
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
	cb myCB = cb();
	std::string path1 = "../resources/calib_data/*.jpg";
	std::string path2="../calibration.xml";

    //myCB.calib(path1, path2);
	//myCB.readCalibResult(path2);

//	cv::Mat T = myCB.T;
//	float baseline = T.at<float>(0,0);
//	if(baseline<0) baseline*-1;
//	cout<<"baseline is "<<baseline<<'\n';

	std::string cfgpath = "../darknet/cfg/yolov4-tiny.cfg";
	std::string weightspath = "../darknet/yolov4-tiny.weights";
	bbox_t_container cont1,cont2;
	int contsize1,contsize2;

	std::string photopath = "../resources/135cm/135cm_1.jpg";
	std::string detectedpath="../resources/135cm/135cm_detected.jpg";
	char *contpath = "../cont135.txt";

	CallYolo *yolo = new CallYolo();
	yolo->init(cfgpath, weightspath);

	CommandLineParser parser(argc,argv,keys);
    Display3DReCon* dp=new Display3DReCon();
    dp->init(parser);

    double elapsed1,elapsed2,diff;
    double sum=0,avg;

	cv::Mat srcImg = cv::imread(photopath);
	cv::Mat detectedImg;
	double detecting_time;

	std::string photopath1 = "left.jpg";
	std::string photopath2 = "right.jpg";
	imwrite(photopath1,srcImg(cv::Rect(0,0,srcImg.cols/2,srcImg.rows)));
	imwrite(photopath2,srcImg(cv::Rect(srcImg.cols/2,0,srcImg.cols/2,srcImg.rows)));

	//bool isContSaved = saveCont(contpath, cont, contsize);

	
	// cv::imshow("srcImg",srcImg);
	// cv::imshow("detectedImg",detectedImg);
	// cv::imwrite(detectedpath, detectedImg);
	// cv::waitKey(0);
	
	

	//cont=readCont(contpath,&contsize);
	printf("\telapsed1\tdetecting_time\telpased2\tdetecting_time+elpased2\tdiff\n");
    for(int i = 0;i < EPOCH;i++){
		detecting_time = (double)getTickCount();
		yolo->setPhoto(photopath1);
		cont1 = yolo->getCont();
		contsize1=yolo->getContSize();
		yolo->setPhoto(photopath2);
		cont2 = yolo->getCont();
		contsize2=yolo->getContSize();
		detecting_time = ((double)getTickCount() - detecting_time)/getTickFrequency();
        
		for(int i=0;i<contsize2;i++){
			cont1.candidates[contsize1+i]=cont2.candidates[i];
			cont1.candidates[contsize1+i].x+=srcImg.cols/2;
		}
		contsize1+=contsize2;

		detectedImg = cropDetection(cont1, contsize1,srcImg,CHAIR);
        elapsed1=dp->test3dReCon(srcImg);
        elapsed2=dp->test3dReCon(detectedImg);
        diff=elapsed1-(detecting_time+elapsed2);
        sum+=(diff);
		//printf("[%d] difference: %lf\n",i,diff);
		printf("%d\t%lf\t%lf\t%lf\t%lf\t%lf\n",i,elapsed1,detecting_time,elapsed2,detecting_time+elapsed2,diff);
    }
    avg=sum/EPOCH;
    printf("avg is %lf\n",avg);
	
	return 0;
}

cv::Mat cropDetection(bbox_t_container cont, int contsize, cv::Mat src, int object)
{

	cv::Mat res = src.clone();
	cv::Mat detected;
	cv::Mat croptedImg[2];

	int lx = src.cols,ly,lw,lh;
	int rx=0,ry,rw,rh;

	cv::Mat mask = cv::Mat::zeros(res.size(), CV_8U);  
	cv::Mat masked = cv::Mat::zeros(res.size(), CV_8U); 

	for (int i = 0; i < contsize; i++)
	{
		int cx = cont.candidates[i].x;
		int cy = cont.candidates[i].y;
		int cw = cont.candidates[i].w;
		int ch = cont.candidates[i].h;

		cv::Scalar color=cv::Scalar(255,0,0);
		if(cont.candidates[i].obj_id==object)
			color=cv::Scalar(0,0,255);
		cv::rectangle(res, cv::Point(cx, cy), cv::Point(cx + cw, cy + ch), color, 8, 0);
		
#if DEBUG
		printf("obj_id, cx, cy, cw, ch, prob = (%d, %d,%d,%d,%d,%lf)\n", cont.candidates[i].obj_id, cx, cy, cw, ch, cont.candidates[i].prob);
		cv::imshow("making boxes...",res);
		cv::waitKey(0);
#endif
		if (cont.candidates[i].obj_id == object && cont.candidates[i].prob >= THRESHOLD)
		{
			if(cx<lx){
				lx=cx;
				ly=cy;
				lw=cw;
				lh=ch;
			}
			if(cx>rx){
				rx=cx;
				ry=cy;
				rw=cw;
				rh=ch;
			}
		}
	}
	int rx2=rx-src.cols/2;
	int nx,ny,nw,nh;
	nx=min(lx,rx2);
	ny=min(ly,ry);
	nw=max(lx+lw,rx2+rw)-nx;
	nh=max(ly+lh,ry+rh)-ny;
#if DEBUG
	printf("(nx,ny,nw,nh) = (%d,%d,%d,%d)\n",nx,ny,nw,nh);
#endif
	croptedImg[0] = src(cv::Rect(nx, ny, nw, nh));
	croptedImg[1] = src(cv::Rect(nx+src.cols/2, ny, nw, nh));
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

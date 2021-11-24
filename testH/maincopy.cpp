#include "darknet.h"
#include "CallYolo.h"
#include "Display3DReCon.h"
#include "calibration.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#define BICYCLE 1
#define CHAIR 56
#define MONITER 62
#define CONTROLLER 67
#define THRESHOLD 0.9
#define EPOCH 300
#define DEBUG 0

using namespace cv;
using namespace std;

Mat cropDetection(bbox_t_container cont, int contsize, Mat src, int object);
bool saveCont(char *filepath, bbox_t_container cont, int contsize);
bbox_t_container readCont(char *filepath, int *contsize);

int main(int argc, char **argv){
	cb myCB = cb();
	string path1 = "../resources/calib_data/*.jpg";
	string path2 = "../calibration.xml";

	string cfgpath = "../darknet/cfg/yolov4.cfg";
	string weightspath = "../darknet/yolov4.weights";
	bbox_t_container cont;
	int contsize;

	string photopath = "../resources/135cm/135cm_1.jpg";
	string detectedpath = "../resources/135cm/135cm_detected.jpg";
	char *contpath = "../cont135.txt";

	cont = readCont(contpath, &contsize);

	Mat srcImg = imread(photopath);
	Mat detectedImg = cropDetection(cont, contsize, srcImg, CHAIR);
	imshow("srcImg", srcImg);
	imshow("detectedImg", detectedImg);
	imwrite(detectedpath, detectedImg);
	waitKey(0);

	CommandLineParser parser(argc, argv, keys);
	Display3DReCon *dp = new Display3DReCon();
	dp->init(parser);

	double elapsed1, elapsed2, diff;
	double sum = 0, avg;

	for(int i = 0; i < EPOCH; i++){
		elapsed1 = dp->test3dReCon(srcImg);
		elapsed2 = dp->test3dReCon(detectedImg);
		diff = elapsed1-elapsed2;
		sum += diff;
		printf("[%d] difference: %lf\n", i, diff);
	}
	avg = sum/EPOCH;
	printf("avg is %lf\n", avg);

	return 0;
}

Mat cropDetection(bbox_t_container cont, int contsize, Mat src, int object){
	Mat res = src.clone();
	Mat detected;
	Mat croptedImg[2];

	int lx = src.cols, ly, lw, lh;
	int rx = 0, ry, rw, rh;

	Mat mask = Mat::zeros(res.size(), CV_8U);
	Mat masked = Mat::zeros(res.size(), CV_8U);

	for(int i = 0; i < contsize; i++){
		int cx = cont.candidates[i].x;
		int cy = cont.candidates[i].y;
		int cw = cont.candidates[i].w;
		int ch = cont.candidates[i].h;
		rectangle(res, Point(cx, cy), Point(cx + cw, cy + ch), 1, 8, 0);

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

	printf("(nx,ny,nw,nh) = (%d,%d,%d,%d)\n",nx,ny,nw,nh);

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

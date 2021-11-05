#include "darknet.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include "CallYolo.h"
#include <stdio.h>

#define CHAIR 56
#define MONITER 62
#define THRESHOLD 0.8

void callYolo(cv::Mat image);
void display3DReCon(cv::Mat image);

cv::Mat cropDetection(bbox_t_container cont, int contsize, cv::Mat src);
bool saveCont(char *filepath, bbox_t_container cont, int contsize);
bbox_t_container readCont(char *filepath, int *contsize);

int main(int argc, char **argv)
{

	std::string photopath = "../resources/anotherImg.jpg";
	std::string cfgpath = "../darknet/cfg/yolov4.cfg";
	std::string weightspath = "../darknet/yolov4.weights";
	bbox_t_container cont;
	int contsize;

	CallYolo *yolo = new CallYolo();
	yolo->init(cfgpath, weightspath);
	yolo->setPhoto(photopath);
	cont = yolo->getCont();
	contsize = (int)yolo->getContSize();

	cv::Mat src = cv::imread(photopath);
	char *contpath = "../cont2.txt";
	bool isContSaved = saveCont(contpath, cont, contsize);
	//cont=readCont(contpath,&contsize);
	cv::Mat detected = cropDetection(cont, contsize, src);

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

	for (int i = 0; i < contsize; i++)
	{
		int cx = cont.candidates[i].x;
		int cy = cont.candidates[i].y;
		int cw = cont.candidates[i].w;
		int ch = cont.candidates[i].h;

		printf("obj_id, cx, cy, cw, ch, prob = (%d, %d,%d,%d,%d,%lf)\n", cont.candidates[i].obj_id, cx, cy, cw, ch, cont.candidates[i].prob);

		cv::rectangle(res, cv::Point(cx, cy), cv::Point(cx + cw, cx + ch), 1, 8, 0);

		if (cont.candidates[i].obj_id == 1 && cont.candidates[i].prob >= THRESHOLD)
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

	cv::imwrite("../resources/detected.jpg", detected);
	printf("../resources/detected.jpg is saved\n");
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

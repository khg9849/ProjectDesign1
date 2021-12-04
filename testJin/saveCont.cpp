#include "darknet.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "CallYolo.h"


bool saveCont(char *filepath, bbox_t_container cont, int contsize);
bbox_t_container readCont(char *filepath, int *contsize);

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

/*
 ./saveCont ../darknet/cfg/yolov4-tiny.cfg ../darknet/yolov4-tiny.weights ../resources/135cm/135cm_1.jpg
 ./saveCont ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/1117dataset/31cm_scissors.jpg
 ./saveCont ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/45cm/45cm_1.jpg
 ./saveCont ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/45cm/45cm_1.jpg ../resources/45cm/cont.txt
 ./saveCont ../darknet/cfg/yolov4.cfg ../darknet/yolov4.weights ../resources/90cm/90cm_1.jpg ../resources/90cm/cont.txt
*/

void getDepthZ(){}
int main(int argc, char **argv)
{
	if(argc!=5){
		std::cout<<"usage: ./saveCont cfg_path weights_path src_path cont_path"<<'\n';
		exit(1);
	}

	std::string cfgpath = argv[1];
	std::string weightspath = argv[2];
	std::string photopath = argv[3];
	char* contpath = argv[4];

	CallYolo *yolo = new CallYolo();
	yolo->init(cfgpath, weightspath);
	cv::Mat srcImg = cv::imread(photopath);

	bbox_t_container cont;
	int contsize;

	yolo->setPhoto(photopath);
	cont = yolo->getCont();
	contsize=yolo->getContSize();

	saveCont(contpath,cont,contsize);
	return 0;
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

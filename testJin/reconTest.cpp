#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"

bbox_t_container readCont(char *filepath, int *contsize);


int main(int argc, char **argv)
{
	bbox_t_container cont;
	int contsize;
	cont=readCont("../cont.txt",&contsize);

	const cv::Mat src=cv::imread("../resources/1115Dataset/WIN_20211115_17_09_34_Pro.jpg");
	
	/* array */
	/*
	bbox_t pObject[1000];
	cv::Mat rtn[1000];
	for(int i=0;i<contsize;i++)
		pObject[i]=cont.candidates[i];

	SYE::YeStereoCamera *sye=new SYE::YeStereoCamera();
	sye->getSgbmInRect(src,pObject,contsize,rtn);
	for(int i=0;i<contsize/2;i++){
		cv::imshow("result",rtn[i]);
		cv::waitKey(0);
	}
	*/
	
	/* vector */
	std::vector<bbox_t> pObject;
	std::vector<cv::Mat> rtn;
	for(int i=0;i<contsize;i++)
		pObject.push_back(cont.candidates[i]);

	SYE::YeStereoCamera *sye=new SYE::YeStereoCamera();
	sye->getSgbmInRect(src,pObject,&rtn);
	for(cv::Mat res:rtn){
		cv::imshow("result",res);
		cv::waitKey(0);
	}
	return 0;
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
	return cont;
}

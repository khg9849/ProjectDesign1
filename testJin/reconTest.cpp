#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"

bbox_t_container readCont(char *filepath, int *contsize);

/*
 ./reconTest ../resources/1117dataset/31cm_scissors.jpg ../resources/1117dataset/scirrsors.txt
 ./reconTest ../resources/45cm/45cm_1.jpg ../resources/45cm/cont.txt
  ./reconTest ../resources/90cm/90cm_1.jpg ../resources/90cm/cont.txt
*/
using namespace std;
using namespace SYE;
using namespace cv;

int main(int argc, char **argv)
{
	

	bbox_t_container cont;
	int contsize;
	cont=readCont(argv[2],&contsize);

	const cv::Mat src=cv::imread(argv[1]);
	imshow("src",src);
	waitKey(0);
	/* vector */
	std::vector<bbox_t> pObject;
	std::vector<cv::Mat> rtn;
	std::vector<bbox_t> rtnPos;
	for(int i=0;i<contsize;i++){
		pObject.push_back(cont.candidates[i]);
	}
	
	for (int i = 0; i < pObject.size(); i++) {
            std::cout << "x y w h id prob : " << pObject[i].x << " " << pObject[i].y << " " << pObject[i].w << " " << pObject[i].h << " " << pObject[i].obj_id << " " << pObject[i].prob<<"\n";
        }
	
	SYE::YeStereoCamera *sye=new SYE::YeStereoCamera();

	if (!sye->initCalibData("calibration.xml")) {
		std::cout << "initCalibData failed\n";
		exit(1);
    }
	sye->initMatrix();

	sye->getSgbmInRect(src,pObject,rtn,rtnPos);
	imshow("rtn[0]",rtn[0]);

	std::vector<SYE::YePos3D> feature;
    std::vector<bbox_t> pos;
	if ((sye->getAbsoluteLengthInRect(src, pObject,rtnPos, feature, pos)) == false) {
		std::cout << "getAbsoluteLengthInRect failed\n";
		exit(1);
	}


    sye->showResult(src,rtn,rtnPos,feature,pos);

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

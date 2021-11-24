#include "yeStereoCamera.hpp"

#include <fstream>

int main(int argc, char** argv){
	cv::FileStorage fsr("calibration.xml", cv::FileStorage::READ);
	
	SYE::YeStereoCamera a;

	fsr["translationVector"] >> a.matT;
	fsr["leftCameraMatrix"] >> a.matCamMat1;
	fsr["rightCameraMatrix"] >> a.matCamMat2;

	bbox_t temp;
	temp.x = 600;
	temp.y = 160;
	temp.w = 1080;
	temp.h = 500;

	cv::Mat img = cv::imread("../resources/cube/new/20CM.jpg");

	std::vector<SYE::YePos3D> feature;

	a.getAbsoluteLengthInRect(img, &temp, feature);

	cv::imshow("test", img(cv::Range(160, 500), cv::Range(600, 1080)));
	cv::waitKey(0);

	for(int i = 0; i < feature.size(); i++){
		//std::cout<<feature[i].x<<' '<<feature[i].y<<' '<<feature[i].z<<'\n';
	}

	return 0;
}

#include "yeStereoCamera.hpp"

#include <fstream>

int main(int argc, char** argv){
	cv::FileStorage fsr("calibration.xml", cv::FileStorage::READ);
	
	SYE::YeStereoCamera a;

	a.doCalibration("resources/calib_data/", "calibration.xml");

	/*fsr["translationVector"] >> a.matT;
	fsr["leftCameraMatrix"] >> a.matCamMat1;
	fsr["rightCameraMatrix"] >> a.matCamMat2;

	bbox_t temp[2];
	temp[0].x = 600;
	temp[0].y = 250;
	temp[0].w = 1000;
	temp[0].h = 500;
	temp[1].x = 1750;
	temp[1].y = 250;
	temp[1].w = 2150;
	temp[1].h = 500;

	cv::Mat img = cv::imread("../resources/cube/new/30cm.jpg");

	std::vector<std::vector<SYE::YePos3D>> feature;

	a.getAbsoluteLengthInRect(img, temp, 2, feature);

	cv::imshow("test", img(cv::Range(250, 500), cv::Range(600, 1000)));
	cv::waitKey(0);

	for(int i = 0; i < feature[0].size(); i++){
		std::cout<<feature[0][i].x<<' '<<feature[0][i].y<<' '<<feature[0][i].z<<'\n';
	}*/

	return 0;
}

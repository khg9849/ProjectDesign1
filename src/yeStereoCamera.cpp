#include "yeStereoCamera.hpp"

#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}
YeStereoCamera::~YeStereoCamera() {
}

//경로 내부의 이미지 파일을 읽어서 켈리브레이션 실시.
bool YeStereoCamera::doCalibration(const char* pPath, const char* ext = ".jpg") {

}
bool YeStereoCamera::doCalibration(std::vector<std::string>& imgList) {

}



// Yolo를 이용하여 특정 이름의 영역을 추출.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, bbox_t* pObjRect) 
{

}



//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, std::vector<YePos3D>& features) {

}


// 추춘된 특정 영역만 SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 팀 화 이 팅 ! ! ! ,,----------------------------------------------------*/
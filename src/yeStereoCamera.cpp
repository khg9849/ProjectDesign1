#include "yeStereoCamera.hpp"

#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}
YeStereoCamera::~YeStereoCamera() {
}

//��� ������ �̹��� ������ �о �̸��극�̼� �ǽ�.
bool YeStereoCamera::doCalibration(const char* pPath, const char* ext = ".jpg") {

}
bool YeStereoCamera::doCalibration(std::vector<std::string>& imgList) {

}



// Yolo�� �̿��Ͽ� Ư�� �̸��� ������ ����.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, bbox_t* pObjRect) 
{

}



//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, std::vector<YePos3D>& features) {

}


// ����� Ư�� ������ SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 �� ȭ �� �� ! ! ! ,,----------------------------------------------------*/
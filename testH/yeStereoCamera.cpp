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
	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	cv::Mat gray, pic[2], dc[2];
	std::vector<cv::KeyPoint> kp[2];
	std::vector<cv::DMatch> matches;

	cv::Mat invMat[2];
	invMat[0] = matCamMat1.inv();
	invMat[1] = matCamMat2.inv();

	cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);

	pic[0] = gray(cv::Range(pObjRect->y, pObjRect->h), cv::Range(pObjRect->x, pObjRect->w));
	pic[1] = gray(cv::Range(pObjRect->y, pObjRect->h), cv::Range(gray.cols/2+pObjRect->x, gray.cols/2+pObjRect->w));

	fast->detect(pic[0], kp[0], pic[1]);
	brief->compute(pic[0], kp[0], dc[0]);
	fast->detect(pic[1], kp[1], pic[0]);
	brief->compute(pic[1], kp[1], dc[1]);
	matcher->match(dc[0], dc[1], matches);

	for(int i = 0; i < matches.size(); i++){
		YePos3D temp;
		
		temp.x = (int)kp[0][matches[i].queryIdx].pt.x;
		temp.y = (int)kp[1][matches[i].queryIdx].pt.y;
		temp.z = -matT.at<double>(0,0)/(invMat[1].at<double>(0,0)*(int)(-kp[1][matches[i].trainIdx].pt.x)-invMat[1].at<double>(0,2)+invMat[0].at<double>(0,0)*(int)(kp[0][matches[i].queryIdx].pt.x)+invMat[0].at<double>(0,2));

		features.push_back(temp);
	}
}


// 추춘된 특정 영역만 SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 팀 화 이 팅 ! ! ! ,,----------------------------------------------------*/

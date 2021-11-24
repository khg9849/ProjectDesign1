#include "yeStereoCamera.hpp"

#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}
YeStereoCamera::~YeStereoCamera() {
}

//��� ������ �̹��� ������ �о �̸��극�̼� �ǽ�.
bool YeStereoCamera::doCalibration(const char* pPath, const char* ext) {

}
bool YeStereoCamera::doCalibration(std::vector<std::string>& imgList) {

}



// Yolo�� �̿��Ͽ� Ư�� �̸��� ������ ����.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, bbox_t* pObjRect) 
{

}



//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, std::vector<bbox_t> pObjRect, std::vector<std::vector<YePos3D>>& features) {
	
	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
	
	cv::Mat invMat[2];
	invMat[0] = matCamMat1.inv();
	invMat[1] = matCamMat2.inv();

	for(int i = 0; i < pObjRect.size(); i+=2){
		int left, right;
		if(pObjRect[i].x < pObjRect[i+1].x){
			left = i;
			right = i+1;
		}
		else{
			left = i+1;
			right = i;
		}

		cv::Mat pic[2], dc[2];
		std::vector<cv::KeyPoint> kp[2];
		std::vector<cv::DMatch> matches;

		pic[0] = gray(cv::Range(pObjRect[left].y, pObjRect[left].h), cv::Range(pObjRect[left].x, pObjRect[left].w));
		pic[1] = gray(cv::Range(pObjRect[right].y, pObjRect[right].h), cv::Range(pObjRect[right].x, pObjRect[right].w));

		fast->detect(pic[0], kp[0], pic[1]);
		brief->compute(pic[0], kp[0], dc[0]);
		fast->detect(pic[1], kp[1], pic[0]);
		brief->compute(pic[1], kp[1], dc[1]);
		matcher->match(dc[0], dc[1], matches);

		std::vector<YePos3D> feature_temp[2];
		for(int j = 0; j < matches.size(); j++){
			YePos3D temp[2];
		
			temp[0].x = invMat[0].at<double>(0,0)*((int)(kp[0][matches[j].queryIdx].pt.x)+pObjRect[left].x)+invMat[0].at<double>(0,2);
			temp[1].x = invMat[1].at<double>(0,0)*((int)(kp[1][matches[j].trainIdx].pt.x)+pObjRect[right].x-gray.cols/2)+invMat[1].at<double>(0,2);
			
			temp[0].z = -matT.at<double>(0,0)/(temp[0].x-temp[1].x);
			temp[1].z = temp[0].z;

			temp[0].x = temp[0].x*temp[0].z;
			temp[0].y = (invMat[0].at<double>(1,1)*((int)(kp[0][matches[j].queryIdx].pt.y)+pObjRect[left].y)+invMat[0].at<double>(1,2))*temp[0].z;

			temp[1].x = temp[1].x*temp[1].z;
			temp[1].y = (invMat[1].at<double>(1,1)*((int)(kp[1][matches[j].trainIdx].pt.y)+pObjRect[right].y)+invMat[1].at<double>(1,2))*temp[1].z;

			feature_temp[0].push_back(temp[0]);
			feature_temp[1].push_back(temp[1]);
		}

		features.push_back(feature_temp[0]);
		features.push_back(feature_temp[1]);
	}
}

bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, int objNum, std::vector<std::vector<YePos3D>>& features) {

	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
	
	cv::Mat invMat[2];
	invMat[0] = matCamMat1.inv();
	invMat[1] = matCamMat2.inv();

	for(int i = 0; i < objNum; i+=2){
		int left, right;
		if(pObjRect[i].x < pObjRect[i+1].x){
			left = i;
			right = i+1;
		}
		else{
			left = i+1;
			right = i;
		}

		cv::Mat pic[2], dc[2];
		std::vector<cv::KeyPoint> kp[2];
		std::vector<cv::DMatch> matches;

		pic[0] = gray(cv::Range(pObjRect[left].y, pObjRect[left].h), cv::Range(pObjRect[left].x, pObjRect[left].w));
		pic[1] = gray(cv::Range(pObjRect[right].y, pObjRect[right].h), cv::Range(pObjRect[right].x, pObjRect[right].w));

		fast->detect(pic[0], kp[0], pic[1]);
		brief->compute(pic[0], kp[0], dc[0]);
		fast->detect(pic[1], kp[1], pic[0]);
		brief->compute(pic[1], kp[1], dc[1]);
		matcher->match(dc[0], dc[1], matches);

		std::vector<YePos3D> feature_temp[2];
		for(int j = 0; j < matches.size(); j++){
			YePos3D temp[2];
		
			temp[0].x = invMat[0].at<double>(0,0)*((int)(kp[0][matches[j].queryIdx].pt.x)+pObjRect[left].x)+invMat[0].at<double>(0,2);
			temp[1].x = invMat[1].at<double>(0,0)*((int)(kp[1][matches[j].trainIdx].pt.x)+pObjRect[right].x-gray.cols/2)+invMat[1].at<double>(0,2);
			
			temp[0].z = -matT.at<double>(0,0)/(temp[0].x-temp[1].x);
			temp[1].z = temp[0].z;

			temp[0].x = temp[0].x*temp[0].z;
			temp[0].y = (invMat[0].at<double>(1,1)*((int)(kp[0][matches[j].queryIdx].pt.y)+pObjRect[left].y)+invMat[0].at<double>(1,2))*temp[0].z;

			temp[1].x = temp[1].x*temp[1].z;
			temp[1].y = (invMat[1].at<double>(1,1)*((int)(kp[1][matches[j].trainIdx].pt.y)+pObjRect[right].y)+invMat[1].at<double>(1,2))*temp[1].z;

			feature_temp[0].push_back(temp[0]);
			feature_temp[1].push_back(temp[1]);
		}

		features.push_back(feature_temp[0]);
		features.push_back(feature_temp[1]);
	}
}

// ����� Ư�� ������ SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 �� ȭ �� �� ! ! ! ,,----------------------------------------------------*/

#include "yeStereoCamera.hpp"

#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
	cfg_file = "";
	weight_file = "";
	detector = NULL;
}
YeStereoCamera::~YeStereoCamera() {
	if(detector != NULL)
		delete detector;
}

//��� ������ �̹��� ������ �о �̸��극�̼� �ǽ�.
bool YeStereoCamera::doCalibration(const char* pPath, const char* ext = ".jpg") {

}
bool YeStereoCamera::doCalibration(std::vector<std::string>& imgList) {

}



void YeStereoCamera::getWeight_file(std::string _w){
	weight_file = _w;
}
void YeStereoCamera::getcfg_file(std::string _c){
	cfg_file = _c;
}
// Yolo�� �̿��Ͽ� Ư�� �̸��� ������ ����.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, std::vector<bbox_t> pObjRect) {

	if(weight_file == ""){
		perror("findImage error : failed to find weight file!");
        return false;
	}
	if(cfg_file == ""){
		perror("findImage error : failed to find cfg file!");
        return false;
	}
	if(objName == ""){
		perror("findImage error : failed to understand objName!");
        return false;
	}
	if(detector == NULL){
		if((detector = new Detector(cfg_file, weight_file)) == NULL){
    	    perror("findImage error : failed to make detector object!");
			return false;
    	}
	}

	std::vector<bbox_t> detection_left, detection_right;
	size_t detectionSize;
	double threshold = 0.7;

    cv::Mat mat_left = mat(cv::Range::all(), cv::Range(0, mat.cols/2));
    cv::Mat mat_right = mat(cv::Range::all(), cv::Range(mat.cols/2, mat.cols));

	//yolo_v2_class.hpp
	//std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
	detection_left = detector->detect(mat_left);
	detection_right = detector->detect(mat_right);
    detectionSize = detection_left.size();

	//did you know how to match between obj_id and objName?
	//*.names
    for(size_t i = 0; i < detectionSize; ++i){
		if(	detection_left[i].obj_id == 0 && detection_left[i].prob >= threshold &&
			detection_right[i].obj_id == 0 && detection_right[i].prob >= threshold &&
			detection_left[i].obj_id == detection_right[i].obj_id){
				pObjRect.push_back(detection_left[i]);
				pObjRect.push_back(detection_right[i]);
			}
	}

    return true;
}
bool YeStereoCamera::findImage(const cv::Mat mat, const char *objName, bbox_t *pObjRect){
	
	if(weight_file == ""){
		perror("findImage error : failed to find weight file!");
        return false;
	}
	if(cfg_file == ""){
		perror("findImage error : failed to find cfg file!");
        return false;
	}
	if(objName == ""){
		perror("findImage error : failed to understand objName!");
        return false;
	}
	if(detector == NULL){
		if((detector = new Detector(cfg_file, weight_file)) == NULL){
    	    perror("findImage error : failed to make detector object!");
			return false;
    	}
	}

	std::vector<bbox_t> detection_left, detection_right;
	size_t detectionSize;
	double threshold = 0.7;

    cv::Mat mat_left = mat(cv::Range::all(), cv::Range(0, mat.cols/2));
    cv::Mat mat_right = mat(cv::Range::all(), cv::Range(mat.cols/2, mat.cols));

	//yolo_v2_class.hpp
	//std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
	detection_left = detector->detect(mat_left);
	detection_right = detector->detect(mat_right);
    detectionSize = detection_left.size();

	//did you know how to match between obj_id and objName?
	//*.names
    for(size_t i = 0; i < detectionSize; ++i){
		if(	detection_left[i].obj_id == 0 && detection_left[i].prob >= threshold &&
			detection_right[i].obj_id == 0 && detection_right[i].prob >= threshold &&
			detection_left[i].obj_id == detection_right[i].obj_id){
				pObjRect[i * 2] = detection_left[i];
				pObjRect[i * 2 + 1] = detection_right[i];
			}
	}

	findImageSize = detectionSize * 2;

    return true;
}



//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, std::vector<YePos3D>& features) {

}


// ����� Ư�� ������ SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, cv::Mat rtn) {

}

/*--------------------------------------------------4 �� ȭ �� �� ! ! ! ,,----------------------------------------------------*/
#ifndef YESTEREOCAMERA_HPP
#define YESTEREOCAMERA_HPP

#include <vector>
#include <string>

#include <yolo_v2_class.hpp>
#include <opencv2/opencv.hpp>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

namespace SYE {

struct YePos3D {
	double x;
	double y;
	double z;
};

class YeStereoCamera {
private:
	cv::Mat matCamMat1; //camera matrix.
	cv::Mat matCamMat2; //camera matrix.
	cv::Mat matDistCoffs1;
	cv::Mat matDistCoffs2;
	cv::Mat matT;			// translation matrix between two lens.
	cv::Mat matR;			// rotation matrix between two lens.
protected:
public:
	YeStereoCamera();
	~YeStereoCamera();
	/*
	virtual int doTest(int i) {
		return i * i;
	}*/

	//경로 내부의 이미지 파일을 읽어서 켈리브레이션 실시.
	// bool doCalibration(const char *pPath, const char *ext = ".jpg");
	// bool doCalibration(std::vector<std::string> &imgList);

	// Yolo를 이용하여 특정 이름의 영역을 추출.
	bool findImage(const cv::Mat mat, const char *objName, bbox_t *pObjRect);

	//Absolute length from camera.
	bool getAbsoluteLengthInRect(const cv::Mat src, bbox_t *pObjRect, std::vector<YePos3D> &features);


	// 추춘된 특정 영역만 SGBM 3D reconstruction.
	bool getSgbmInRect(const cv::Mat src, bbox_t *pObject, int size, cv::Mat* rtn);
	bool getSgbmInRect(const cv::Mat src, std::vector<bbox_t> pObject, std::vector<cv::Mat>* rtn);
};

}

#endif

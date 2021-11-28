#ifndef YESTEREOCAMERA_HPP
#define YESTEREOCAMERA_HPP
#define OPENCV

#include <vector>
#include <string>

#include <yolo_v2_class.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"

#include <fstream>

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
	cv::Mat matE;			// essential matrix between two lens.
	cv::Mat matF;			// fundamental matrix between two lens.

	
	Detector *detector;
	std::string weight_file;
	std::string cfg_file;
	int findImageSize;
	
	std::vector<std::string> objNames;
protected:
public:
	YeStereoCamera();
	~YeStereoCamera();
	/*
	virtual int doTest(int i) {
		return i * i;
	}*/

	//경로 내부의 이미지 파일을 읽어서 켈리브레이션 실시.
	bool initCalibData(const char* xmlName);
	bool doCalibration(const char *pPath, const char* xmlName, const char* ext = ".jpg");
	bool doCalibration(std::vector<std::string> &imgList, const char* xmlName);

	// Yolo를 이용하여 특정 이름의 영역을 추출.
	void getWeight_file(const char *_w);
	void getcfg_file(const char *_c);
	void getObjNames_file(const char *_c);
	bool findImage(const cv::Mat &mat, const char *objName, std::vector<bbox_t> &pObjRect);

	//Absolute length from camera.
	bool getAbsoluteLengthInRect(const cv::Mat src, std::vector<bbox_t> pObjRect, std::vector<std::vector<YePos3D>>& features);
	bool getAbsoluteLengthInRect(const cv::Mat src, bbox_t *pObjRect, std::vector<std::vector<YePos3D>>& features);


	// 추춘된 특정 영역만 SGBM 3D reconstruction.
	bool getSgbmInRect(const cv::Mat& src, std::vector<bbox_t>& pObject, std::vector<cv::Mat>& rtn,std::vector<bbox_t>& rtnPos);

	bool showResult(const cv::Mat& src, std::vector<cv::Mat>& rtn,std::vector<bbox_t>& rtnPos,std::vector<std::vector<YePos3D>>& features);
};

}

#endif

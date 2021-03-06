#ifndef YESTEREOCAMERA_HPP
#define YESTEREOCAMERA_HPP
#define OPENCV

#include <vector>
#include <string>
#include <sstream>
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

struct sgbmParam {
	// numDisparities -> max_disp
	// blocksize -> wsize
	int minDisparity,max_disp,P1,P2,disp12MaxDiff,preFilterCap,uniquenessRatio,speckleWindowSize,speckleRange;
	double vis_mult;
	double lambda;
	double sigma;
	int wsize;
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

	cv::Ptr<cv::Feature2D> fast;
	cv::Ptr<cv::Feature2D> brief;
	cv::Ptr<cv::DescriptorMatcher> matcher;
	cv::Mat invCamMat[2];

	bool no_display = true;		//don't display results
	bool no_downscale = true;	//force stereo matching on full-sized views to improve quality
	
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

	void initMatrix();
	//Absolute length from camera.
	bool getAbsoluteLengthInRect(const cv::Mat &src, const bbox_t &pObjRect, YePos3D &features, bbox_t &depthPos);

	bool getSgbm(const cv::Mat& src, cv::Mat& rtn,sgbmParam param);
	// 추춘된 특정 영역만 SGBM 3D reconstruction.
	bool getSgbmInRect(const cv::Mat& src, bbox_t& pObject, cv::Mat& rtn,sgbmParam param);
	
	bool showResult(const cv::Mat& src, cv::Mat &rtn, bbox_t &rtnPos, YePos3D &features, bbox_t &depthPos);
};

}

#endif

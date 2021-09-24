#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include "yeStereoCamera.hpp"

using namespace std;
using namespace cv;

class KshCalibration: public SYE::YeStereoCamera{
private:
	void getPath(string _path);
	bool doSteroProperty();
	bool writeXML();
	
	vector<cv::String> images;
	string path;
	
	vector<vector<cv::Point3f> > objpoints_right;	
	vector<vector<cv::Point3f> > objpoints_left;
	vector<vector<cv::Point2f> > imgpoints_right;		
	vector<vector<cv::Point2f> > imgpoints_left;	
	vector<cv::Point3f> objp;
	
	cv::Mat frame, gray;
	vector<cv::Point2f> corner_pts;
	cv::Mat lrImg[2];	
	
	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;	
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;
public:

	virtual bool doCalibration(const char *pPath);
	virtual bool doCalibration(std::vector<std::string> &imgList);
	
	bool doDetectAndMatch();
};

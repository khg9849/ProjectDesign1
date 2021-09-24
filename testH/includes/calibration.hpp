#include "yeStereoCamera.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <iostream>
#include <vector>
using namespace std;
class hwangCalibration: public SYE::YeStereoCamera{
	int CHECKERBOARD[2]{6,9};

public:
	virtual bool doCalibration(const char *pPath);
	virtual bool doCalibration(vector<string> &imgList);
};

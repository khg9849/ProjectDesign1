#pragma warning(disable : 4996);

#include <stdio.h>
#include <iostream>
#include <vector>
#include "calibration.h"
#include "matching.h"


using namespace cv;
using namespace std;

int main()
{
	// string path = "./resources/CameraDataSet/sCalibration-90/*.jpg";
	// cb myCB = cb();
	// myCB.calib(path, "output.xml");
	// myCB.printCalibResult();
	// myCB.readCalibResult("output.xml");

	// path="resources/CameraDataSet/s1625714490-90/1625714490.692858.jpg";
	 String path2="result_undistorted.jpg";

    // myCB.undistort2(path, path2);
	stereoMatch(path2, "result_ScoreType_FAST.jpg");

	


	return 0;
}

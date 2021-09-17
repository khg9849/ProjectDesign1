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
	string path = "./resources/CameraDataSet/sCalibration-90/*.jpg";
	cb myCB = cb();
	myCB.calib(path, "output.xml");
	//myCB.printCalibResult();
	//myCB.readCalibResult("output.xml");

	path="resources/CameraDataSet/s1625714490-90/1625714490.692858.jpg";
    myCB.undistort2(path);

	String path2="undistorted_1625714490.692858.jpg";
	stereoMatch(path2);

	return 0;
}

#include <iostream>

#include "stereo.hpp"

using namespace cv;
using namespace std;
using namespace SYE;

int main()
{
    const char *path = "../resources/CameraDataSet/sCalibration-90/*.jpg";
    stereo s1;
    s1.doCalibration(path);

    return 0;
}
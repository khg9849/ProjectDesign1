#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "yeStereoCamera.hpp"
#include <opencv2/videoio.hpp>
#define OPENCV

using namespace SYE;
using namespace std;

// ./main saveCont ../resources/
int main()
{
    cv::Mat left=cv::imread("../resources/temp/left.jpg");
    cv::Mat right=cv::imread("../resources/temp/right.jpg");
    cv::Mat res;
    cv::hconcat(left,right,res);
    cv::imwrite("result.jpg",res);
    return 0;
}


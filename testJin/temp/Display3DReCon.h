#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"
#include <iostream>
#include <string>
#include <unistd.h>


using namespace cv;
using namespace cv::ximgproc;
using namespace std;

class Display3DReCon{
private:
    String left_im,right_im,GT_path,dst_path,dst_raw_path,dst_conf_path,algo,filter;
    bool no_display,no_downscale;
    int max_disp;
    double lambda,sigma,fbs_spatial,fbs_luma,fbs_chroma,fbs_lambda,vis_mult;
    int wsize;

    VideoWriter newVideo;


public:
    Display3DReCon();
    bool init(CommandLineParser parser);
    double test3dReCon(Mat frame);
    Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);
};
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
using namespace cv;
using namespace std;


class cb{
private:
    int CHECKERBOARD[2]{6, 9};
    vector<cv::String> images;

    /* Creating vector to store vector of 3D points for each checkerboard image */
    vector< vector<cv::Point3f> > objpoints_right; // 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언
    vector< vector<cv::Point3f> > objpoints_left;  // 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언

    /* Creating vector to store vectors of 2D points for each checkerboard image */
    vector< vector<cv::Point2f> > imgpoints_right; // 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언
    vector< vector<cv::Point2f> > imgpoints_left;  // 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언

/* Defining the world coordinates for 3D points */
    vector<cv::Point3f> objp; // 월드 좌표계 선언
   cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left; // 파라미터를 구하기 위한 Mat 객체 선언
    cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

    // stereo camera calibration result
    cv::Mat R, T, E, F;

    // stereoRectify
    cv::Mat R1, R2, P1, P2, Q;

    cv::Rect validRoi[2];

public:
    cb();

    void calib(string dir,string outputFile);
    void setting(string dir);
    void writeCalibResult(string outputFile);
    void readCalibResult(string inputFile);
    void printCalibResult();

    void undistort2(string inputFile, string outputFile);
};

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <iostream>
#include <vector>

using namespace std;

int main(){
	

	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;
	cv::Size imgsize;
	cv::Mat R, T;
	cv::Mat frame;
	cv::Mat lrImage[2];
	
	
	cv::FileStorage fs("90d_calibration.xml", cv::FileStorage::READ);
	
	
	if(fs.isOpened()){
		cv::FileNode fn = fs["Size"];
		fn >> imgsize;
		fn = fs["C_LEFT"];
		fn >> cameraMatrix_left;
		fn = fs["D_LEFT"];
		fn >> distCoeffs_left;
		fn = fs["C_RIGHT"];
		fn >> cameraMatrix_right;
		fn = fs["D_RIGHT"];
		fn >> distCoeffs_right;
		fn = fs["R"];
		fn >> R;
		fn = fs["T"];
		fn >> T;
		fs.release();
	}	
	else{
		cout << "Error : can not save the extrinsic parameters" << endl;
	}
	
	
	vector<cv::String> normalimages;
	string normalImagepath = "../image/90image";
	
	cv::glob(normalImagepath, normalimages);
	cv::Mat undistortnormalImage[2]; // normal image's left and right
	char buf[256];
		for (int i = 0; i < normalimages.size(); i++){
			frame = cv::imread(normalimages[i]);
			lrImage[0] = frame(cv::Range::all(), cv::Range(0, frame.cols/2));
			lrImage[1] = frame(cv::Range::all(), cv::Range(frame.cols/2, frame.cols));
			undistort(lrImage[0], undistortnormalImage[0], cameraMatrix_left ,distCoeffs_left);
			undistort(lrImage[1], undistortnormalImage[1], cameraMatrix_left ,distCoeffs_left);
			cv::imshow("image1", undistortnormalImage[0]);
			cv::imshow("image2", undistortnormalImage[1]);
			
			
			
			cv::waitKey(0);
			sprintf(buf, "%d_Limage.jpg", i);
			cv::imwrite(buf, undistortnormalImage[0]);
			sprintf(buf, "%d_Rimage.jpg", i);
			cv::imwrite(buf, undistortnormalImage[1]);
			
		}
		
}

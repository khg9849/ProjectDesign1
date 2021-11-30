#include "yeStereoCamera.hpp"
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d_c.h>
using namespace SYE;
using std::vector;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
	cfg_file = "";
	weight_file = "";
	detector = NULL;
}
YeStereoCamera::~YeStereoCamera() {
	if(detector != NULL)
		delete detector;
}

// //경로 내부의 이미지 파일을 읽어서 켈리브레이션 실시.
bool YeStereoCamera::initCalibData(const char* xmlName) {

	cv::FileStorage fs(xmlName, cv::FileStorage::READ);

	if (fs.isOpened()) {

		fs["matCamMat1"] >> matCamMat1;
		fs["matDistCoffs1"] >> matDistCoffs1;
		fs["matCamMat2"] >> matCamMat2;
		fs["matDistCoffs2"] >> matDistCoffs2;
		fs["matR"] >> matR;
		fs["matT"] >> matT;

	}

	else {

		return false;

	}
	return true;
}

bool YeStereoCamera::doCalibration(const char* pPath, const char* xmlName, const char* ext) {
	
	int CHECKERBOARD[2]{ 6, 9 };

	/*		Load Images		*/
	
	vector<cv::String> images;
	cv::glob(pPath, images);
	std::cout << "Number of Loaded images : " << images.size() << std::endl;
	if (images.size() == 0) {
		std::cout << "Error: No images!\n" << std::endl;
		return false;
	}

	/*   Calibration ����   */

	vector<vector<cv::Point3f> > objpoints_right;		
	vector<vector<cv::Point3f> > objpoints_left;	

	vector<vector<cv::Point2f> > imgpoints_right;		
	vector<vector<cv::Point2f> > imgpoints_left;			

	vector<cv::Point3f> objp;							


	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23.8, i * 23.8, 0));
			
		}
	}

	cv::Mat frame, gray, lrImage[2];

	vector<cv::Point2f> corner_pts;	
	bool success;
	char buf[256];


	for (int i = 0; i < images.size(); i++) {
		frame = cv::imread(images[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

	
		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols / 2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols / 2, gray.cols));
		
		for (int l = 0; l < 2; l++) {
		
			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
				
				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	
			
				if ((l % 2) == 0) {
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
				}
				else {
					
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

				}
				

				if ((l % 2) == 0)
				{
					objpoints_left.push_back(objp);				
					imgpoints_left.push_back(corner_pts);		
				}
				else
				{
					objpoints_right.push_back(objp);			
					imgpoints_right.push_back(corner_pts);	
				}

			}
		}
	
	}

	cv::Mat R_left, T_left, R_right, T_right, R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), matCamMat1, matDistCoffs1, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), matCamMat2, matDistCoffs2, R_right, T_right);

	

	cv::Size imgsize = lrImage[0].size();

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize,
		matR, matT, matE, matF, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

	cv::stereoRectify(matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize, matR, matT, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);

	cv::FileStorage fs(xmlName, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "Size" << imgsize;
		fs << "matCamMat1" << matCamMat1 << "matDistCoffs1" << matDistCoffs1 << "matCamMat2" << matCamMat2 << "matDistCoffs2" << matDistCoffs2;
		fs << "matR" << matR << "matT" << matT;
		fs.release();
	}
	else {
		std::cout << "Error : can not save the extrinsic parameters" << std::endl;
		return false;
	}

	return true;


}

bool YeStereoCamera::doCalibration(vector<std::string>& imgList, const char* xmlName) {
	
	int CHECKERBOARD[2]{ 6, 9 };


	/*   Calibration ����   */

	vector<vector<cv::Point3f> > objpoints_right;			
	vector<vector<cv::Point3f> > objpoints_left;			

	vector<vector<cv::Point2f> > imgpoints_right;		
	vector<vector<cv::Point2f> > imgpoints_left;		
	vector<cv::Point3f> objp;								

	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23.8, i * 23.8, 0));

		}
	}

	cv::Mat frame, gray, lrImage[2];

	vector<cv::Point2f> corner_pts;
	bool success;
	char buf[256];


	for (int i = 0; i < imgList.size(); i++) {
		frame = cv::imread(imgList[i]);
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);


		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols / 2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols / 2, gray.cols));

		for (int l = 0; l < 2; l++) {

			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

				if ((l % 2) == 0) {
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
				}
				else {

					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

				}


				if ((l % 2) == 0)
				{
					objpoints_left.push_back(objp);
					imgpoints_left.push_back(corner_pts);
				}
				else
				{
					objpoints_right.push_back(objp);
					imgpoints_right.push_back(corner_pts);
				}

			}
		}

	}

	cv::Mat R_left, T_left, R_right, T_right, R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), matCamMat1, matDistCoffs1, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), matCamMat2, matDistCoffs2, R_right, T_right);



	cv::Size imgsize = lrImage[0].size();

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize,
		matR, matT, matE, matF, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

	cv::stereoRectify(matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize, matR, matT, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);

	cv::FileStorage fs(xmlName, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		fs << "Size" << imgsize;
		fs << "matCamMat1" << matCamMat1 << "matDistCoffs1" << matDistCoffs1 << "matCamMat2" << matCamMat2 << "matDistCoffs2" << matDistCoffs2;
		fs << "matR" << matR << "matT" << matT;
		fs.release();
	}
	else {
		std::cout << "Error : can not save the extrinsic parameters" << std::endl;
		return false;
	}

	return true;

}




// Yolo를 이용하여 특정 이름의 영역을 추출.

void YeStereoCamera::getWeight_file(const char* _w){
	weight_file = _w;
}
void YeStereoCamera::getcfg_file(const char* _c){
	cfg_file = _c;
}
void YeStereoCamera::getObjNames_file(const char* _c){
	std::string str;
	std::ifstream file(_c);
	if (file.is_open()) {
		while (file) {
			getline(file, str);
			objNames.push_back(str);
			//std::cout << str << std::endl;
		}
		file.close();
	} else {
		std::cout << "file open fail" << std::endl;
		return;
	}
}

bool YeStereoCamera::findImage(const cv::Mat &mat, const char* objName, std::vector<bbox_t> &pObjRect) {

	if(weight_file == ""){
		perror("findImage error : failed to find weight file!");
        return false;
	}
	if(cfg_file == ""){
		perror("findImage error : failed to find cfg file!");
        return false;
	}
	if(objName == "" || strcmp(objName, "test") == 0){
		perror("findImage error : failed to understand objName!");
        return false;
	}
	if(detector == NULL){
		if((detector = new Detector(cfg_file, weight_file)) == NULL){
    	    perror("findImage error : failed to make detector object!");
			return false;
    	}
	}


    cv::Mat mat_left(mat(cv::Range::all(), cv::Range(0, mat.cols/2)));
    cv::Mat mat_right(mat(cv::Range::all(), cv::Range(mat.cols/2, mat.cols)));

	//yolo_v2_class.hpp
	//std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
	std::vector<bbox_t> detection_left, detection_right;
	detection_left = detector->detect(mat_left);
	detection_right = detector->detect(mat_right);

	// if(detection_left.size() != detection_right.size()){
	// 	perror("findImage error : dismatch number of finding object with left and right in photo");
	// 	return false;
	// }

	//did you know how to match between obj_id and objName?
	//*.names
	double threshold = 0.2;
    for(size_t i = 0; i < detection_left.size(); ++i){
		if(objNames[detection_left[i].obj_id]==objName && detection_left[i].prob >= threshold &&
		objNames[detection_right[i].obj_id]==objName && detection_right[i].prob >= threshold){
		
			pObjRect.push_back(detection_left[i]);
			detection_right[i].x += mat.cols/2;
			pObjRect.push_back(detection_right[i]);
				
		}
	}
    return true;
}

void YeStereoCamera::initMatrix(){
	fast = cv::FastFeatureDetector::create();
	brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	invCamMat[0] = matCamMat1.inv();
	invCamMat[1] = matCamMat2.inv();
}

bool compare(YePos3D a, YePos3D b){
	return a.z<b.z;
}
//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat &src, std::vector<bbox_t> &pObjRect, std::vector<YePos3D> &features, std::vector<bbox_t> &depthPos) {
	if(pObjRect.size()<1){
		std::cout<<"obj size is 0\n";
		return false;
	}
	if(matCamMat1.rows != 3){
		std::cout<<"can't read camera calibration matrix\n";
		return false;
	}

	const cv::Mat *gray;
	cv::Mat temp;
	if(src.type()/8){
		cv::cvtColor(src, temp, cv::COLOR_RGB2GRAY);
		gray = &temp;
	}
	else{
		gray = &src;
	}

	for(int i = 0; i < pObjRect.size(); i += 2){
		bbox_t *leftBbox, *rightBbox;
		if(pObjRect[i].x < pObjRect[i+1].x){
			leftBbox = &pObjRect[i];
			rightBbox = &pObjRect[i+1];
		}
		else{
			leftBbox = &pObjRect[i+1];
			rightBbox = &pObjRect[i];
		}

		cv::Mat pic[2], dc[2];
		std::vector<cv::KeyPoint> kp[2];
		std::vector<cv::DMatch> matches;

		pic[0] = (*gray)(cv::Range(leftBbox->y, std::min((int)(leftBbox->y + leftBbox->h), 960)), cv::Range(leftBbox->x, std::min((int)(leftBbox->x + leftBbox->w), 1280)));
		pic[1] = (*gray)(cv::Range(rightBbox->y, std::min((int)(rightBbox->y + rightBbox->h), 960)), cv::Range(rightBbox->x, std::min((int)(rightBbox->x + rightBbox->w), 2560)));

		fast->detect(pic[0], kp[0], pic[1]);
		brief->compute(pic[0], kp[0], dc[0]);
		fast->detect(pic[1], kp[1], pic[0]);
		brief->compute(pic[1], kp[1], dc[1]);
		matcher->match(dc[0], dc[1], matches);

		bool findPos = false;
		int dfRight = leftBbox->w-rightBbox->w;

		std::vector<YePos3D> feature_temp[2];
		for(int j = 0; j < matches.size(); j++){
			YePos3D temp[2];
			//int dif = (int)kp[0][matches[j].queryIdx].pt.x-(int)kp[1][matches[j].trainIdx].pt.x;
			//if((dif<5 && dif>-5)||(dif<dfRight+5 && dif>dfRight-5)){
				temp[0].x = invCamMat[0].at<double>(0,0)*((int)(kp[0][matches[j].queryIdx].pt.x)+leftBbox->x)+invCamMat[0].at<double>(0,2);
				temp[1].x = invCamMat[1].at<double>(0,0)*((int)(kp[1][matches[j].trainIdx].pt.x)+rightBbox->x-(*gray).cols/2)+invCamMat[1].at<double>(0,2);
			
				temp[0].z = -matT.at<double>(0,0)/(temp[0].x-temp[1].x);
				temp[1].z = temp[0].z;

				temp[0].x = temp[0].x*temp[0].z;
				temp[0].y = (invCamMat[0].at<double>(1,1)*((int)(kp[0][matches[j].queryIdx].pt.y)+leftBbox->y)+invCamMat[0].at<double>(1,2))*temp[0].z;

				temp[1].x = temp[1].x*temp[1].z;
				temp[1].y = (invCamMat[1].at<double>(1,1)*((int)(kp[1][matches[j].trainIdx].pt.y)+rightBbox->y)+invCamMat[1].at<double>(1,2))*temp[1].z;

				temp[0].idx = j;
				temp[1].idx = j;

				feature_temp[0].push_back(temp[0]);
				feature_temp[1].push_back(temp[1]);

			//	bbox_t posTemp;
			//	posTemp.x = leftBbox->x+(int)kp[0][matches[j].queryIdx].pt.x;
			//	posTemp.y = leftBbox->y+(int)kp[0][matches[j].queryIdx].pt.y;

			//	depthPos.push_back(posTemp);
			//	findPos = true;
			//}
		}
		
		std::sort(feature_temp[0].begin(), feature_temp[0].end(), compare);

		int pre = feature_temp[0][0].z;
		int count = 1, maxCount = 0, maxIdx = 0;
		for(int j = 1; j < feature_temp[0].size(); j++){
			if(feature_temp[0][j].z == pre){
				count++;
			}
			else{
				pre = feature_temp[0][j].z;
				if(count > maxCount){
					maxCount = count;
					maxIdx = j-1;
				}
				count = 1;
			}
		}
		if(count>maxCount){
			maxIdx = feature_temp[0].size()-1;
		}
		features.push_back(feature_temp[0][maxIdx]);
		bbox_t posTemp;
		posTemp.x = leftBbox->x+(int)kp[0][matches[feature_temp[0][maxIdx].idx].queryIdx].pt.x;
		posTemp.y = leftBbox->y+(int)kp[0][matches[feature_temp[0][maxIdx].idx].queryIdx].pt.y;
		
		depthPos.push_back(posTemp);
	}

	return true;
}

bool YeStereoCamera::getSgbm(const cv::Mat& src, cv::Mat& rtn) {
	bool no_display = true;		//don't display results
	bool no_downscale = true;	//force stereo matching on full-sized views to improve quality
	double vis_mult = 8;		//coefficient used to scale disparity map visualizations
	int max_disp = 16;			//parameter of stereo matching
	double lambda = 8000;		//parameter of wls post-filtering
	double sigma  = 1.5;		//parameter of wls post-filtering
	int wsize=3;
	
	if(max_disp<=0 || max_disp%16!=0)
	{
		std::cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
		return false;
	}
	if(wsize<=0 || wsize%2!=1)
	{
		std::cout<<"Incorrect window_size value: it should be positive and odd";
		return false;
	}
	cv::Mat mat_left=src(cv::Range::all(), cv::Range(0, src.cols/2)).clone();
    cv::Mat mat_right=src(cv::Range::all(), cv::Range(src.cols/2, src.cols)).clone();


	cv::Mat left_for_matcher, right_for_matcher;
	cv::Mat left_disp,right_disp;
	cv::Mat filtered_disp,solved_disp,solved_filtered_disp;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	if(!no_downscale)
		{
			max_disp/=2;
			if(max_disp%16!=0)
				max_disp += 16-(max_disp%16);
			cv::resize(mat_left ,left_for_matcher ,cv::Size(),0.5,0.5, cv::INTER_LINEAR_EXACT);
		}
	else{
			left_for_matcher  = mat_left.clone();
			right_for_matcher = mat_right.clone();
	}

	// sgbm
	cv::Ptr<cv::StereoSGBM> left_matcher  = cv::StereoSGBM::create(0,max_disp,wsize);
	left_matcher->setP1(24*wsize*wsize);
	left_matcher->setP2(96*wsize*wsize);
	left_matcher->setPreFilterCap(63);
	left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
	wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
	cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

	left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
	right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

	// filtering
	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	wls_filter->filter(left_disp,mat_left,filtered_disp,right_disp);
	cv::Mat filtered_disp_vis;
	cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
	rtn=filtered_disp_vis.clone(); //filtered image

	//visualization
	if(!no_display){
		cv::namedWindow("filtered disparity", cv::WINDOW_AUTOSIZE);
		cv::imshow("filtered disparity", filtered_disp_vis);
		cv::waitKey(0);
	}
	return true;
}

// 추춘된 특정 영역만 SGBM 3D reconstruction.

bool YeStereoCamera::getSgbmInRect(const cv::Mat& src, std::vector<bbox_t>& pObject, std::vector<cv::Mat>& rtn,std::vector<bbox_t>& rtnPos) {
	
	int stride=src.cols/2;
	cv::Mat detected;
	int nx,ny,nw,nh;
	bbox_t *bbox1,*bbox2;
	
	for(int i=0;i<pObject.size();i+=2){
		if(pObject[i].x<pObject[i+1].x){
			bbox1=&pObject[i];
			bbox2=&pObject[i+1];
		}
		else{
			bbox1=&pObject[i+1];
			bbox2=&pObject[i];
		}
		bbox_t pos;
		pos.x=std::min((*bbox1).x,(*bbox2).x-stride);
		pos.y=std::min((*bbox1).y,(*bbox2).y);
		pos.w=std::max((*bbox1).x+(*bbox1).w,(*bbox2).x-stride+(*bbox2).w)-pos.x;
		pos.h=std::max((*bbox1).y+(*bbox1).h,(*bbox2).y+(*bbox2).h)-pos.y;
		if(pos.x+pos.w>=stride) pos.w=stride-pos.x;
		if(pos.y+pos.h>=src.rows) pos.h=src.rows-pos.y;
		rtnPos.push_back(pos);

		cv::Mat left  = src(cv::Rect(pos.x,pos.y,pos.w,pos.h)).clone();
		cv::Mat right = src(cv::Rect(pos.x+stride,pos.y,pos.w,pos.h)).clone();
		hconcat(left, right,detected);
		getSgbm(detected,detected);
		rtn.push_back(detected); //filtered image
	}
	return true;
}

bool YeStereoCamera::showResult(const cv::Mat& src, std::vector<cv::Mat>& rtn,std::vector<bbox_t>& rtnPos,std::vector<YePos3D>& features, std::vector<bbox_t> &depthPos){
	for(int i=0;i<rtn.size();i++){
		cv::Mat res=src.clone();
		cv::imwrite("1.jpg", res);
		cv::rectangle(res, cv::Rect(rtnPos[i].x,rtnPos[i].y,rtnPos[i].w,rtnPos[i].h), cv::Scalar(0, 0, 255), 3, 8, 0);
		cv::imwrite("2.jpg", res);
		cv::Mat ROI=res.rowRange(rtnPos[i].y,rtnPos[i].y+rtnPos[i].h).colRange(rtnPos[i].x,rtnPos[i].x+rtnPos[i].w);
		cv::Mat img_rgb(rtn[i].size(), CV_8UC3);
		cv::cvtColor(rtn[i], img_rgb, CV_GRAY2RGB);
		img_rgb.copyTo(ROI);

		cv::imwrite("3.jpg", res);
		//((int)(rtnPos[i].x+rtnPos[i].w)/2,(int)(rtnPos[i].y+rtnPos[i].h)/2)
		//cv::rectangle(res, cv::Rect(rtnPos[i].x,rtnPos[i].y,rtnPos[i].w,rtnPos[i].h), cv::Scalar(0, 0, 255), 3, 8, 0);
		//cv::imwrite("2.jpg", res);
		cv::line(res,cv::Point(depthPos[i].x, depthPos[i].y),cv::Point(depthPos[i].x, depthPos[i].y),cv::Scalar(0, 0, 255),5,3);
		//cv::putText(res, std::to_string(features[0][i].z), cv::Point(rtnPos[i].x+rtnPos[i].w/2,rtnPos[i].y+rtnPos[i].h/2), 1, 2, cv::Scalar(255, 255, 0), 1, 8);
		cv::putText(res, "117.972603", cv::Point(depthPos[i].x, depthPos[i].y), 1, 2, cv::Scalar(255, 255, 0), 1, 8);
		cv::imwrite("4.jpg", res);

		cv::imshow("res",res);
		cv::waitKey(0);
	}
	return true;
}
/*--------------------------------------------------4 팀 화 이 팅 ! ! ! ,,----------------------------------------------------*/

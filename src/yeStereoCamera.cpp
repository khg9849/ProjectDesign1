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
	int leftIdx=-1, rightIdx=-1;

    for(size_t i = 0; i < detection_left.size(); ++i){
		if(objNames[detection_left[i].obj_id]==objName && detection_left[i].prob >= threshold){
			leftIdx = i;
			break;
		}
	}
	for(size_t i = 0; i < detection_right.size(); ++i){
		if(objNames[detection_right[i].obj_id]==objName && detection_right[i].prob >= threshold){
			rightIdx = i;
			break;
		}
	}
	if(leftIdx==-1||rightIdx==-1){
		printf("findImage is failed. leftIdx or rightIdx is -1\n");
		return false;
	}

	bbox_t pos;
	pos.x = std::min(detection_left[leftIdx].x, detection_right[rightIdx].x);
	pos.y = std::min(detection_left[leftIdx].y, detection_right[rightIdx].y);
	pos.w = std::max(detection_left[leftIdx].x + detection_left[leftIdx].w, detection_right[rightIdx].x + detection_right[rightIdx].w) - pos.x;
	pos.h = std::max(detection_left[leftIdx].y + detection_left[leftIdx].h, detection_right[rightIdx].y + detection_right[rightIdx].h) - pos.y;
	pos.obj_id = detection_left[leftIdx].obj_id;
	pos.prob = detection_left[leftIdx].prob;
	pObjRect.push_back(pos);
	
    return true;
}

void YeStereoCamera::initMatrix(){
	fast = cv::FastFeatureDetector::create();
	brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	invCamMat[0] = matCamMat1.inv();
	invCamMat[1] = matCamMat2.inv();
}

//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat &src, const bbox_t &pObjRect, YePos3D &features, bbox_t &depthPos) {
	if(src.empty()){
		std::cout<<"image matrix is empty\n";
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

	cv::Mat pic[2], dc[2];
	std::vector<cv::KeyPoint> kp[2];
	std::vector<cv::DMatch> matches;

	pic[0] = (*gray)(cv::Range(std::min((int)(pObjRect.y), src.rows-1), std::min((int)(pObjRect.y + pObjRect.h), src.rows)), cv::Range(std::min((int)(pObjRect.x), src.cols/2-1), std::min((int)(pObjRect.x + pObjRect.w), src.cols/2)));
	pic[1] = (*gray)(cv::Range(std::min((int)(pObjRect.y), src.rows-1), std::min((int)(pObjRect.y + pObjRect.h), src.rows)), cv::Range(std::min((int)(pObjRect.x)+src.cols/2, src.cols-1), std::min((int)(pObjRect.x + pObjRect.w)+src.cols/2, src.cols)));

	fast->detect(pic[0], kp[0], pic[1]);
	brief->compute(pic[0], kp[0], dc[0]);
	fast->detect(pic[1], kp[1], pic[0]);
	brief->compute(pic[1], kp[1], dc[1]);
	matcher->match(dc[0], dc[1], matches);

	if(matches.size() == 0){
		std::cout<<"feature match is zero\n";
		return false;
	}

	double baseline = matT.at<double>(0)/10;
	double FOV = 83;
	double PI = 3.14159265358979;
	double f_pixel = ((*gray).cols*0.5*0.5)/tan(FOV*0.5*PI/180);

	std::vector<YePos3D> feature_temp;
	for(int i = 0; i < matches.size(); i++){
		YePos3D temp;

		temp.x = invCamMat[0].at<double>(0,0)*((int)(kp[0][matches[i].queryIdx].pt.x)+pObjRect.x)+invCamMat[0].at<double>(0,2);
		double rightX = invCamMat[1].at<double>(0,0)*((int)(kp[1][matches[i].trainIdx].pt.x)+pObjRect.x)+invCamMat[1].at<double>(0,2);

		temp.z = -matT.at<double>(0,0)/(temp.x-rightX); 

		temp.x = temp.x*temp.z;
		temp.y = (invCamMat[0].at<double>(1,1)*((int)(kp[0][matches[i].queryIdx].pt.y)+pObjRect.y)+invCamMat[0].at<double>(1,2))*temp.z;

		temp.z = -(baseline*f_pixel)/((int)(kp[0][matches[i].queryIdx].pt.x)-(int)(kp[1][matches[i].trainIdx].pt.x));

		feature_temp.push_back(temp);
	}

	std::vector<int> index;
	for(int i = 0; i < feature_temp.size(); i++){
		index.push_back(i);
	}
	std::sort(index.begin(), index.end(),
			[&feature_temp](int f1, int f2){
				return feature_temp[f1].z < feature_temp[f2].z;
			}
			);

	double pre = feature_temp[index[0]].z;
	int count = 1, maxCount = 0, maxIdx = 0;
	
	for(int i = 1; i < feature_temp.size(); i++){
		if(feature_temp[index[i]].z == pre){
			count++;
		}
		else{
			pre = feature_temp[index[i]].z;
			if(count > maxCount){
				maxCount = count;
				maxIdx = i-1;
			}
			count = 1;
		}
	}
	if(count > maxCount){
		maxCount = count;
		maxIdx = feature_temp.size()-1;
	}
	
	int idx = index[(maxIdx+(maxIdx-maxCount+1))/2];
	features.x = feature_temp[idx].x;
	features.y = feature_temp[idx].y;
	features.z = feature_temp[idx].z;

	depthPos.x = pObjRect.x+(int)kp[0][matches[idx].queryIdx].pt.x;
	depthPos.y = pObjRect.y+(int)kp[0][matches[idx].queryIdx].pt.y;

	return true;
}

bool YeStereoCamera::getSgbm(const cv::Mat& src, cv::Mat& rtn,sgbmParam param) {

	cv::Mat mat_left=src(cv::Range::all(), cv::Range(0, src.cols/2)).clone();
    cv::Mat mat_right=src(cv::Range::all(), cv::Range(src.cols/2, src.cols)).clone();

	cv::Mat left_for_matcher, right_for_matcher;
	cv::Mat left_disp,right_disp;
	cv::Mat filtered_disp,solved_disp,solved_filtered_disp;
	cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	if(!no_downscale)
		{
			param.max_disp/=2;
			if(param.max_disp%16!=0)
				param.max_disp += 16-(param.max_disp%16);
			cv::resize(mat_left ,left_for_matcher ,cv::Size(),0.5,0.5, cv::INTER_LINEAR_EXACT);
		}
	else{
			left_for_matcher  = mat_left.clone();
			right_for_matcher = mat_right.clone();
	}

	 cv::Ptr<cv::StereoSGBM> left_matcher  = cv::StereoSGBM::create(0,param.max_disp,param.wsize);
	
	 left_matcher->setNumDisparities(param.max_disp);
	 left_matcher->setP1(24*param.wsize*param.wsize);
	 left_matcher->setP2(96*param.wsize*param.wsize);
	 left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
	left_matcher->setBlockSize(param.wsize);
	left_matcher->setMinDisparity(param.minDisparity);
	left_matcher->setDisp12MaxDiff(param.disp12MaxDiff);
	left_matcher->setPreFilterCap(param.preFilterCap);
	left_matcher->setUniquenessRatio(param.uniquenessRatio);
	left_matcher->setSpeckleWindowSize(param.speckleWindowSize);
	left_matcher->setSpeckleRange(param.speckleRange);

	wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
	cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

	left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
	right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);

	// filtering
	wls_filter->setLambda(param.lambda);
	wls_filter->setSigmaColor(param.sigma);
	wls_filter->filter(left_disp,mat_left,filtered_disp,right_disp);
	cv::Mat filtered_disp_vis;
	cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,param.vis_mult);
	rtn=filtered_disp_vis.clone(); //filtered image

	//visualization
	if(!no_display){
		cv::namedWindow("filtered disparity", cv::WINDOW_AUTOSIZE);
		cv::imshow("filtered disparity", filtered_disp_vis);
		cv::waitKey(0);
	}

	return true;
}

// 특정 영역만 SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat& src, bbox_t& pObject,cv::Mat& rtn,sgbmParam param) {
	cv::Mat left  = src(cv::Rect(pObject.x,pObject.y,pObject.w,pObject.h));
	cv::Mat right = src(cv::Rect(pObject.x+src.cols/2,pObject.y,pObject.w,pObject.h));
	hconcat(left, right,rtn);
	getSgbm(rtn,rtn,param);
	return true;
}

bool YeStereoCamera::showResult(const cv::Mat& src, cv::Mat &rtn, bbox_t &rtnPos, YePos3D &features, bbox_t &depthPos){

	cv::Mat res=src.clone();
	cv::Mat ROI=res.rowRange(rtnPos.y, rtnPos.y+rtnPos.h).colRange(rtnPos.x, rtnPos.x+rtnPos.w);
	cv::Mat img_rgb(rtn.size(), CV_8UC3);
	cv::cvtColor(rtn, img_rgb, CV_GRAY2RGB);
	img_rgb.copyTo(ROI);

	cv::rectangle(res, cv::Rect(rtnPos.x,rtnPos.y,rtnPos.w,rtnPos.h), cv::Scalar(0, 0, 255), 3, 8, 0);
	cv::line(res,cv::Point(depthPos.x, depthPos.y),cv::Point(depthPos.x, depthPos.y),cv::Scalar(0, 0, 255),5,3);
	cv::putText(res, std::to_string(features.z), cv::Point(depthPos.x, depthPos.y), 1, 5, cv::Scalar(255, 255, 0), 3, 8);
		
	rtn=res.clone();

	return true;
}
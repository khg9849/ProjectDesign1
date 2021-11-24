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
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));
			
		}
	}

	cv::Mat frame, gray, lrImage[2];

	vector<cv::Point2f> corner_pts;	
	bool success;
	char buf[256];
	int index = 0;


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
					for (int k = 0; k < corner_pts.size(); k++)
					{
						corner_pts[k].x += lrImage[l].cols;
					}
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
		//cv::imshow("Image", frame);

		index++;

		cv::waitKey(0);	
	}
	//cv::destroyAllWindows(); 

	cv::Mat R_left, T_left, R_right, T_right;
	cv::Mat matR, matT, matE, matF, matCamMat1, matCamMat2, matDistCoffs1, matDistCoffs2;

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), matCamMat1, matDistCoffs1, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), matCamMat2, matDistCoffs2, R_right, T_right);

	

	cv::Size imgsize(gray.rows, gray.cols / 2);

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize,
		matR, matT, matE, matF, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

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
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	
		}
	}

	cv::Mat frame, gray, lrImage[2];

	vector<cv::Point2f> corner_pts;
	bool success;
	char buf[256];
	int index = 0;


	for (int i = 0; i < imgList.size(); i++){
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
					for (int k = 0; k < corner_pts.size(); k++)
					{
						corner_pts[k].x += lrImage[l].cols;
					}
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
		cv::imshow("Image", frame);

		index++;

		cv::waitKey(0);
	}
	cv::destroyAllWindows();


	cv::Mat R_left, T_left, R_right, T_right;

	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), matCamMat1, matDistCoffs1, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), matCamMat2, matDistCoffs2, R_right, T_right);


	cv::Size imgsize(gray.rows, gray.cols / 2);

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		matCamMat1, matDistCoffs1, matCamMat2, matDistCoffs2, imgsize,
		matR, matT, matE, matF, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

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

void YeStereoCamera::getWeight_file(std::string _w){
	weight_file = _w;
}
void YeStereoCamera::getcfg_file(std::string _c){
	cfg_file = _c;
}
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, std::vector<bbox_t> pObjRect) {

	if(weight_file == ""){
		perror("findImage error : failed to find weight file!");
        return false;
	}
	if(cfg_file == ""){
		perror("findImage error : failed to find cfg file!");
        return false;
	}
	if(objName == ""){
		perror("findImage error : failed to understand objName!");
        return false;
	}
	if(detector == NULL){
		if((detector = new Detector(cfg_file, weight_file)) == NULL){
    	    perror("findImage error : failed to make detector object!");
			return false;
    	}
	}

	std::vector<bbox_t> detection_left, detection_right;
	size_t detectionSize;
	double threshold = 0.7;

    cv::Mat mat_left = mat(cv::Range::all(), cv::Range(0, mat.cols/2));
    cv::Mat mat_right = mat(cv::Range::all(), cv::Range(mat.cols/2, mat.cols));

	//yolo_v2_class.hpp
	//std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
	detection_left = detector->detect(mat_left);
	detection_right = detector->detect(mat_right);
    detectionSize = detection_left.size();

	//did you know how to match between obj_id and objName?
	//*.names
    for(size_t i = 0; i < detectionSize; ++i){
		if(	detection_left[i].obj_id == 0 && detection_left[i].prob >= threshold &&
			detection_right[i].obj_id == 0 && detection_right[i].prob >= threshold &&
			detection_left[i].obj_id == detection_right[i].obj_id){
				pObjRect.push_back(detection_left[i]);
				pObjRect.push_back(detection_right[i]);
			}
	}

    return true;
}
bool YeStereoCamera::findImage(const cv::Mat mat, const char *objName, bbox_t *pObjRect){
	
	if(weight_file == ""){
		perror("findImage error : failed to find weight file!");
        return false;
	}
	if(cfg_file == ""){
		perror("findImage error : failed to find cfg file!");
        return false;
	}
	if(objName == ""){
		perror("findImage error : failed to understand objName!");
        return false;
	}
	if(detector == NULL){
		if((detector = new Detector(cfg_file, weight_file)) == NULL){
    	    perror("findImage error : failed to make detector object!");
			return false;
    	}
	}

	std::vector<bbox_t> detection_left, detection_right;
	size_t detectionSize;
	double threshold = 0.7;

    cv::Mat mat_left = mat(cv::Range::all(), cv::Range(0, mat.cols/2));
    cv::Mat mat_right = mat(cv::Range::all(), cv::Range(mat.cols/2, mat.cols));

	//yolo_v2_class.hpp
	//std::vector<bbox_t> detect(cv::Mat mat, float thresh = 0.2, bool use_mean = false)
	detection_left = detector->detect(mat_left);
	detection_right = detector->detect(mat_right);
    detectionSize = detection_left.size();

	//did you know how to match between obj_id and objName?
	//*.names
    for(size_t i = 0; i < detectionSize; ++i){
		if(	detection_left[i].obj_id == 0 && detection_left[i].prob >= threshold &&
			detection_right[i].obj_id == 0 && detection_right[i].prob >= threshold &&
			detection_left[i].obj_id == detection_right[i].obj_id){
				pObjRect[i * 2] = detection_left[i];
				pObjRect[i * 2 + 1] = detection_right[i];
			}
	}

	findImageSize = detectionSize * 2;

    return true;
}

//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, std::vector<bbox_t> pObjRect, std::vector<std::vector<YePos3D>>& features) {
	
	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);
	
	cv::Mat invMat[2];
	invMat[0] = matCamMat1.inv();
	invMat[1] = matCamMat2.inv();

	for(int i = 0; i < pObjRect.size(); i+=2){
		int left, right;
		if(pObjRect[i].x < pObjRect[i+1].x){
			left = i;
			right = i+1;
		}
		else{
			left = i+1;
			right = i;
		}

		cv::Mat pic[2], dc[2];
		std::vector<cv::KeyPoint> kp[2];
		std::vector<cv::DMatch> matches;

		pic[0] = gray(cv::Range(pObjRect[left].y, pObjRect[left].h), cv::Range(pObjRect[left].x, pObjRect[left].w));
		pic[1] = gray(cv::Range(pObjRect[right].y, pObjRect[right].h), cv::Range(pObjRect[right].x, pObjRect[right].w));

		fast->detect(pic[0], kp[0], pic[1]);
		brief->compute(pic[0], kp[0], dc[0]);
		fast->detect(pic[1], kp[1], pic[0]);
		brief->compute(pic[1], kp[1], dc[1]);
		matcher->match(dc[0], dc[1], matches);

		std::vector<YePos3D> feature_temp[2];
		for(int j = 0; j < matches.size(); j++){
			YePos3D temp[2];
		
			temp[0].x = invMat[0].at<double>(0,0)*((int)(kp[0][matches[j].queryIdx].pt.x)+pObjRect[left].x)+invMat[0].at<double>(0,2);
			temp[1].x = invMat[1].at<double>(0,0)*((int)(kp[1][matches[j].trainIdx].pt.x)+pObjRect[right].x-gray.cols/2)+invMat[1].at<double>(0,2);
			
			temp[0].z = -matT.at<double>(0,0)/(temp[0].x-temp[1].x);
			temp[1].z = temp[0].z;

			temp[0].x = temp[0].x*temp[0].z;
			temp[0].y = (invMat[0].at<double>(1,1)*((int)(kp[0][matches[j].queryIdx].pt.y)+pObjRect[left].y)+invMat[0].at<double>(1,2))*temp[0].z;

			temp[1].x = temp[1].x*temp[1].z;
			temp[1].y = (invMat[1].at<double>(1,1)*((int)(kp[1][matches[j].trainIdx].pt.y)+pObjRect[right].y)+invMat[1].at<double>(1,2))*temp[1].z;

			feature_temp[0].push_back(temp[0]);
			feature_temp[1].push_back(temp[1]);
		}

		features.push_back(feature_temp[0]);
		features.push_back(feature_temp[1]);
	}
}

bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, int objNum, std::vector<std::vector<YePos3D>>& features) {

	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);

	cv::Mat invMat[2];
	invMat[0] = matCamMat1.inv();
	invMat[1] = matCamMat2.inv();

	for (int i = 0; i < objNum; i += 2) {
		int left, right;
		if (pObjRect[i].x < pObjRect[i + 1].x) {
			left = i;
			right = i + 1;
		}
		else {
			left = i + 1;
			right = i;
		}

		cv::Mat pic[2], dc[2];
		std::vector<cv::KeyPoint> kp[2];
		std::vector<cv::DMatch> matches;

		pic[0] = gray(cv::Range(pObjRect[left].y, pObjRect[left].h), cv::Range(pObjRect[left].x, pObjRect[left].w));
		pic[1] = gray(cv::Range(pObjRect[right].y, pObjRect[right].h), cv::Range(pObjRect[right].x, pObjRect[right].w));

		fast->detect(pic[0], kp[0], pic[1]);
		brief->compute(pic[0], kp[0], dc[0]);
		fast->detect(pic[1], kp[1], pic[0]);
		brief->compute(pic[1], kp[1], dc[1]);
		matcher->match(dc[0], dc[1], matches);

		std::vector<YePos3D> feature_temp[2];
		for(int j = 0; j < matches.size(); j++){
			YePos3D temp[2];
		
			temp[0].x = invMat[0].at<double>(0,0)*((int)(kp[0][matches[j].queryIdx].pt.x)+pObjRect[left].x)+invMat[0].at<double>(0,2);
			temp[1].x = invMat[1].at<double>(0,0)*((int)(kp[1][matches[j].trainIdx].pt.x)+pObjRect[right].x-gray.cols/2)+invMat[1].at<double>(0,2);
			
			temp[0].z = -matT.at<double>(0,0)/(temp[0].x-temp[1].x);
			temp[1].z = temp[0].z;

			temp[0].x = temp[0].x*temp[0].z;
			temp[0].y = (invMat[0].at<double>(1,1)*((int)(kp[0][matches[j].queryIdx].pt.y)+pObjRect[left].y)+invMat[0].at<double>(1,2))*temp[0].z;

			temp[1].x = temp[1].x*temp[1].z;
			temp[1].y = (invMat[1].at<double>(1,1)*((int)(kp[1][matches[j].trainIdx].pt.y)+pObjRect[right].y)+invMat[1].at<double>(1,2))*temp[1].z;

			feature_temp[0].push_back(temp[0]);
			feature_temp[1].push_back(temp[1]);
		}

		features.push_back(feature_temp[0]);
		features.push_back(feature_temp[1]);
	}
}

// 추춘된 특정 영역만 SGBM 3D reconstruction.
bool YeStereoCamera::getSgbmInRect(const cv::Mat src, bbox_t* pObject, int size, cv::Mat* rtn) {
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

	int idx=0;
	int stride=src.cols/2;
	bbox_t bbox1,bbox2;
	for(int i=0;i<size;i+=2){
		if(pObject[i].x<pObject[i+1].x){
			bbox1=pObject[i];
			bbox2=pObject[i+1];
		}
		else{
			bbox1=pObject[i+1];
			bbox2=pObject[i];
		}

		cv::Mat detected;
		int nx,ny,nw,nh;

		nx=std::min(bbox1.x,bbox2.x-stride);
		ny=std::min(bbox1.y,bbox2.y);
		nw=std::max(bbox1.x+bbox1.w,bbox2.x-stride+bbox2.w)-nx;
		nh=std::max(bbox1.y+bbox1.h,bbox2.y+bbox2.h)-ny;

		// cv::Mat srcCpy=src.clone();
		// cv::rectangle(srcCpy,cv::Rect(nx,ny,nw,nh), cv::Scalar(0, 255, 0), 5, 8, 0);
		// cv::rectangle(srcCpy,cv::Rect(nx+stride,ny,nw,nh), cv::Scalar(0, 255, 0), 5, 8, 0);
		// imshow("detected Rect",srcCpy);
		// cv::waitKey(0);

		cv::Mat left  = src(cv::Rect(nx,ny,nw,nh));
		cv::Mat right = src(cv::Rect(nx+stride,ny,nw,nh));
		hconcat(left, right,detected);
		// imshow("detected",detected);
		// cv::waitKey(0);

		cv::Mat left_for_matcher, right_for_matcher;
		cv::Mat left_disp,right_disp;
		cv::Mat filtered_disp,solved_disp,solved_filtered_disp;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

		if(!no_downscale)
            {
                max_disp/=2;
                if(max_disp%16!=0)
                    max_disp += 16-(max_disp%16);
                cv::resize(left ,left_for_matcher ,cv::Size(),0.5,0.5, cv::INTER_LINEAR_EXACT);
            }
        else{
                left_for_matcher  = left.clone();
                right_for_matcher = right.clone();
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
		wls_filter->filter(left_disp,left,filtered_disp,right_disp);
		cv::Mat filtered_disp_vis;
		cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
		rtn[idx++]=filtered_disp_vis; //filtered image

		//visualization
		if(!no_display){
            cv::namedWindow("filtered disparity", cv::WINDOW_AUTOSIZE);
            cv::imshow("filtered disparity", filtered_disp_vis);
            cv::waitKey(0);
		}
	}
	return true;
}

bool YeStereoCamera::getSgbmInRect(const cv::Mat src, std::vector<bbox_t> pObject, std::vector<cv::Mat>* rtn) {
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

	int idx=0;
	int stride=src.cols/2;
	bbox_t bbox1,bbox2;
	for(int i=0;i<pObject.size();i+=2){
		if(pObject[i].x<pObject[i+1].x){
			bbox1=pObject[i];
			bbox2=pObject[i+1];
		}
		else{
			bbox1=pObject[i+1];
			bbox2=pObject[i];
		}

		cv::Mat detected;
		int nx,ny,nw,nh;

		nx=std::min(bbox1.x,bbox2.x-stride);
		ny=std::min(bbox1.y,bbox2.y);
		nw=std::max(bbox1.x+bbox1.w,bbox2.x-stride+bbox2.w)-nx;
		nh=std::max(bbox1.y+bbox1.h,bbox2.y+bbox2.h)-ny;

		// cv::Mat srcCpy=src.clone();
		// cv::rectangle(srcCpy,cv::Rect(nx,ny,nw,nh), cv::Scalar(0, 255, 0), 5, 8, 0);
		// cv::rectangle(srcCpy,cv::Rect(nx+stride,ny,nw,nh), cv::Scalar(0, 255, 0), 5, 8, 0);
		// imshow("detected Rect",srcCpy);
		// cv::waitKey(0);

		cv::Mat left  = src(cv::Rect(nx,ny,nw,nh));
		cv::Mat right = src(cv::Rect(nx+stride,ny,nw,nh));
		hconcat(left, right,detected);
		// imshow("detected",detected);
		// cv::waitKey(0);

		cv::Mat left_for_matcher, right_for_matcher;
		cv::Mat left_disp,right_disp;
		cv::Mat filtered_disp,solved_disp,solved_filtered_disp;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

		if(!no_downscale)
            {
                max_disp/=2;
                if(max_disp%16!=0)
                    max_disp += 16-(max_disp%16);
                cv::resize(left ,left_for_matcher ,cv::Size(),0.5,0.5, cv::INTER_LINEAR_EXACT);
            }
        else{
                left_for_matcher  = left.clone();
                right_for_matcher = right.clone();
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
		wls_filter->filter(left_disp,left,filtered_disp,right_disp);
		cv::Mat filtered_disp_vis;
		cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
		rtn->push_back(filtered_disp_vis); //filtered image

		//visualization
		if(!no_display){
            cv::namedWindow("filtered disparity", cv::WINDOW_AUTOSIZE);
            cv::imshow("filtered disparity", filtered_disp_vis);
            cv::waitKey(0);
		}
	}
	return true;
}
/*--------------------------------------------------4 팀 화 이 팅 ! ! ! ,,----------------------------------------------------*/

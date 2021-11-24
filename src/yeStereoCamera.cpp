#include "yeStereoCamera.hpp"

#include <iostream>

using namespace SYE;

YeStereoCamera::YeStereoCamera() {
	std::cout << "Create class" << std::endl;
}
YeStereoCamera::~YeStereoCamera() {
}

// //경로 내부의 이미지 파일을 읽어서 켈리브레이션 실시.
bool YeStereoCamera::doCalibration(const char* pPath, const char* ext = ".jpg") {

}
bool YeStereoCamera::doCalibration(std::vector<std::string>& imgList) {

}



// Yolo를 이용하여 특정 이름의 영역을 추출.
bool YeStereoCamera::findImage(const cv::Mat mat, const char* objName, bbox_t* pObjRect) 
{

}



//Absolute length from camera.
bool YeStereoCamera::getAbsoluteLengthInRect(const cv::Mat src, bbox_t* pObjRect, std::vector<YePos3D>& features) {
	cv::Ptr<cv::Feature2D> fast = cv::FastFeatureDetector::create();
	cv::Ptr<cv::Feature2D> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, true);

	cv::Mat gray, pic[2], dc[2];
	std::vector<cv::KeyPoint> kp[2];
	std::vector<cv::DMatch> matches;

	cv::Mat invMat[2];
	invMat[0] = matCamMat1.inv();
	invMat[1] = matCamMat2.inv();

	cv::cvtColor(src, gray, cv::COLOR_RGB2GRAY);

	pic[0] = gray(cv::Range(pObjRect->y, pObjRect->h), cv::Range(pObjRect->x, pObjRect->w));
	pic[1] = gray(cv::Range(pObjRect->y, pObjRect->h), cv::Range(gray.cols/2+pObjRect->x, gray.cols/2+pObjRect->w));

	fast->detect(pic[0], kp[0], pic[1]);
	brief->compute(pic[0], kp[0], dc[0]);
	fast->detect(pic[1], kp[1], pic[0]);
	brief->compute(pic[1], kp[1], dc[1]);
	matcher->match(dc[0], dc[1], matches);

	for(int i = 0; i < matches.size(); i++){
		YePos3D temp;
		
		temp.x = (int)kp[0][matches[i].queryIdx].pt.x;
		temp.y = (int)kp[1][matches[i].queryIdx].pt.y;
		temp.z = -matT.at<double>(0,0)/(invMat[1].at<double>(0,0)*(int)(-kp[1][matches[i].trainIdx].pt.x)-invMat[1].at<double>(0,2)+invMat[0].at<double>(0,0)*(int)(kp[0][matches[i].queryIdx].pt.x)+invMat[0].at<double>(0,2));

		features.push_back(temp);
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

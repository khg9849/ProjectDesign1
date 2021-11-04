#include <stdio.h>

#include <iostream>
#include <fstream>
#include <yeStereoCamera.hpp>

//#include <darknet.h>
#include <opencv2/opencv.hpp>
#include <yolo_v2_class.hpp>

class SeoCalibration: public SYE::YeStereoCamera {
public:
	virtual bool doCalibration(const char *pPath);

	virtual bool doCalibration(std::vector<std::string> &imgList) {
		return false;
	}
};

bool SeoCalibration::doCalibration(const char *pPath) {
	return false;
}

int main(int argc, char** argv) {

	SeoCalibration camSt;

	SeoCalibration *pCamSt = new SeoCalibration();

	pCamSt->doCalibration("../../alsdkjfalsdjf;lasdjl");


	//Make coco names list.

	std::vector<std::string> names;
	std::ifstream file("../../../darknet/data/coco.names");
	if (true == file.is_open()) {
		std::string s;
		while (file) {
			getline(file, s);
			names.push_back(s);
			//std::cout << s << std::endl;
		}
		file.close();
	} else {
		std::cout << "file open fail" << std::endl;
		return -1;
	}


	Detector yoloDetector("../../../darknet/cfg/yolov3.cfg", "../../../darknet/yolov3.weights");

	std::string imgFile = "../../../darknet/data/dog.jpg";
	cv::Mat mat = cv::imread(imgFile);
	//cv::imshow("origin", mat);
	//cv::waitKey(0);
	printf("\n\n\n Run YOLO \n\n\n");

	std::vector<bbox_t> yoloBoxes = yoloDetector.detect("../../../darknet/data/dog.jpg");

	for(bbox_t box: yoloBoxes) {
		cv::rectangle(mat, cv::Rect(box.x, box.y, box.w, box.h), cv::Scalar(0, 0, 255), 3);
		cv::putText(mat, names[box.obj_id], cv::Point(box.x, box.y), 3, 1., cv::Scalar(255, 0, 0), 2);
		//printf("[obj_id : %3d] [track_id : %3d]\n", box.obj_id, box.track_id);
	}

	cv::imshow("detected", mat);
	cv::waitKey(0);

	delete pCamSt;

	return 0;
}

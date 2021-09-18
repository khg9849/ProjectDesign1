#include <stdio.h>
#include <yeStereoCamera.hpp>


class SeoCalibration: public SYE::YeStereoCamera {
public:
	virtual bool doCalibration(const char *pPath) {
		return false;
	}

	virtual bool doCalibration(std::vector<std::string> &imgList) {
		return false;
	}
};

int main(int argc, char** argv) {

	SeoCalibration camSt;

	SeoCalibration *pCamSt = new SeoCalibration();

	delete pCamSt;

	return 0;
}

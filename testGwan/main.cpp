#include<iostream>

#include"yeStereoCamera.hpp"
#include"calibration.hpp"

using namespace SYE;

int main(){
	const char* path = "resources/image/90chess";
	std::cout << path << std::endl;
	Calibration test;
	
	test.doCalibration(path);
	
	return 0;
}

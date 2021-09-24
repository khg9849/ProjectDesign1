#include<iostream>
#include"yeStereoCamera.hpp"

using namespace SYE;

int main(){
	const char* path = "../../image/90chess";
	std::cout << path << std::endl;
	Calibration test;
	
	test.doCalibration(path);
	
	return 0;
}

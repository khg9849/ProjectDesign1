#include "calibration.hpp"
#include <iostream>

int main(int argc, char** argv){
	hwangCalibration a;

	a.doCalibration(argv[1]);

	return 0;
}

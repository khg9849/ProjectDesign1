#include "calibration.hpp"

int main(int argc, char** argv){
	hwangCalibration camSt;
	
	camSt.doCalibration(argv[1]);

	return 0;
}

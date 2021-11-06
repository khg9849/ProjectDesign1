#include <darknet.h>

int main(int argc, char**argv){
	float thresh = find_float_arg(argc, argv, "-thresh", .24);
	int ext_output = find_arg(argc, argv, "-ext_output");
	char *filename = (argc > 4) ? argv[4]: 0;
	test_detector("darknet/cfg/coco.data", argv[2], argv[3], filename, thresh, 0.5, 0, ext_output, 0, NULL, 0, 0);

	return 0;
}

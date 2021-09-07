#include <opencv2/opencv.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
//#include <stdio.h>

using namespace cv;
using namespace std;

int main()
{
	namedWindow("case", WINDOW_AUTOSIZE);
	string path = "case01.jpg";
	Mat image = imread(path, IMREAD_ANYCOLOR);
	
	if(image.empty())
	{
		cerr << "Image Loaded Fail !! " << endl;
		return -1;
	}
	
	unsigned char* pData = image.data;
	int width = image.cols;
	int height = image.rows;
	int channel = image.channels();
	int depth = image.depth();
	int type = image.type();
	
	cout << "width : " << width << endl;
	cout << "height : " << height << endl;
	cout << "channel : " << channel << endl;
	cout << "depth : " << depth << endl;
	cout << "type : " << type << endl;
	
	while(true)
	{
		imshow("case", image);
		
		if(waitKey(100) == 27)
			break;
	}
	
	//printf("hello, World!\n");
	

	return 0;
}



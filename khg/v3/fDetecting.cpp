#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;
int main(){

	Mat matchingImage[2]; // 0 is left, 1 is right
	
	matchingImage[0] = imread("1_Limage.jpg", IMREAD_GRAYSCALE);
	matchingImage[1] = imread("1_Rimage.jpg", IMREAD_GRAYSCALE);
	
	if(matchingImage[0].empty() || matchingImage[1].empty()){ //exception
		cerr << "Image load failed" << endl;
		
		return -1;
	}
	
	Ptr<Feature2D> feature = ORB::create();
	vector<KeyPoint> lKeypoints, rKeypoints;
	Mat lDesc, rDesc;
	feature->detectAndCompute(matchingImage[0], Mat(), lKeypoints, lDesc);
	feature->detectAndCompute(matchingImage[1], Mat(), rKeypoints, rDesc);
	
	Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);
	
	vector<DMatch> matches;
	
	matcher->match(lDesc, rDesc, matches);
	
	sort(matches.begin(), matches.end());
	vector<DMatch> good_matches(matches.begin(), matches.begin() + 100);
	
	Mat dst;
	drawMatches(matchingImage[0], lKeypoints, matchingImage[1], rKeypoints, good_matches, dst, Scalar::all(-1), Scalar(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	imshow("dst", dst);
	waitKey();
	return 0;
}

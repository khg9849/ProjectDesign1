#include "matching.h"
#include <ctime>

int stereoMatch(string inputFile, string outputFile){

    Mat img=cv::imread(inputFile);
    Mat gray;
    Mat lrImg[2];

     cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY); 
    lrImg[0] = gray(Range::all(), Range(0, gray.cols / 2));
    lrImg[1] = gray(Range::all(), Range(gray.cols / 2, gray.cols));
	
	if(lrImg[0].empty() || lrImg[1].empty()){ 
		cerr << "Image load failed" << endl;
		return -1;
	}
	

	Ptr<Feature2D> feature = ORB::create(
        500,1.2f,
        8,
        31,0,
        2,
        ORB::FAST_SCORE
    ); //Ptr: smart pointer class, works like pointer of ORB 

    vector<KeyPoint> lKeypoints, rKeypoints; //keypoints
	Mat lDesc, rDesc; //descriptors

    // detect keypoints and compute descriptors
    //영상의 특정 부분에서만 특징점을 검출하고 싶다면 Mat() 수정


    clock_t start, finish;
    double duration;
 
    start = clock();
    
	feature->detectAndCompute(lrImg[0], Mat(), lKeypoints, lDesc);
	feature->detectAndCompute(lrImg[1], Mat(), rKeypoints, rDesc);

	finish = clock();
 
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    cout << "duration: "<<duration << "s" << '\n';

    
    // Brute-force matcher create method
	Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);
	vector<DMatch> matches; // Class for matching keypoint descriptor
    matcher->match(lDesc, rDesc, matches); // finds the best match for each descriptor from a query set
	
    // get good matches
	sort(matches.begin(), matches.end()); //distance 작은 순으로 정렬
	vector<DMatch> good_matches(matches.begin(), matches.begin() + 100); //matching 잘 된 100개만 추출
	
	Mat dst;
    // drawMatches(lrImg[0], lKeypoints, lrImg[1], rKeypoints,
    //             good_matches, dst, 
    //             Scalar::all(-1), Scalar(-1), vector<char>(), 
    //             DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	// Scalar::all(-1) 각 키포인트마다 임의의 색상으로 원을 그림
    // NOT_DRAW_SINGLE_POINTS 매칭되지 않은 점은 그리지 않음

    drawKeypoints(lrImg[0],lKeypoints,dst,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    imshow("stereo matching result", dst);
    waitKey();
/*
    string outputFile="matched_";
    string base_filename = path.substr(path.find_last_of("/\\") + 1);
    cout<<base_filename<<'\n';
    outputFile.append(base_filename);
    */
    imwrite(outputFile,dst);
    cout<<outputFile<<" is saved"<<'\n';

    return 0;
}
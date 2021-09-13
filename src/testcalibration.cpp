#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <iostream>
#include <vector>

// using namespace cv;
using namespace std;

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{ 6, 9 };

// 이미지는 프로젝트 내 record images 프로젝트에서 저장해 놓은 것 가져다 쓸 것임
int main()
{
	/* 저장한 이미지 불러오기 */

	/* Extracting path of individual image stored in a given directory */
	vector<cv::String> images;
	vector<cv::String> normalimages;
	/* Path of the folder containing checkerboard images */

	//  "D:\Cali_images" 으로 하나 "D:\Cali_images/*.jpg"으로 하나 동일한 결과를 얻음을 확인하였다.
	string path = "../../image/chessImage";
	string normalImagepath = "../../image/normalImage";
	cv::glob(path, images);		// 파일 목록을 가져오는 glob 함수
	// glob(찾을 파일 경로, 찾은 파일 경로, recusive(true or false))
			// true : 폴더 내 하위 폴더 속 까지 파일을 찾음
			// false : 폴더 내 파일을 찾음

	// images에는 각각의 경로가 저장되어 있음?
	cout << "로드한 이미지 개수 : " << images.size() << endl;
	if (images.size() == 0)
		cout << "이미지가 존재하지 않음! \n" << endl;

	///***************************************************************************************************************************************************/
	/*   Calibration 시작   */

	/* Creating vector to store vector of 3D points for each checkerboard image */
	vector<vector<cv::Point3f> > objpoints_right;			// 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언
	vector<vector<cv::Point3f> > objpoints_left;			// 각 이미지에 대해 3D 좌표를 저장하는 벡터 선언

	/* Creating vector to store vectors of 2D points for each checkerboard image */
	vector<vector<cv::Point2f> > imgpoints_right;			// 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언
	vector<vector<cv::Point2f> > imgpoints_left;			// 각 이미지에 대해 2D 좌표를 저장하는 벡터 선언


	/* Defining the world coordinates for 3D points */
	vector<cv::Point3f> objp;								// 월드 좌표계 선언

	for (int i = 0; i < CHECKERBOARD[1]; i++)		// CHECKERBOARD[1] = 6
	{
		for (int j = 0; j < CHECKERBOARD[0]; j++)	// CHECKERBOARD[0] = 9
		{
			objp.push_back(cv::Point3f(j * 23, i * 23, 0));	// z는 0 이고 (x, y ,z)로 담기니까 (j, i , 0)으로 벡터에 push_back으로 값 담기
			// 실제 값에 대한 정보가 들어가야 하므로 j i 가 아니라 체스보드의 한칸 길이(나의 경우 2.8cm)까지 곱해서 넣어줘어야 함
		}
	}

	// 월드 좌표계 objp에는 54개의 좌표가 저장되어 있음
	//	실세계 좌표이며 (0,0,0) 다음에 (1, 0, 0)이 아니라 square length를 곱해서 실제 거리로 저장되어있는 좌표

	cv::Mat frame, gray, lrImage[2];

	/* vector to store the pixel coordinates of detected checker board corners */
	vector<cv::Point2f> corner_pts;		// 검출된 check board corner 포인트(2D 좌표)를 담을 벡터 선언
	bool success;						          // findChessboardCorners 되었는지 안 되었는지를 확인하기 위한 용도
	char buf[256];
	int index = 0;
	/* Looping over all the images in the directory */
	
	for (int i = 0; i < images.size(); i++)				// 받아온 이미지들은 images에 경로가? 저장되어있음. 한장씩 불러온다		// 매번 이미지에 대해 CHESSBOARD를 그리고 창 띄우기
	{
		frame = cv::imread(images[i]);	// images로부터 한 장씩 받아서 frame으로 읽어옴
		cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);		// COLOR 니까 GRAY로 바꾸기 위해 cvtColor 함수를 사용해 변경

		/* Finding checker board corners */
		/* If desired number of corners are found in the image then success = true */
		/*success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0],
			CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);*/
		lrImage[0] = gray(cv::Range::all(), cv::Range(0, gray.cols/2));
		lrImage[1] = gray(cv::Range::all(), cv::Range(gray.cols/2, gray.cols));
		//success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		for(int l = 0; l < 2; l++){
			// bool findChessboardCorners(InputArray image, Size patternSize, OutputArray corners, int flags = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE )
			success = cv::findChessboardCorners(lrImage[l], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			/* If desired number of vcorner are detected, we refine the pixel coordinates and display them on the images of checker board*/
			if (success)
			{
				cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);
				// TermCriteria::TermCriteria(int type, int maxCount, double epsilon)

				/* refining pixel coordinates for given 2d points. */
				cv::cornerSubPix(lrImage[l], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);	// 주어진 2D point에 대해 더 정제시켜 업데이트
				// void cornerSubPix(InputArray image, InputOutputArray corners, Size winSize, Size zeroZone, TermCriteria criteria)

				/* Displaying the detected corner points on the checker board */
				if((l%2) == 0){
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
				}
				else {
					for(int k = 0; k < corner_pts.size(); k++)
					{
						corner_pts[k].x += lrImage[l].cols;
					}
					cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);
				
				}
						// Chessboard Corner 그리기
				// drawChessboardCorners(images.at(i), boardSize, Mat(cornersScene[0]), patternFound );

				if ((l % 2) == 0)
				{
					objpoints_left.push_back(objp);				// 해당 번째의 objp의 값을 objpoints에 추가
					imgpoints_left.push_back(corner_pts);		// 해당 번째의 corner_pts의 값을 imgopoints에 추가
				}
				else
				{
					objpoints_right.push_back(objp);				// 해당 번째의 objp의 값을 objpoints에 추가
					imgpoints_right.push_back(corner_pts);		// 해당 번째의 corner_pts의 값을 imgopoints에 추가
				}

			}
		}
		cv::imshow("Image", frame);			// 해당 번째의 이미지에 대해 CHESSBOARD CORNER 그린걸 창에 띄움
		
		sprintf(buf, "%d_image.jpg", index);
		cv::imwrite(buf, frame);
		index++;
		
		cv::waitKey(0);						// 특별한 입력 있을 때까지 대기
	}

	cv::destroyAllWindows();			// 모든 WINDOW 제거하기

	cv::Mat cameraMatrix_left, distCoeffs_left, R_left, T_left;	// 파라미터를 구하기 위한 Mat 객체 선언
	cv::Mat cameraMatrix_right, distCoeffs_right, R_right, T_right;

	// cameraMatrix는 내부 파라미터
	// distCoeffs는 왜곡 파라미터
	// R, T 는 외부 파라미터

	/* Performing camera calibration by passing the value of known 3D points (objpoints and corresponding pixel coordinates of the detected corners (imgpoints)	*/

	// cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
	cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(lrImage[0].rows, lrImage[0].cols), cameraMatrix_left, distCoeffs_left, R_left, T_left);
	cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(lrImage[1].rows, lrImage[1].cols), cameraMatrix_right, distCoeffs_right, R_right, T_right);
	// 위에서 저장했던 object point와 image point를 이용하여 parameter 구하기
	// double calibrateCamera(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, Size imageSize, InputOutputArray cameraMatrix, InputOutputArray distCoeffs, OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs)


	// 각각 구해진 왼쪽과 오른쪽의 R과 T , and 내부파라미터를 통해 새로운 함수를 사용해 Pair의 관계를 구해야 함  (OpenCV stereo calibration)

	// 왼쪽
	cout << "[왼쪽 IR Camera Parameters]\n";
	cout << "Left CameraMatrix\n" << cameraMatrix_left << "\n\n";
	cout << "Left DistCoeffs\n" << distCoeffs_left << "\n\n";
	cout << "Left Rotation Vector\n" << R_left << "\n\n";
	cout << "Left Translation Vector\n" << T_left << "\n\n\n";

	// 오른쪽
	cout << "[오른쪽 IR Camera Parameters]\n";
	cout << "Right CameraMatrix\n" << cameraMatrix_right << "\n\n";
	cout << "Right DistCoeffs\n" << distCoeffs_right << "\n\n";
	cout << "Right Rotation Vector\n" << R_right << "\n\n";
	cout << "Right Translation Vector\n" << T_right << "\n\n\n";

	/*      Stereo Calibration 시작     */
	cv::Mat R, T, E, F;

	/* cv::stereoCalibrate (InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints1, InputArrayOfArrays imagePoints2,
							InputOutputArray cameraMatrix1, InputOutputArray distCoeffs1, InputOutputArray cameraMatrix2, InputOutputArray distCoeffs2, Size imageSize,
							InputOutputArray R, InputOutputArray T, OutputArray E, OutputArray F, OutputArray perViewErrors,
							int flags=CALIB_FIX_INTRINSIC, TermCriteria criteria=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6))
	*/
	cv::Size imgsize(gray.rows, gray.cols/2);

	cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right,
		cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize,
		R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

	// Stereo Calibration Parameters
	cout << "[Stereo Camera parameters]\n";
	cout << "Rotation Matrix\n" << R << "\n\n";
	cout << "Translation Vector\n" << T << "\n\n";
	cout << "Essential Matrix\n" << E << "\n\n";
	cout << "Fundamental Matrix\n" << F << "\n\n\n";

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validRoi[2];

	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize,
		R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);

	// 양 카메라의 이미지 평면을 같은 평면으로 바꾸는 변환 행렬을 계산
	cout << "[Stereo Rectify parameters]\n";
	cout << "R1\n" << R1 << "\n\n";
	cout << "R2\n" << R2 << "\n\n";
	cout << "P1\n" << P1 << "\n\n";
	cout << "P2\n" << P2 << "\n\n";
	cout << "Q\n" << Q << "\n\n\n";
	
	cv::Mat undistortedImage[2];
	
	undistort(lrImage[0], undistortedImage[0], cameraMatrix_left ,distCoeffs_left);
	undistort(lrImage[1], undistortedImage[1], cameraMatrix_left ,distCoeffs_left);
	cv::imshow("image1", undistortedImage[0]);
	cv::imshow("image2", undistortedImage[1]);
	
	cv::waitKey(0);

	cv::glob(normalImagepath, normalimages);
	cv::Mat undistortnormalImage[2]; // normal image's left and right
	for (int i = 0; i < normalimages.size(); i++){
		frame = cv::imread(normalimages[i]);
	
		
		lrImage[0] = frame(cv::Range::all(), cv::Range(0, frame.cols/2));
		lrImage[1] = frame(cv::Range::all(), cv::Range(frame.cols/2, frame.cols));
		undistort(lrImage[0], undistortnormalImage[0], cameraMatrix_left ,distCoeffs_left);
		undistort(lrImage[1], undistortnormalImage[1], cameraMatrix_left ,distCoeffs_left);
		cv::imshow("image1", undistortnormalImage[0]);
		cv::imshow("image2", undistortnormalImage[1]);
	
		cv::waitKey(0);
	}
	
	
	
	return 0;
}

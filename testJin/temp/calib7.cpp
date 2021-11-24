#include "calibration.h"

cb::cb(){};

void cb::calib(string dir, string outputFile)
{
     //setObjp
    for (int i = 0; i < CHECKERBOARD[1]; i++) 
        for (int j = 0; j < CHECKERBOARD[0]; j++) 
            objp.push_back(cv::Point3f(j * 20, i * 20, 0)); 
        
    /* 저장한 이미지 불러오기 */
    cv::glob(dir, images, false); 
    cout << "로드한 이미지 개수 : " << images.size() << endl;
    if (images.size() == 0)
        cout << "이미지가 존재하지 않음\n";

    cv::Mat frame, gray;

    /* vector to store the pixel coordinates of detected checker board corners */
    vector<cv::Point2f> corner_pts; // 검출된 check board corner 포인트(2D 좌표)를 담을 벡터 선언
    bool success;                   // findChessboardCorners 되었는지 안 되었는지를 확인하기 위한 용도
    char buf[256];
    int index = 0;

    cv::Mat lrImg[2];

    for (int i = 0; i < images.size(); i++)
    {
         cout<<"processing image["<<i<<"]..."<<'\n';
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); 
        lrImg[0] = gray(Range::all(), Range(0, gray.cols / 2));
        lrImg[1] = gray(Range::all(), Range(gray.cols / 2, gray.cols));

        for (int ii = 0; ii < 2; ii++)
        {
            success = cv::findChessboardCorners(lrImg[ii], cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
            if (success)
            {
                cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 300, 0.001);

                /* refining pixel coordinates for given 2d points. */
                cv::cornerSubPix(lrImg[ii], corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria); // 주어진 2D point에 대해 더 정제시켜 업데이트
                if ((ii % 2) == 0)
                {
                    objpoints_left.push_back(objp);       
                    imgpoints_left.push_back(corner_pts); 
                }
                else
                {
                    objpoints_right.push_back(objp);       
                    imgpoints_right.push_back(corner_pts);
                    }
            }
        }
    }

    cv::calibrateCamera(objpoints_left, imgpoints_left, cv::Size(gray.rows, gray.cols / 2), cameraMatrix_left, distCoeffs_left, R_left, T_left);
    cv::calibrateCamera(objpoints_right, imgpoints_right, cv::Size(gray.rows, gray.cols / 2), cameraMatrix_right, distCoeffs_right, R_right, T_right);

    /*      Stereo Calibration 시작     */
    cv::Size imgsize=lrImg[0].size();
    cv::stereoCalibrate(objpoints_left, imgpoints_left, imgpoints_right, cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, E, F, cv::CALIB_FIX_INTRINSIC, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));
	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, imgsize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, imgsize, &validRoi[0], &validRoi[1]);
    
    writeCalibResult(outputFile);
}


void cb::printCalibResult()
{
    // 왼쪽
    cout << "[왼쪽 IR Camera Parameters]\n";
    cout << "Left CameraMatrix\n"
         << cameraMatrix_left << "\n\n";
    cout << "Left DistCoeffs\n"
         << distCoeffs_left << "\n\n";
    cout << "Left Rotation Vector\n"
         << R_left << "\n\n";
    cout << "Left Translation Vector\n"
         << T_left << "\n\n\n";

    // 오른쪽
    cout << "[오른쪽 IR Camera Parameters]\n";
    cout << "Right CameraMatrix\n"
         << cameraMatrix_right << "\n\n";
    cout << "Right DistCoeffs\n"
         << distCoeffs_right << "\n\n";
    cout << "Right Rotation Vector\n"
         << R_right << "\n\n";
    cout << "Right Translation Vector\n"
         << T_right << "\n\n\n";

    // Stereo Calibration Parameters
    cout << "[Stereo Camera parameters]\n";
    cout << "Rotation Matrix\n"
         << R << "\n\n";
    cout << "Translation Vector\n"
         << T << "\n\n";
    cout << "Essential Matrix\n"
         << E << "\n\n";
    cout << "Fundamental Matrix\n"
         << F << "\n\n\n";

    // 양 카메라의 이미지 평면을 같은 평면으로 바꾸는 변환 행렬을 계산
    cout << "[Stereo Rectify parameters]\n";
    cout << "R1\n"
         << R1 << "\n\n";
    cout << "R2\n"
         << R2 << "\n\n";
    cout << "P1\n"
         << P1 << "\n\n";
    cout << "P2\n"
         << P2 << "\n\n";
    cout << "Q\n"
         << Q << "\n\n\n";
}

void cb::writeCalibResult(string outputFile)
{
     cout<<"writing calibration result to "<<outputFile<<"..."<<'\n';
    FileStorage fs(outputFile, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        cerr << "OUTPUT FILE cannot be opened" << '\n';
        return;
    }

    fs << "cameraMatrix_left" << cameraMatrix_left;
    fs << "distCoeffs_left" << distCoeffs_left;
    
    fs << "cameraMatrix_right" << cameraMatrix_right;
    fs << "distCoeffs_right" << distCoeffs_right;
   
	fs << "rotationMatrix" << R;
    fs << "TranslationVector" << T;
    fs << "essentialMatrix" << E;
	fs << "fundamentalMatrix" << F;
	fs << "rectifyR1" << R1;
	fs << "rectifyR2" << R2;
	fs << "rectifyP1" << P1;
	fs << "rectifyP2" << P2;
	fs << "rectifyQ" << Q;

    fs.release();
}

void cb::readCalibResult(string inputFile)
{

    FileStorage fs(inputFile, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        cerr << "INPUT FILE cannot be opened" << '\n';
        return;
    }

    fs["cameraMatrix_left"] >> cameraMatrix_left;
    fs["distCoeffs_left"] >> distCoeffs_left;

    
    fs["cameraMatrix_right"] >> cameraMatrix_right;
    fs["distCoeffs_right"] >> distCoeffs_right;


    fs["rotationMatrix"] >> R;
    fs ["TranslationVector"] >> T;
    fs ["essentialMatrix"] >> E;
	fs["fundamentalMatrix"] >> F;
	fs ["rectifyR1"] >> R1;
	fs ["rectifyR2"] >> R2;
	fs  ["rectifyP1"] >> P1;
	fs ["rectifyP2"] >> P2;
	fs ["rectifyQ"] >> Q;

    fs.release();
}

void cb::undistort2(string inputFile, string outputFile)
{
    cv::Mat frame = cv::imread(inputFile);
    cv::Mat lrImg[2];
    lrImg[0] = frame(Range::all(), Range(0, frame.cols / 2));
    lrImg[1] = frame(Range::all(), Range(frame.cols / 2, frame.cols));

    Mat imageUndistorted[2];
    undistort(lrImg[0], imageUndistorted[0], cameraMatrix_left, distCoeffs_left);
    undistort(lrImg[1], imageUndistorted[1], cameraMatrix_left, distCoeffs_left);

    Mat dst;
    hconcat(imageUndistorted[0], imageUndistorted[1],dst);

    imshow("originalImg", frame);
    imshow("undistortedImg", dst);
    waitKey();

    imwrite(outputFile,dst);
    cout<<outputFile<<" is saved"<<'\n';
}
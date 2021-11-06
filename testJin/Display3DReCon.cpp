#include "Display3DReCon.h"

Display3DReCon::Display3DReCon(){
    ;
}
bool Display3DReCon::init(CommandLineParser parser){

    parser.about("Disparity Filtering Demo");
    if (parser.has("help"))
    {
        parser.printMessage();
        return false;
    }

     left_im = parser.get<String>(0);
     right_im = parser.get<String>(1);
     GT_path = parser.get<String>("GT");

     dst_path = parser.get<String>("dst_path");
     dst_raw_path = parser.get<String>("dst_raw_path");
     dst_conf_path = parser.get<String>("dst_conf_path");
     algo = parser.get<String>("algorithm");
     filter = parser.get<String>("filter");
     no_display = parser.has("no-display");
     no_downscale = parser.has("no-downscale");
     max_disp = parser.get<int>("max_disparity");
     lambda = parser.get<double>("wls_lambda");
     sigma  = parser.get<double>("wls_sigma");
     fbs_spatial = parser.get<double>("fbs_spatial");
     fbs_luma = parser.get<double>("fbs_luma");
     fbs_chroma = parser.get<double>("fbs_chroma");
     fbs_lambda = parser.get<double>("fbs_lambda");
     vis_mult = parser.get<double>("vis_mult");

    if(parser.get<int>("window_size")>=0) //user provided window_size value
        wsize = parser.get<int>("window_size");
    else
    {
        if(algo=="sgbm")
            wsize = 3; //default window size for SGBM
        else if(!no_downscale && algo=="bm" && filter=="wls_conf")
            wsize = 7; //default window size for BM on downscaled views (downscaling is performed only for wls_conf)
        else
            wsize = 15; //default window size for BM on full-sized views
    }

    if (!parser.check())
    {
        parser.printErrors();
        return false;
    }

    //! [load_views]
    
    VideoCapture video("../darknet/data/WIN_20211104_15_51_04_Pro.mp4");
    

    float videoFPS = video.get(CAP_PROP_FPS);
    int videoWidth = video.get(CAP_PROP_FRAME_WIDTH);
    int videoHeight = video.get(CAP_PROP_FRAME_HEIGHT);
    int fourcc = VideoWriter::fourcc('D', 'I', 'V', 'X');
    if(!video.isOpened()){
        cout << "Can't open the video" << endl;
        return false;
    }

    newVideo = VideoWriter("disparity.mp4", fourcc, videoFPS, Size(videoWidth, videoHeight), true);
    if(!newVideo.isOpened()){
        cout << "Can't write video" << endl;
        return false;
    }
    return true;

}
    



double Display3DReCon::test3dReCon(Mat frame){

        //video >> frame;
        Mat left  = frame(Range::all(), Range(0, frame.cols/2));
        Mat right = frame(Range::all(), Range(frame.cols/2, frame.cols));
    #if DEBUG
        namedWindow("origin", WINDOW_AUTOSIZE);
        imshow("origin", frame);
        waitKey(0);
    #endif
        //! [load_views]

        bool noGT;
        Mat GT_disp;
        if (GT_path=="./aloeGT.png" && left_im!="../data/aloeL.jpg")
            noGT=true;
        else
        {
            noGT=false;
            if(readGT(GT_path,GT_disp)!=0)
            {
                cout<<"Cannot read ground truth image file: "<<GT_path<<endl;
                return -1;
            }
        }


        Mat left_for_matcher, right_for_matcher;
        Mat left_disp,right_disp;
        Mat filtered_disp,solved_disp,solved_filtered_disp;
        Mat conf_map = Mat(left.rows,left.cols,CV_8U);
        conf_map = Scalar(255);
        Rect ROI;
        Ptr<DisparityWLSFilter> wls_filter;
        double matching_time, filtering_time;
        double solving_time = 0;
        if(max_disp<=0 || max_disp%16!=0)
        {
            cout<<"Incorrect max_disparity value: it should be positive and divisible by 16";
            return -1;
        }
        if(wsize<=0 || wsize%2!=1)
        {
            cout<<"Incorrect window_size value: it should be positive and odd";
            return -1;
        }

        if(filter=="wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
        {
            if(!no_downscale)
            {
                // downscale the views to speed-up the matching stage, as we will need to compute both left
                // and right disparity maps for confidence map computation
                //! [downscale]
                max_disp/=2;
                if(max_disp%16!=0)
                    max_disp += 16-(max_disp%16);
                resize(left ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
                resize(right,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
                //! [downscale]
            }
            else
            {
                left_for_matcher  = left.clone();
                right_for_matcher = right.clone();
            }

            if(algo=="bm")
            {
                //! [matching]
                Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
                wls_filter = createDisparityWLSFilter(left_matcher);
                Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

                cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
                cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

                matching_time = (double)getTickCount();
                left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
                right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
                //! [matching]
            }
            else if(algo=="sgbm")
            {
                Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
                left_matcher->setP1(24*wsize*wsize);
                left_matcher->setP2(96*wsize*wsize);
                left_matcher->setPreFilterCap(63);
                left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
                wls_filter = createDisparityWLSFilter(left_matcher);
                Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

                matching_time = (double)getTickCount();
                left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
                right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            }
            else
            {
                cout<<"Unsupported algorithm";
                return -1;
            }

            //! [filtering]
            wls_filter->setLambda(lambda);
            wls_filter->setSigmaColor(sigma);
            filtering_time = (double)getTickCount();
            wls_filter->filter(left_disp,left,filtered_disp,right_disp);
            filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
            //! [filtering]
            conf_map = wls_filter->getConfidenceMap();

            // Get the ROI that was used in the last filter call:
            ROI = wls_filter->getROI();
            if(!no_downscale)
            {
                // upscale raw disparity and ROI back for a proper comparison:
                resize(left_disp,left_disp,Size(),2.0,2.0,INTER_LINEAR_EXACT);
                left_disp = left_disp*2.0;
                ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
            }
        }
        else if(filter=="fbs_conf") // filtering with fbs and confidence using also wls pre-processing
        {
            if(!no_downscale)
            {
                // downscale the views to speed-up the matching stage, as we will need to compute both left
                // and right disparity maps for confidence map computation
                //! [downscale_wls]
                max_disp/=2;
                if(max_disp%16!=0)
                    max_disp += 16-(max_disp%16);
                resize(left ,left_for_matcher ,Size(),0.5,0.5);
                resize(right,right_for_matcher,Size(),0.5,0.5);
                //! [downscale_wls]
            }
            else
            {
                left_for_matcher  = left.clone();
                right_for_matcher = right.clone();
            }

            if(algo=="bm")
            {
                //! [matching_wls]
                Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
                wls_filter = createDisparityWLSFilter(left_matcher);
                Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

                cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
                cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

                matching_time = (double)getTickCount();
                left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
                right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
                //! [matching_wls]
            }
            else if(algo=="sgbm")
            {
                Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
                left_matcher->setP1(24*wsize*wsize);
                left_matcher->setP2(96*wsize*wsize);
                left_matcher->setPreFilterCap(63);
                left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
                wls_filter = createDisparityWLSFilter(left_matcher);
                Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

                matching_time = (double)getTickCount();
                left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
                right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            }
            else
            {
                cout<<"Unsupported algorithm";
                return -1;
            }

            //! [filtering_wls]
            wls_filter->setLambda(lambda);
            wls_filter->setSigmaColor(sigma);
            filtering_time = (double)getTickCount();
            wls_filter->filter(left_disp,left,filtered_disp,right_disp);
            filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
            //! [filtering_wls]

            conf_map = wls_filter->getConfidenceMap();

            Mat left_disp_resized;
            resize(left_disp,left_disp_resized,left.size());

            // Get the ROI that was used in the last filter call:
            ROI = wls_filter->getROI();
            if(!no_downscale)
            {
                // upscale raw disparity and ROI back for a proper comparison:
                resize(left_disp,left_disp,Size(),2.0,2.0);
                left_disp = left_disp*2.0;
                left_disp_resized = left_disp_resized*2.0;
                ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
            }

    #ifdef HAVE_EIGEN
            //! [filtering_fbs]
            solving_time = (double)getTickCount();
            fastBilateralSolverFilter(left, left_disp_resized, conf_map/255.0f, solved_disp, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda);
            solving_time = ((double)getTickCount() - solving_time)/getTickFrequency();
            //! [filtering_fbs]

            //! [filtering_wls2fbs]
            fastBilateralSolverFilter(left, filtered_disp, conf_map/255.0f, solved_filtered_disp, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda);
            //! [filtering_wls2fbs]
    #else
            (void)fbs_spatial;
            (void)fbs_luma;
            (void)fbs_chroma;
            (void)fbs_lambda;
    #endif
        }
        else if(filter=="wls_no_conf")
        {
            /* There is no convenience function for the case of filtering with no confidence, so we
            will need to set the ROI and matcher parameters manually */

            left_for_matcher  = left.clone();
            right_for_matcher = right.clone();

            if(algo=="bm")
            {
                Ptr<StereoBM> matcher  = StereoBM::create(max_disp,wsize);
                matcher->setTextureThreshold(0);
                matcher->setUniquenessRatio(0);
                cvtColor(left_for_matcher,  left_for_matcher, COLOR_BGR2GRAY);
                cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
                ROI = this->computeROI(left_for_matcher.size(),matcher);
                wls_filter = createDisparityWLSFilterGeneric(false);
                wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33*wsize));

                matching_time = (double)getTickCount();
                matcher->compute(left_for_matcher,right_for_matcher,left_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            }
            else if(algo=="sgbm")
            {
                Ptr<StereoSGBM> matcher  = StereoSGBM::create(0,max_disp,wsize);
                matcher->setUniquenessRatio(0);
                matcher->setDisp12MaxDiff(1000000);
                matcher->setSpeckleWindowSize(0);
                matcher->setP1(24*wsize*wsize);
                matcher->setP2(96*wsize*wsize);
                matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
                ROI = this->computeROI(left_for_matcher.size(),matcher);
                wls_filter = createDisparityWLSFilterGeneric(false);
                wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));

                matching_time = (double)getTickCount();
                matcher->compute(left_for_matcher,right_for_matcher,left_disp);
                matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            }
            else
            {
                cout<<"Unsupported algorithm";
                return -1;
            }

            wls_filter->setLambda(lambda);
            wls_filter->setSigmaColor(sigma);
            filtering_time = (double)getTickCount();
            wls_filter->filter(left_disp,left,filtered_disp,Mat(),ROI);
            filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
        }
        else
        {
            cout<<"Unsupported filter";
            return -1;
        }

        //collect and print all the stats:
    #if DEBUG
        cout.precision(2);
        cout<<"Matching time:  "<<matching_time<<"s"<<endl;
        cout<<"Filtering time: "<<filtering_time<<"s"<<endl;
        cout<<"Solving time: "<<solving_time<<"s"<<endl;
        cout<<endl;
#endif
        double MSE_before,percent_bad_before,MSE_after,percent_bad_after;
        if(!noGT)
        {
            MSE_before = computeMSE(GT_disp,left_disp,ROI);
            percent_bad_before = computeBadPixelPercent(GT_disp,left_disp,ROI);
            MSE_after = computeMSE(GT_disp,filtered_disp,ROI);
            percent_bad_after = computeBadPixelPercent(GT_disp,filtered_disp,ROI);

            cout.precision(5);
            cout<<"MSE before filtering: "<<MSE_before<<endl;
            cout<<"MSE after filtering:  "<<MSE_after<<endl;
            cout<<endl;
            cout.precision(3);
            cout<<"Percent of bad pixels before filtering: "<<percent_bad_before<<endl;
            cout<<"Percent of bad pixels after filtering:  "<<percent_bad_after<<endl;
        }

        if(dst_path!="None")
        {
            Mat filtered_disp_vis;
            getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
            imwrite(dst_path,filtered_disp_vis);
        }
        if(dst_raw_path!="None")
        {
            Mat raw_disp_vis;
            getDisparityVis(left_disp,raw_disp_vis,vis_mult);
            imwrite(dst_raw_path,raw_disp_vis);
        }
        if(dst_conf_path!="None")
        {
            imwrite(dst_conf_path,conf_map);
        }

        if(!no_display)
        {
            //namedWindow("left", WINDOW_AUTOSIZE);
            //imshow("left", left);
            //namedWindow("right", WINDOW_AUTOSIZE);
            //imshow("right", right);

            if(!noGT)
            {
                Mat GT_disp_vis;
                getDisparityVis(GT_disp,GT_disp_vis,vis_mult);
                namedWindow("ground-truth disparity", WINDOW_AUTOSIZE);
                imshow("ground-truth disparity", GT_disp_vis);
            }

            //! [visualization]
            Mat raw_disp_vis;
            getDisparityVis(left_disp,raw_disp_vis,vis_mult);
            //namedWindow("raw disparity", WINDOW_AUTOSIZE);
            //imshow("raw disparity", raw_disp_vis);
            Mat filtered_disp_vis;
            getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
#if DEBUG
            namedWindow("filtered disparity", WINDOW_AUTOSIZE);
            imshow("filtered disparity", filtered_disp_vis);
            waitKey(0);
#endif
            if(!solved_disp.empty())
            {
                Mat solved_disp_vis;
                getDisparityVis(solved_disp,solved_disp_vis,vis_mult);
                namedWindow("solved disparity", WINDOW_AUTOSIZE);
                //imshow("solved disparity", solved_disp_vis);

                Mat solved_filtered_disp_vis;
                getDisparityVis(solved_filtered_disp,solved_filtered_disp_vis,vis_mult);
                namedWindow("solved wls disparity", WINDOW_AUTOSIZE);
                //imshow("solved wls disparity", solved_filtered_disp_vis);
            }
            
            newVideo << filtered_disp_vis;
            

            waitKey(1);
            /*
            while(1)
            {
                char key = (char)waitKey();
                if( key == 27 || key == 'q' || key == 'Q') // 'ESC'
                    break;
            }*/
            //! [visualization]
        }
    return matching_time+solving_time;
}

Rect Display3DReCon::computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
    int min_disparity = matcher_instance->getMinDisparity();
    int num_disparities = matcher_instance->getNumDisparities();
    int block_size = matcher_instance->getBlockSize();

    int bs2 = block_size/2;
    int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

    int xmin = maxD + bs2;
    int xmax = src_sz.width + minD - bs2;
    int ymin = bs2;
    int ymax = src_sz.height - bs2;

    Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
    return r;
}
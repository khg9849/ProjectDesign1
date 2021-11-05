#include "CallYolo.h"

CallYolo::CallYolo()
{
    detector = NULL;
    photopath = "";
}

void CallYolo::init(std::string _cfg, std::string _weight)
{
    if((detector = new Detector(_cfg, _weight)) == NULL){
        perror("CallYolo error : failed to make detector object!");
        exit(1);
    }
}

void CallYolo::setPhoto(std::string _photopath)
{
    if((photopath = _photopath) == ""){
        perror("CallYolo error : failed to make photopath!");
        exit(1);
    }
}

bbox_t_container CallYolo::getCont()
{
    if(detector == NULL){
        perror("CallYolo error : failed to check detector object!");
        exit(1);
    }
    if(photopath == ""){
        perror("CallYolo error : failed to check photofile path!");
        exit(1);
    }

    detection = detector->detect(photopath);

    contsize = detection.size();
    for(size_t i = 0; i < detection.size(); ++i)
        cont.candidates[i] = detection[i];

    /*
        there was not cont.candidates[0]
    
        public struct bbox_t
        {
            public UInt32 x, y, w, h;    // (x,y) - top-left corner, (w, h) - width & height of bounded box
            public float prob;           // confidence - probability that the object was found correctly
            public UInt32 obj_id;        // class of object - from range [0, classes-1]
            public UInt32 track_id;      // tracking id for video (0 - untracked, 1 - inf - tracked object)
            public UInt32 frames_counter;
            public float x_3d, y_3d, z_3d;  // 3-D coordinates, if there is used 3D-stereo camera
        };
    
        how to use?
            printf("%d", cont.candidates[1].x);
    */
    return cont;
}

size_t CallYolo::getContSize()
{
    if(contsize == 0){
        perror("CallYolo error : cont size is 0!");
        exit(1);
    }
    return contsize;
}
#include "darknet.h"
#include "yolo_v2_class.hpp"

class CallYolo{
private:
    Detector *detector;
    std::string photopath;
    
    bbox_t_container cont;
    std::vector<bbox_t> detection;
    size_t contsize; //contsize == detection.size()


public:
    CallYolo();
    ~CallYolo();

    void init(std::string _cfg, std::string _weight);
    void setPhoto(std::string);

    bbox_t_container getCont();
    size_t getContSize();
};
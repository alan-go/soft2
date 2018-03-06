#ifndef SYSTEM_H
#define SYSTEM_H

#include "camera.h"
#include "common_include.h"
#include "tracking.h"
#include "frame.h"
#include "mapping.h"
#include "featurePointBox.h"

namespace SOFT
{

class FeaturePointBox;
class Mapping;

struct Params
{
	int width = 1241;
	int height = 376;

};

class System
{
public:
	Camera camera;
	int frameNumber = 0;
	Params params;
    Frame* currentFrame;
    Tracking* tracker;
    Mapping* mapper;
    FeaturePointBox* fPointBox;
    

    std::thread* threadMapping;
    std::thread* threadViewer;
    
    
    bool systemStop = false;
    
public:
    System(int i);
    ~System ();

    void init();

    void TrackStereo (  cv::Mat &imLeft,  cv::Mat &imRight,  double &timestamp );
};

}

#endif
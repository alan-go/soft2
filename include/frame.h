#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"

#include "camera.h"
#include "featurePointBox.h"

namespace SOFT
{

// forward declare
class FeaturePoint;
class FeaturePointBox;
class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long                  id_;         // id of this frame
    double                         time_stamp_; // when it is recorded
    SE3                            T_c_w_;      // transform from world to camera
    SE3                            T_c_p_;      // transform from PreviousFrame to current
    Camera::Ptr                    camera_;     // Pinhole RGBD Camera model
    std::vector<SOFT::FeaturePoint*> featurePoints;
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points
    bool                           is_key_frame_;  // whether a key-frame

    Frame* nextFrame;
	Frame* PreviousFrame;

public: // data members
    Frame();
	Frame(SOFT::System* ptSys,SOFT::FeaturePointBox* ptFBox);
    ~Frame();

    // Get Camera Center
    Vector3d getCamCenter() const;

    void setPose ( const SE3& T_c_w );

};

}

#endif // FRAME_H

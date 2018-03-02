#include "frame.h"

namespace SOFT
{

Frame::Frame()
{

}


Frame::~Frame()
{

}


Frame::Ptr Frame::createCurrentFrame ( FeaturePointBox* pt )
{
    static long factory_id = 0;
}



void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}


}

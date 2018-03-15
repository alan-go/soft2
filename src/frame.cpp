#include "frame.h"

namespace SOFT
{

Frame::Frame()
{

}


Frame::~Frame()
{

}

Frame::Frame ( System* ptSys, FeaturePointBox* ptFBox )
{
	PreviousFrame = ptSys->currentFrame;
	ptSys->currentFrame->nextFrame = this;

	T_c_p_ = ptFBox->tPrevious2Current;
	T_c_w_ = T_c_p_ *( ptSys->currentFrame->T_c_w_);
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

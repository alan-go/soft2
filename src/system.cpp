#include "system.h"


namespace SOFT
{


//  VisualOdometry::VisualOdometry() :
//    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ (
//new cv::flann::LshIndexParams ( 5,10,2 ) )


System::System ( int i ) :
    currentFrame ( new Frame ),tracker ( new Tracking ),mapper ( new Mapping ),fPointBox ( new FeaturePointBox ( this ) )
{
    camera = Camera();
    threadMapping = new thread ( &SOFT::Mapping::RunMapping, mapper );
}


System::~System()
{

}


void System::init()
{

}


void System::TrackStereo (  cv::Mat &imLeft,  cv::Mat &imRight,  double& timestamp )
{
    printf ( "TrackSsstereo:number = %d\n",frameNumber );
// 	cv::imshow("left",imLeft);
// 	cv::waitKey(200);

    fPointBox->ExtractFPointsFromImage ( imLeft,imRight );
    if ( frameNumber > 0 ) {
        fPointBox->FeatureProcess ( frameNumber );
    }


    frameNumber++;
}


}

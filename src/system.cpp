#include "system.h"


namespace SOFT
{


//  VisualOdometry::VisualOdometry() :
//    state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_ (
//new cv::flann::LshIndexParams ( 5,10,2 ) )


System::System ( int i ) :
    currentFrame ( new Frame ),tracker ( new Tracking ),mapper ( new Mapping ),fPointBox ( new FeaturePointBox ( this ) ),camera(new Camera)
{
    threadMapping = new thread ( &SOFT::Mapping::RunMapping, mapper );
}


System::~System()
{

}


void System::init()
{

}

void System::loadPoses ( string str )
{
	FILE *fp = fopen(str.c_str(),"r");
	if (!fp)
		return ;
	while (!feof(fp)) {
		Eigen::Matrix<double,3,4> pose;
		if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&pose(0,0), &pose(0,1), &pose(0,2), &pose(0,3),
			 &pose(1,0), &pose(1,1), &pose(1,2), &pose(1,3),
			 &pose(2,0), &pose(2,1), &pose(2,2), &pose(2,3) )==12) {
			poses.push_back(pose);
			 }
	}
}


void System::TrackStereo (  cv::Mat &imLeft,  cv::Mat &imRight,  double& timestamp )
{
    printf ( "TrackSsstereo:number = %d\n",frameNumber );
// 	cv::imshow("left",imLeft);
// 	cv::waitKey(200);

    fPointBox->ExtractFPointsFromImage ( imLeft,imRight );
    if ( frameNumber > 0 ) {
        fPointBox->FeatureProcess ( frameNumber );

		SOFT::Frame* nextFrame = new Frame(this,fPointBox);

    }



    frameNumber++;
}


}

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


cv::Mat System::TrackStereo ( const cv::Mat& imLeft, const cv::Mat& imRight, const double& timestamp )
{
    printf ( "TrackSsstereo\n" );

    VectorXf v1 ( 2 ),v2 ( 2 );
    RowVectorXcd vr;

//     v1 << -1,3;
//     v2<< 2,5;
    VectorXf v = v2-v1;

    std::cout << "v.lpNorm<1>() = " << v.lpNorm<1>() << endl;
    std::cout << "v = " << v1<<v << endl;
    vector<int> s;
    vector<int> *p = &s;
    s.push_back ( 1 );
    s.push_back ( 2 );
    s.push_back ( 1 );
    s.push_back ( 6 );
    vector<int>  ss;
	ss.push_back(121);

    //ss.swap ( s );
	ss=s;

    for ( vector<int>::iterator it = p->begin(); it != p->end(); it++ ) {
        std::cout << *it<<endl;    //使用迭代器，正确
    }
ss[0]=100;
s[0]=500;
    for ( vector<int>::iterator it = ss.begin(); it != ss.end(); it++ ) {
        std::cout << *it<<endl;    //使用迭代器，正确
    }
    for ( vector<int>::iterator it = s.begin(); it != s.end(); it++ ) {
		std::cout << *it<<endl;    //使用迭代器，正确
	}

	for ( vector<int>::iterator it = p->begin(); it != p->end(); it++ ) {
		std::cout << *it<<endl;    //使用迭代器，正确
	}
    //std::	cout << p[0] << endl;

    printf ( "TrackSsstereo\n" );
    cout<<"fx="<<camera.fx_<<endl;
    cout<<"cx="<<camera.cx_<<endl;
    fPointBox->ExtractFPointsFromImage ( imLeft,imRight );
    if ( frameNumber > 0 ) {
        fPointBox->FeatureProcess ( frameNumber );
    }
    //currentFrame->createCurrentFrame(fPointBox);
    //tracker->TrackOne();

    frameNumber++;
}


}

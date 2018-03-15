#ifndef FEATUREPOINTBOX_H
#define FEATUREPOINTBOX_H

#include "common_include.h"
#include "system.h"
using namespace Eigen;

namespace SOFT
{
class System;

enum FeatureType {
    BlobMin = 11,		BlobMax = 12,
    CornerMin = 21,		CornerMax = 22
};

typedef Matrix<int16_t,24,1> VecDescriptor;
class FeaturePoint
{
public:

    int id;
    int age=0;
    double xLeft,yLeft,xRigt,yRigt;
    double featureStrength;
    FeatureType type;
    int16_t descriptorLeft[24];
    int16_t descriptorRigt[24];
    //initialDescriptor;

public:
    FeaturePoint();
//  FeaturePoint(double x,double y, double strength, FeatureType tp);
    ~FeaturePoint();

};
struct Point {
    int id;
    float u,v;
    double strength;
    FeatureType type;
    VecDescriptor descriptor;
	int binInd;

    Point() {}
    Point ( double u,double v,double strength,FeatureType type ) :
        u ( u ),v ( v ),strength ( strength ),type ( type ) {}

};
struct Range {
    int umin,umax;
    int vmin,vmax;
    int pointsNumber;
    Range() {}
    Range ( int umin,int umax,int vmin,int vmax ) :
        umin ( umin ),umax ( umax ),vmin ( vmin ),vmax ( vmax ),pointsNumber ( 0 ) {}
//     void Reformate() {
//         if ( pointsNumber ) {
//             umin/=pointsNumber;
//             umax/=pointsNumber;
//             vmin/=pointsNumber;
//             vmax/=pointsNumber;
//         }
//     }
};
struct FrameData {
    cv::Mat leftImage,rightImage;
    cv::Mat leftImageInt16,rightImageInt16;
    cv::Mat leftBlob,leftCorner,rightBlob,rightCorner;
    cv::Mat leftSobelVer,leftSobelHor,rightSobelVer,RightSobelHor;

    vector<Point> pointsLeftLess,pointsLeft;
    vector<Point> pointsRightLess,pointsRight;
    //ranges[0]:previousLeft to currentLeft, 1:currentLeft2currentRight,
    //2:currentR2PreviousR, 3:previousR2PreviousL
};

//0:previousLeft; 1:currentLeft; 2:currentright; 3:previousright
struct MatchFeatures {
    Point point[5];
	MatchFeatures() {}
	MatchFeatures ( Point p0,Point p1,Point p2,Point p3 )
	{
		point[0]=p0;
		point[1]=p1;
		point[2]=p2;
		point[3]=p3;
		point[4]=p0;
	}

};
class FeaturePointBox
{
private:
    System* systemPtr;
    //params:
    int nmsNeighboursmall;
    int nmsNeighbourbig;
    int nmsThreshold;
    int margin;
    int matchSADThreshold;
    FrameData dataCurrent,dataPrevious;

    int binScale;
    int binWidth,binHeight,binNum;
    int serchWindowFast;
	int serchWindowDense;
    vector<vector<Range>> rangesAll;
	int ransacIter;
public:
	cv::Point2d pp_;
	double focal_;
    vector<FeaturePoint> fPointsAll;

    vector<MatchFeatures> matchFast;
    vector<MatchFeatures> matchDense;
	SE3 tPrevious2Current;
	Eigen::Matrix3d R_Eigen;
	Vector3d tEigen;

private:
    // computes the address offset for coordinates u,v of an image of given
    //width
    inline int32_t getAddressOffsetImage ( const int32_t& u,const int32_t&
                                           v,const int32_t& width ) {
        return v*width+u;
    }
    FrameData FilterImage ( Mat leftImage, Mat rigtImage );
    void NonMaximumSuppression ( int16_t* I_f1,int16_t*I_f2,vector<Point> &points,int32_t nms_n );

    void ComputeDiscriptors ( int16_t* sobelVer,int16_t* sobleHor,vector< Point >& points );

    void InitRangesForFastMatch ( bool isBegin );
    bool MatchLoop ( bool isDense );
    int Match ( Point &point1,Point &point2, vector< Point > &points2,vector<Range> &range );
	void UpdateRanges(vector<MatchFeatures> &matches);
	double CacuReprojectError(MatchFeatures match,SE3 transform);
	void CacuTransfer5pRansac(vector<MatchFeatures> &matches);

public:
    FeaturePointBox ( System* pt );
    ~FeaturePointBox();

    bool FeatureProcess ( int frameNumber );
    bool ExtractFPointsFromImage ( cv::Mat &leftImage, cv::Mat &rigtImage );
};

}

#endif
#ifndef FEATUREPOINTBOX_H
#define FEATUREPOINTBOX_H

#include "common_include.h"
#include "system.h"
using namespace Eigen;
namespace SOFT
{
class System;

enum FeatureType
{
	BlobMin = 11,		BlobMax = 12,
	CornerMin = 21,		CornerMax = 22
};
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
struct Point{
	int id;
	double u,v;
	double strength;
	FeatureType type;
	int16_t descriptor[24];

	Point(){}
	Point(double u,double v,double strength,FeatureType type):
	u(u),v(v),strength(strength),type(type){}

};
struct Range
{
	double umin,umax;
	double vmin,vmax;
	int pointsNumber;
	Range(){}
	Range(double umin,double umax,double vmin,double vmax):
	umin(umin),umax(umax),vmin(vmin),vmax(vmax),pointsNumber(0){}
	void Reformate()
	{
        if ( pointsNumber ) {
            umin/=pointsNumber;
            umax/=pointsNumber;
            vmin/=pointsNumber;
            vmax/=pointsNumber;
        }
    }
};
struct FrameData
{
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
struct MatchFeatures
{
	Point point0;
	Point point1;
	Point point2;
	Point point3;

	MatchFeatures(){}
	MatchFeatures(Point p0,Point p1,Point p2,Point p3):
	point0(p0),point1(p1),point2(p2),point3(p3){}
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
	FrameData dataCurrent,dataPrevious;
	SE3 tPrevious2Current( Quaterniond(1,0,0,0));

	int binScale;
	int binWidth,binHeight;
	int serchWindow;
	vector<vector<Range>> ranges;
public:
    vector<FeaturePoint> fPointsAll;

	vector<MatchFeatures> matchFast;
	vector<MatchFeatures> matchDense;

private:
    // computes the address offset for coordinates u,v of an image of given
    //width
    inline int32_t getAddressOffsetImage ( const int32_t& u,const int32_t&
                                           v,const int32_t& width ) {
        return v*width+u;
    }
    FrameData FilterImage(Mat leftImage, Mat rigtImage );
	void NonMaximumSuppression (int16_t* I_f1,int16_t*I_f2,vector<Point> &points,int32_t nms_n);

	void ComputeDiscriptors(int16_t* sobelVer,int16_t* sobleHor,vector< Point >& points);

	void CacuRanges(bool isInit);
	void MatchLoop(bool isDense);
	Point Match(Point point1,vector< Point > points2,vector<Range> range);

public:
    FeaturePointBox(System* pt);
    ~FeaturePointBox();
    
    bool FeatureProcess( int frameNumber);
    bool ExtractFPointsFromImage ( cv::Mat leftImage,cv::Mat rigtImage );
};

}

#endif
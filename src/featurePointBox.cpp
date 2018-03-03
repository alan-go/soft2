#include "common_include.h"
#include "featurePointBox.h"
#include "system.h"

namespace SOFT
{

FeaturePoint::FeaturePoint(){}
FeaturePoint::~FeaturePoint(){}



FeaturePointBox::FeaturePointBox ( System* pt ):
nmsNeighboursmall(3),nmsNeighbourbig(10),nmsThreshold(50),margin(8+1)
{
    systemPtr=pt;
	binScale = 50;
	binWidth = ceil(pt->params.width/binScale);
	binHeight = ceil(pt->params.height/binScale);
	serchWindow = 20;
}
FeaturePointBox::~FeaturePointBox()
{

}

FrameData FeaturePointBox::FilterImage(Mat leftImage, Mat rigtImage )
{
	FrameData dataTemp;
	dataTemp.leftImage = leftImage;
	dataTemp.rightImage = rigtImage;


	Mat kernBlob = (cv::Mat_<char>(5,5) <<
	-1,-1,-1,-1,-1,
	-1, 1, 1, 1,-1,
	-1, 1, 8, 1,-1,
	-1, 1, 1, 1,-1,
	-1,-1,-1,-1,-1);
	Mat kernCorner = (cv::Mat_<char>(5,5) <<
	-1,-1, 0, 1, 1,
	-1,-1, 0, 1, 1,
	 0, 0, 0, 0, 0,
  	 1, 1, 0,-1,-1,
	 1, 1, 0,-1,-1);

	Mat kernSobelV = (cv::Mat_<char>(5,5) <<
	-5,-8, -10, -8,-5,
	-4,-10,-20,-10,-4,
	 0, 0,  0,  0,  0,
	 4, 10, 20, 10, 4,
	 5, 8,  10, 8,  5);

	Mat kernSobelH = (cv::Mat_<char>(5,5) <<
	-5, -4, 0, 4, 5,
	-8, -10,0, 10,8,
	-10,-20,0, 20,10,
	-8, -10,0, 10,8,
	-5, -4, 0, 4, 5);


	leftImage.convertTo(dataTemp.leftImageInt16,CV_16S);
	rigtImage.convertTo(dataTemp.rightImageInt16,CV_16S);

	cv::filter2D(dataTemp.leftImageInt16,dataTemp.leftBlob,-1,kernBlob);
	cv::filter2D(dataTemp.leftImageInt16,dataTemp.leftCorner,-1,kernCorner);
	cv::filter2D(dataTemp.rightImageInt16,dataTemp.rightBlob,-1,kernBlob);
	cv::filter2D(dataTemp.rightImageInt16,dataTemp.rightCorner,-1,kernCorner);


	cv::filter2D(dataTemp.leftImageInt16,dataTemp.leftSobelVer,-1,kernSobelV);
	cv::filter2D(dataTemp.leftImageInt16,dataTemp.leftSobelHor,-1,kernSobelH);
	cv::filter2D(dataTemp.rightImageInt16,dataTemp.rightSobelVer,-1,kernSobelV);
	cv::filter2D(dataTemp.rightImageInt16,dataTemp.RightSobelHor,-1,kernSobelH);

	return dataTemp;

}


bool FeaturePointBox::ExtractFPointsFromImage ( Mat leftImage, Mat rigtImage )
{
	//for test
	VecDescriptor des1 ;
	VecDescriptor des2 ;

	des1<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	des2<<0,2,0,2,0,0,6,0,0,0,0,0,0,0,0,0,0,0,0,0,21,22,23,15;
	cout<<"e-des1 = "<<des1<<endl;
	cout<<"e-des2 = "<<des2<<endl;
	cout<<"e-des1-2 = "<<des1-des2<<endl;

	//####################
	dataPrevious = dataCurrent;

	dataCurrent = FilterImage(leftImage,rigtImage);

	int16_t* pLeftBlob = (int16_t*)dataCurrent.leftBlob.data;
	int16_t* pLeftCorner = (int16_t*)dataCurrent.leftCorner.data;
	int16_t* pRigtBlob = (int16_t*)dataCurrent.rightBlob.data;
	int16_t* pRigtCorner = (int16_t*)dataCurrent.rightCorner.data;

	int16_t* pLeftSobleVer = (int16_t*)dataCurrent.leftSobelVer.data;
	int16_t* pLeftSobleHor = (int16_t*)dataCurrent.leftSobelHor.data;
	int16_t* pRightSobleVer = (int16_t*)dataCurrent.rightSobelVer.data;
	int16_t* pRightSobleHor = (int16_t*)dataCurrent.RightSobelHor.data;


	//计算nms，左图，右图，fast，dense
	NonMaximumSuppression(pLeftBlob,pLeftCorner,dataCurrent.pointsLeftLess,10);
	NonMaximumSuppression(pRigtBlob,pRigtCorner,dataCurrent.pointsRightLess,10);
	NonMaximumSuppression(pLeftBlob,pLeftCorner,dataCurrent.pointsLeft,3);
	NonMaximumSuppression(pRigtBlob,pRigtCorner,dataCurrent.pointsRight,3);

	for(int i =0;i<dataCurrent.pointsLeftLess.size();i++)
	{
		cv::Point center = cv::Point(dataCurrent.pointsLeftLess[i].u,dataCurrent.pointsLeftLess[i].v);
		//参数为：承载的图像、圆心、半径、颜色、粗细、线型
		cv::circle(dataCurrent.leftImage,center,2,cv::Scalar(255,0,0));
	}

	imshow("pointsLeftLess",dataCurrent.leftImage);
// 	cvWaitKey(0);
	//nms1,nms2;


	//computediscriptors
	//左图
	ComputeDiscriptors(pLeftSobleVer,pLeftSobleHor,dataCurrent.pointsLeftLess);
	ComputeDiscriptors(pLeftSobleVer,pLeftSobleHor,dataCurrent.pointsLeft);
	//右图

	ComputeDiscriptors(pRightSobleVer,pRightSobleHor,dataCurrent.pointsRightLess);
	ComputeDiscriptors(pRightSobleVer,pRightSobleHor,dataCurrent.pointsRight);
  return true;

}

bool FeaturePointBox::FeatureProcess ( int frameNumber )
{
	if(ranges.size()==4){	CacuRanges(false);}
	else{CacuRanges(true);}

	MatchLoop(false);

	//cacu range

	//matchdense
	//refine
	//cacu transfer
    return true;
}

void FeaturePointBox::CacuRanges ( bool isInit )
{
	vector<vector<Range>> result;
	int binNum = binWidth*binHeight;

    if ( !isInit ) {
        for ( int i=0; i<4; i++ ) {
            for ( int n=0; n<binNum; n++ ) {
                Range temp = ranges[i][n];
                temp.Reformate();
            }
        }
    } else {
        Range range0 ( -serchWindow,serchWindow,-serchWindow,serchWindow );
        Range rangeL2R ( -serchWindow,0,0,0 );
        Range rangeR2L ( 0,serchWindow,0,0 );

        for ( int n=0; n<binWidth*binHeight; n++ ) {
            result[0].push_back ( range0 );
			result[1].push_back ( rangeL2R );
			result[2].push_back ( range0 );
			result[3].push_back ( rangeR2L );
        }
        ranges=result;
    }
}

void FeaturePointBox::MatchLoop ( bool isDense )
{
    vector<MatchFeatures> matches;
    if ( isDense ) {
        for ( int i=0; i<dataPrevious.pointsLeft.size(); i++ ) {
            Point point0 = dataPrevious.pointsLeft[i];
            Point point1 = Match ( point0,dataCurrent.pointsLeft,ranges[0] );
            Point point2 = Match ( point1,dataCurrent.pointsRight,ranges[1] );
            Point point3 = Match ( point2,dataPrevious.pointsRight,ranges[2] );
            Point point00 = Match ( point3,dataPrevious.pointsLeft,ranges[3] );
            if ( point00.id==point0.id ) {
                matches.push_back ( MatchFeatures ( point0,point1,point2,point3 ) );
            }
        }
        matchDense = matches;
    } else {
        for ( int i=0; i<dataPrevious.pointsLeftLess.size(); i++ ) {
            Point point0 = dataPrevious.pointsLeftLess[i];
            Point point1 = Match ( point0,dataCurrent.pointsLeftLess,ranges[0] );
            Point point2 = Match ( point1,dataCurrent.pointsRightLess,ranges[1] );
            Point point3 = Match ( point2,dataPrevious.pointsRightLess,ranges[2] );
            Point point00 = Match ( point3,dataPrevious.pointsLeftLess,ranges[3] );
            if ( point00.id==point0.id ) {
                matches.push_back ( MatchFeatures ( point0,point1,point2,point3 ) );
            }
        }
        matchFast = matches;
    }
}

Point FeaturePointBox::Match ( Point point1, vector< Point > points2, vector< Range > range )
{
	//find range
	int u1 = int(point1.u);
	int v1 = int(point1.v);
	int binu = ceil(u1/binScale);
	int binv = ceil(v1/binScale);
	int binInd = binWidth*(binv-1)+binu-1;
	Range rangeTemp = range[binInd];



}


void FeaturePointBox::NonMaximumSuppression (int16_t* I_f1,int16_t*I_f2,vector<Point> &points,int32_t nms_n)
{

	// extract parameters
	int32_t width  = systemPtr->params.width;
	int32_t height = systemPtr->params.height;
	int32_t bpl    = width;
	int32_t n      = nms_n;
	int32_t tau    = nmsThreshold;

	// loop variables
	int32_t f1mini,f1minj,f1maxi,f1maxj,f2mini,f2minj,f2maxi,f2maxj;
	int32_t f1minval,f1maxval,f2minval,f2maxval,currval;
	int32_t addr;

	for (int32_t i=n+margin; i<width-n-margin;i+=n+1) {
		for (int32_t j=n+margin; j<height-n-margin;j+=n+1) {

			f1mini = i; f1minj = j; f1maxi = i; f1maxj = j;
			f2mini = i; f2minj = j; f2maxi = i; f2maxj = j;

			addr     = getAddressOffsetImage(i,j,bpl);
			f1minval = *(I_f1+addr);
			f1maxval = f1minval;
			f2minval = *(I_f2+addr);
			f2maxval = f2minval;

			for (int32_t i2=i; i2<=i+n; i2++) {
				for (int32_t j2=j; j2<=j+n; j2++) {
					addr    = getAddressOffsetImage(i2,j2,bpl);
					currval = *(I_f1+addr);
					if (currval<f1minval) {
						f1mini   = i2;
						f1minj   = j2;
						f1minval = currval;
					} else if (currval>f1maxval) {
						f1maxi   = i2;
						f1maxj   = j2;
						f1maxval = currval;
					}
					currval = *(I_f2+addr);
					if (currval<f2minval) {
						f2mini   = i2;
						f2minj   = j2;
						f2minval = currval;
					} else if (currval>f2maxval) {
						f2maxi   = i2;
						f2maxj   = j2;
						f2maxval = currval;
					}
				}
			}

			// f1 minimum
			for (int32_t i2=f1mini-n; i2<=min(f1mini+n,width-1-margin); i2++) {
				for (int32_t j2=f1minj-n; j2<=min(f1minj+n,height-1-margin);
					 j2++) {
					currval = *(I_f1+getAddressOffsetImage(i2,j2,bpl));
				if (currval<f1minval && (i2<i || i2>i+n || j2<j || j2>j+n))
					goto failed_f1min;
					 }
			}
			if (f1minval<=-tau)
				points.push_back(Point(f1mini,f1minj,f1minval,BlobMin));
			failed_f1min:;

			// f1 maximum
			for (int32_t i2=f1maxi-n; i2<=min(f1maxi+n,width-1-margin); i2++) {
				for (int32_t j2=f1maxj-n; j2<=min(f1maxj+n,height-1-margin);
					 j2++) {
					currval = *(I_f1+getAddressOffsetImage(i2,j2,bpl));
				if (currval>f1maxval && (i2<i || i2>i+n || j2<j || j2>j+n))
					goto failed_f1max;
					 }
			}
			if (f1maxval>=tau)
				points.push_back(Point(f1maxi,f1maxj,f1maxval,BlobMax));
			failed_f1max:;

			// f2 minimum
			for (int32_t i2=f2mini-n; i2<=min(f2mini+n,width-1-margin); i2++) {
				for (int32_t j2=f2minj-n; j2<=min(f2minj+n,height-1-margin);
					 j2++) {
					currval = *(I_f2+getAddressOffsetImage(i2,j2,bpl));
				if (currval<f2minval && (i2<i || i2>i+n || j2<j || j2>j+n))
					goto failed_f2min;
					 }
			}
			if (f2minval<=-tau)
				points.push_back(Point(f2mini,f2minj,f2minval,CornerMin));
			failed_f2min:;

			// f2 maximum
			for (int32_t i2=f2maxi-n; i2<=min(f2maxi+n,width-1-margin); i2++) {
				for (int32_t j2=f2maxj-n; j2<=min(f2maxj+n,height-1-margin);
					 j2++) {
					currval = *(I_f2+getAddressOffsetImage(i2,j2,bpl));
				if (currval>f2maxval && (i2<i || i2>i+n || j2<j || j2>j+n))
					goto failed_f2max;
					 }
			}
			if (f2maxval>=tau)
				points.push_back(Point(f2maxi,f2maxj,f2maxval,CornerMax));
			failed_f2max:;
    }
  }
}

  void FeaturePointBox::ComputeDiscriptors (int16_t* sobelVer,int16_t* sobleHor,vector< Point >& points )
{
	int32_t width  = systemPtr->params.width;
	int32_t height = systemPtr->params.height;
	int32_t bpl    = width;

	int addr;
	for (int i = 0; i<points.size();i++)
	{
		points[i].id = i;
		int u = int(points[i].u);
		int v = int(points[i].v);
		addr     = getAddressOffsetImage(u,v,bpl);
		{
		points[i].descriptor[0]=abs(*(sobelVer+addr-1*bpl-3))+abs(*(sobelVer+addr-1*bpl-3));
		points[i].descriptor[1]=abs(*(sobelVer+addr-1*bpl-1))+abs(*(sobelVer+addr-1*bpl-1));
		points[i].descriptor[2]=abs(*(sobelVer+addr-1*bpl+1))+abs(*(sobelVer+addr-1*bpl+1));
		points[i].descriptor[3]=abs(*(sobelVer+addr-1*bpl+3))+abs(*(sobelVer+addr-1*bpl+3));
		points[i].descriptor[4]=abs(*(sobelVer+addr+1*bpl-3))+abs(*(sobelVer+addr+1*bpl-3));
		points[i].descriptor[5]=abs(*(sobelVer+addr+1*bpl-1))+abs(*(sobelVer+addr+1*bpl-1));
		points[i].descriptor[6]=abs(*(sobelVer+addr+1*bpl+1))+abs(*(sobelVer+addr+1*bpl+1));
		points[i].descriptor[7]=abs(*(sobelVer+addr+1*bpl+3))+abs(*(sobelVer+addr+1*bpl+3));

		points[i].descriptor[8]=abs(*(sobelVer+addr-3*bpl-5))+abs(*(sobelVer+addr-3*bpl-5));
		points[i].descriptor[9]=abs(*(sobelVer+addr-3*bpl-3))+abs(*(sobelVer+addr-3*bpl-3));
		points[i].descriptor[10]=abs(*(sobelVer+addr-3*bpl-1))+abs(*(sobelVer+addr-3*bpl-1));
		points[i].descriptor[11]=abs(*(sobelVer+addr-3*bpl+1))+abs(*(sobelVer+addr-3*bpl+1));
		points[i].descriptor[12]=abs(*(sobelVer+addr-3*bpl+3))+abs(*(sobelVer+addr-3*bpl+3));
		points[i].descriptor[13]=abs(*(sobelVer+addr-3*bpl+5))+abs(*(sobelVer+addr-3*bpl+5));
		points[i].descriptor[14]=abs(*(sobelVer+addr+3*bpl-5))+abs(*(sobelVer+addr+3*bpl-5));
		points[i].descriptor[15]=abs(*(sobelVer+addr+3*bpl-3))+abs(*(sobelVer+addr+3*bpl-3));
		points[i].descriptor[16]=abs(*(sobelVer+addr+3*bpl-1))+abs(*(sobelVer+addr+3*bpl-1));
		points[i].descriptor[17]=abs(*(sobelVer+addr+3*bpl+1))+abs(*(sobelVer+addr+3*bpl+1));
		points[i].descriptor[18]=abs(*(sobelVer+addr+3*bpl+3))+abs(*(sobelVer+addr+3*bpl+3));
		points[i].descriptor[19]=abs(*(sobelVer+addr+3*bpl+5))+abs(*(sobelVer+addr+3*bpl+5));

		points[i].descriptor[20]=abs(*(sobelVer+addr-5*bpl-1))+abs(*(sobelVer+addr-5*bpl-1));
		points[i].descriptor[21]=abs(*(sobelVer+addr-5*bpl+1))+abs(*(sobelVer+addr-5*bpl+1));
		points[i].descriptor[22]=abs(*(sobelVer+addr+5*bpl-1))+abs(*(sobelVer+addr+5*bpl-1));
		points[i].descriptor[23]=abs(*(sobelVer+addr+5*bpl+1))+abs(*(sobelVer+addr+5*bpl+1));
		}
	}
}

}


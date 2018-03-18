#include "common_include.h"
#include "featurePointBox.h"
#include "system.h"
using namespace Eigen;

namespace SOFT
{

FeaturePoint::FeaturePoint() {}
FeaturePoint::~FeaturePoint() {}



FeaturePointBox::FeaturePointBox ( System* pt ) :
    nmsNeighboursmall ( 3 ),nmsNeighbourbig ( 10 ),nmsThreshold ( 50 ),margin ( 8+1 )
{
    systemPtr=pt;
    binScale = 50;
    binWidth = ceil ( pt->params.width/binScale );
    binHeight = ceil ( pt->params.height/binScale );
    binNum = binWidth*binHeight;
    serchWindowFast = 300;
    serchWindowDense = 5;
    matchSADThreshold = 100000;
    NCCThreshold = 0.98;
    focal_=systemPtr->camera.fx_;
    pp_=cv::Point2d ( systemPtr->camera.cx_,systemPtr->camera.cy_ );
    tPrevious2Current = SE3( );
    tEigen = Vector3d ( 0,0,0 );
    R_Eigen<<1,0,0,0,1,0,0,0,1;
    ransacIter = 200;
    map2D = Mat ( 1000, 1000, CV_8UC3, cv::Scalar ( 0, 0, 0 ) );
}
FeaturePointBox::~FeaturePointBox()
{

}

FrameData FeaturePointBox::FilterImage ( Mat leftImage, Mat rigtImage )
{
    FrameData dataTemp;
    dataTemp.leftImage = leftImage;
    dataTemp.rightImage = rigtImage;


    Mat kernBlob = ( cv::Mat_<char> ( 5,5 ) <<
                     -1,-1,-1,-1,-1,
                     -1, 1, 1, 1,-1,
                     -1, 1, 8, 1,-1,
                     -1, 1, 1, 1,-1,
                     -1,-1,-1,-1,-1 );
    Mat kernCorner = ( cv::Mat_<char> ( 5,5 ) <<
                       -1,-1, 0, 1, 1,
                       -1,-1, 0, 1, 1,
                       0, 0, 0, 0, 0,
                       1, 1, 0,-1,-1,
                       1, 1, 0,-1,-1 );

    Mat kernSobelV = ( cv::Mat_<char> ( 5,5 ) <<
                       -5,-8, -10, -8,-5,
                       -4,-10,-20,-10,-4,
                       0, 0,  0,  0,  0,
                       4, 10, 20, 10, 4,
                       5, 8,  10, 8,  5 );

    Mat kernSobelH = ( cv::Mat_<char> ( 5,5 ) <<
                       -5, -4, 0, 4, 5,
                       -8, -10,0, 10,8,
                       -10,-20,0, 20,10,
                       -8, -10,0, 10,8,
                       -5, -4, 0, 4, 5 );


    leftImage.convertTo ( dataTemp.leftImageInt16,CV_16S );
    rigtImage.convertTo ( dataTemp.rightImageInt16,CV_16S );

    cv::filter2D ( dataTemp.leftImageInt16,dataTemp.leftBlob,-1,kernBlob );
    cv::filter2D ( dataTemp.leftImageInt16,dataTemp.leftCorner,-1,kernCorner );
    cv::filter2D ( dataTemp.rightImageInt16,dataTemp.rightBlob,-1,kernBlob );
    cv::filter2D ( dataTemp.rightImageInt16,dataTemp.rightCorner,-1,kernCorner );


    cv::filter2D ( dataTemp.leftImageInt16,dataTemp.leftSobelVer,-1,kernSobelV );
    cv::filter2D ( dataTemp.leftImageInt16,dataTemp.leftSobelHor,-1,kernSobelH );
    cv::filter2D ( dataTemp.rightImageInt16,dataTemp.rightSobelVer,-1,kernSobelV );
    cv::filter2D ( dataTemp.rightImageInt16,dataTemp.RightSobelHor,-1,kernSobelH );

    return dataTemp;

}


bool FeaturePointBox::ExtractFPointsFromImage ( Mat &leftImage, Mat &rigtImage )
{
    dataPrevious = dataCurrent;

    dataCurrent = FilterImage ( leftImage,rigtImage );

    int16_t* pLeftBlob = ( int16_t* ) dataCurrent.leftBlob.data;
    int16_t* pLeftCorner = ( int16_t* ) dataCurrent.leftCorner.data;
    int16_t* pRigtBlob = ( int16_t* ) dataCurrent.rightBlob.data;
    int16_t* pRigtCorner = ( int16_t* ) dataCurrent.rightCorner.data;

    int16_t* pLeftSobleVer = ( int16_t* ) dataCurrent.leftSobelVer.data;
    int16_t* pLeftSobleHor = ( int16_t* ) dataCurrent.leftSobelHor.data;
    int16_t* pRightSobleVer = ( int16_t* ) dataCurrent.rightSobelVer.data;
    int16_t* pRightSobleHor = ( int16_t* ) dataCurrent.RightSobelHor.data;


    //计算nms，左图，右图，fast，dense
    NonMaximumSuppression ( pLeftBlob,pLeftCorner,dataCurrent.pointsLeftLess,10 );
    NonMaximumSuppression ( pRigtBlob,pRigtCorner,dataCurrent.pointsRightLess,10 );
    NonMaximumSuppression ( pLeftBlob,pLeftCorner,dataCurrent.pointsLeft,3 );
    NonMaximumSuppression ( pRigtBlob,pRigtCorner,dataCurrent.pointsRight,3 );
// //#################################
//     for ( int i =0; i<dataCurrent.pointsLeftLess.size(); i++ ) {
//         cv::Point center = cv::Point ( dataCurrent.pointsLeftLess[i].u,dataCurrent.pointsLeftLess[i].v );
//         //参数为：承载的图像、圆心、半径、颜色、粗细、线型
//         cv::circle ( dataCurrent.leftImage,center,2,cv::Scalar ( 255,0,0 ) );
//     }
//     imshow ( "pointsLeftLess",dataCurrent.leftImage );
//
// 	for ( int i =0; i<dataCurrent.pointsRight.size(); i++ ) {
// 		cv::Point center = cv::Point ( dataCurrent.pointsRight[i].u,dataCurrent.pointsRight[i].v );
// 		//参数为：承载的图像、圆心、半径、颜色、粗细、线型
// 		cv::circle ( dataCurrent.rightImage,center,2,cv::Scalar ( 255,0,0 ) );
// 	}
// 	imshow ( "pointsRight",dataCurrent.rightImage );
// 	cvWaitKey(0);
// //#################################


    //computediscriptors
    //左图
    ComputeDiscriptors ( pLeftSobleVer,pLeftSobleHor,dataCurrent.pointsLeftLess );
    ComputeDiscriptors ( pLeftSobleVer,pLeftSobleHor,dataCurrent.pointsLeft );
    //右图

    ComputeDiscriptors ( pRightSobleVer,pRightSobleHor,dataCurrent.pointsRightLess );
    ComputeDiscriptors ( pRightSobleVer,pRightSobleHor,dataCurrent.pointsRight );
    return true;

}

bool FeaturePointBox::FeatureProcess ( int frameNumber )
{
// 	//////////////////////////////
// 	VecDescriptor vSAD;
//     Point A0 = dataPrevious.pointsLeftLess[100];
//     for ( int i=0; i<100; i++ ) {
// 		Point temp = dataCurrent.pointsLeft[i];
//         vSAD = temp.descriptor-A0.descriptor;
//         int32_t SAD = vSAD.lpNorm<1>();
// 		std::cout<<SAD<<endl<<endl;
// 				std::cout<<vSAD<<endl<<endl;
//     }
//     std::cout<<endl<<endl;
// 	//////////////////////////////
    InitRangesForFastMatch ( frameNumber );
    //fast match
    MatchLoop ( false ) ;
//     MatchLoop ( true );


    //77777777777777777777777777777777777777777
// 	for(int i=0;i<matchFast.size();i++)
// 	{
// 		MatchFeatures temp = matchFast[i];
// 		double NCC = CacuNCC(temp.point[0],temp.point[2],dataPrevious.leftImage,dataCurrent.rightImage);
// 		std::cout<<NCC<<endl;
// 	}

// 	for(int i=0;i<matchDense.size();i++)
// 	{
// 		MatchFeatures temp = matchDense[i];
// 		double NCC = CacuNCC(temp.point[0],temp.point[2],dataPrevious.leftImage,dataCurrent.rightImage);
// 		std::cout<<NCC<<endl;
// 	}

    //77777777777777777777777777777777777777777
    //refine
    CacuTransfer5pRansac ( matchFast );
// 	CacuTransfer5pRansac ( matchDense );
    return true;
}

void FeaturePointBox::InitRangesForFastMatch ( bool isBegin )
{
    if ( !isBegin ) {
        for ( int i =0; i<4; i++ ) {
            for ( int n=0; n<binNum; n++ ) {
                Range temp = rangesAll[i][n];
                if ( temp.pointsNumber>0 ) {
                    int umin = temp.umin-serchWindowFast;
                    int umax = temp.umax+serchWindowFast;
                    int vmin = temp.vmin-serchWindowFast;
                    int vmax = temp.vmax+serchWindowFast;
                    if ( i==1||i==3 ) {
                        vmin=0;
                        vmax=0;
                    }
                    rangesAll[i][n]= Range ( umin,umax,vmin,vmax );
                }
            }
        }
    } else {
        vector<vector<Range> > result ( 4,vector<Range> ( binNum ) );

        Range range0 ( -serchWindowFast,serchWindowFast,-serchWindowFast,serchWindowFast );
        Range rangeL2R ( -serchWindowFast,0,0,0 );
        Range rangeR2L ( 0,serchWindowFast,0,0 );

        for ( int n=0; n<binNum; n++ ) {
            result[0][n] = range0 ;
            result[1][n] =  rangeL2R ;
            result[2][n] =  range0 ;
            result[3][n] =  rangeR2L ;
        }
        rangesAll=result;
    }
}

double FeaturePointBox::CacuNCC ( Point point0, Point point1, cv::Mat img0, cv::Mat img1 )
{
    int size =16;
    double result;
    Matrix<int16_t,16,16> v0,v1;
// 	Matrix<double,256,1> vd0,vd1;

    cv::Mat piece0 = img0 ( cv::Rect ( point0.u-7,point0.v-7,16,16 ) );
    cv::Mat piece1 = img1 ( cv::Rect ( point1.u-7,point1.v-7,16,16 ) );
    cv::cv2eigen ( piece0,v0 );
    cv::cv2eigen ( piece1,v1 );

    double sumAB=0,sumA2=0,sumB2=0;
    for ( int i =0; i<16; i++ ) {
        for ( int j =0; j<16; j++ ) {
            sumAB += v0 ( i,j ) *v1 ( i,j );
            sumA2 += v0 ( i,j ) *v0 ( i,j );
            sumB2 += v1 ( i,j ) *v1 ( i,j );
        }
    }
    result=sumAB/sqrt ( sumA2*sumB2 );
    return result;
}

//这里只在point0-point1 之间计算了NCC 来剔除误匹配点
bool FeaturePointBox::MatchLoop ( bool isDense )
{
    double tmep;
    vector<MatchFeatures> matches;
    Point point1,point2,point3,point00;
    if ( isDense ) {
        for ( int i=0; i<dataPrevious.pointsLeft.size(); i++ ) {
            Point point0 = dataPrevious.pointsLeft[i];
            if ( Match ( point0,point1, dataCurrent.pointsLeft,rangesAll[0] ) < matchSADThreshold )
                if ( CacuNCC ( point0,point1,dataPrevious.leftImageInt16,dataCurrent.leftImageInt16 ) >NCCThreshold )
                    if ( Match ( point1,point2, dataCurrent.pointsRight,rangesAll[1] ) <matchSADThreshold )
                        if ( Match ( point2,point3, dataPrevious.pointsRight,rangesAll[2] ) <matchSADThreshold )
                            if ( Match ( point3,point00, dataPrevious.pointsLeft,rangesAll[3] ) <matchSADThreshold )
                                if ( point00.id==point0.id ) {
                                    matches.push_back ( MatchFeatures ( point0,point1,point2,point3 ) );
                                }
        }
        matchDense = matches;
    } else {
        for ( int i=0; i<dataPrevious.pointsLeftLess.size(); i++ ) {
            Point point0 = dataPrevious.pointsLeftLess[i];
            if ( Match ( point0,point1,dataCurrent.pointsLeftLess,rangesAll[0] ) < matchSADThreshold )
                if ( CacuNCC ( point0,point1,dataPrevious.leftImageInt16,dataCurrent.leftImageInt16 ) > NCCThreshold )
                    if ( Match ( point1,point2,dataCurrent.pointsRightLess,rangesAll[1] ) < matchSADThreshold )
                        if ( Match ( point2,point3,dataPrevious.pointsRightLess,rangesAll[2] ) < matchSADThreshold )
                            if ( Match ( point3,point00,dataPrevious.pointsLeftLess,rangesAll[3] ) < matchSADThreshold )
                                if ( point00.id==point0.id ) {
                                    matches.push_back ( MatchFeatures ( point0,point1,point2,point3 ) );
                                }

        }
        matchFast = matches;
    }

    if ( matches.size() >5 ) {
        UpdateRanges ( matches );
        return true;
    } else {
        return false;
    }
}

double FeaturePointBox::CacuReprojectError ( MatchFeatures match, SE3 transform )
{
    float cu = systemPtr->camera.cx_;
    float cv = systemPtr->camera.cy_;
    float base = systemPtr->camera.base;
    float uLp = match.point[0].u;
    float vLp = match.point[0].v;
    float uRp = match.point[3].u;
    float vRp = match.point[3].v;

    double d = max ( uLp - uRp,0.0001f );
    double x = ( uLp-cu ) *base/d;
    double y = ( vLp-cv ) *base/d;
    double z = focal_*base/d;
    Vector3d point3D ( x,y,z );

    SOFT::Point pLObserve = match.point[1];
    SOFT::Point pRObserve = match.point[2];
    Vector2d pLPredict = systemPtr->camera.world2pixel ( point3D,transform );
    Vector2d pRPredict = systemPtr->camera.world2pixel ( point3D-Vector3d ( base,0,0 ),transform );

    Vector4d resV ( pLObserve.u-pLPredict ( 0 ),pLObserve.v-pLPredict ( 1 ),pRObserve.u-pRPredict ( 0 ),pRObserve.v-pRPredict ( 1 ) );

    return resV.lpNorm<1>();
}

//matches 分四类没写？
void FeaturePointBox::CacuTransfer5pRansac ( vector< MatchFeatures >& matches )
{
    cv::Mat E, R, t, mask;
    vector<vector<cv::Point2f> > cvPoint ( 4,vector<cv::Point2f> ( matches.size() ) );
    cout<<"match-size"<<"   matchFast = "<<matchFast.size() <<"    matchDense = "<<matchDense.size() <<endl;

    for ( int n =0; n<matches.size(); n++ ) {
        for ( int ind =0; ind<4; ind++ ) {
            cvPoint[ind][n].x=matches[n].point[ind].u;
            cvPoint[ind][n].y=matches[n].point[ind].v;
        }
    }

    vector<cv::Point2f> px_cur_,px_ref_;
    px_cur_= cvPoint[1];
    px_ref_ = cvPoint[0];

    //**********//
// 	for(int i =0;i<px_cur_.size();i++)
// 	{
// 		printf("x=%f,y=%f\t",px_ref_[i].x,px_ref_[i].y);
// 		printf("x=%f,y=%f\t",px_cur_[i].x,px_cur_[i].y);
// 		printf("x=%f,y=%f\t",cvPoint[2][i].x,cvPoint[2][i].y);
// 		printf("x=%f,y=%f\n",cvPoint[3][i].x,cvPoint[3][i].y);
// 	}
// 	//**********//
//
    Vector3d ttt;
    Matrix3d RRR;
    E = cv::findEssentialMat ( px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask );
    cv::recoverPose ( E, px_cur_, px_ref_, R, t, focal_, pp_,mask );
//***********//
    cv::cv2eigen ( R,RRR );

    cv::cv2eigen ( t,ttt );
    R_Eigen = RRR*R_Eigen;


    tEigen+=R_Eigen*ttt;
    std::cout<<R_Eigen<<endl;
    std::cout<<R_Eigen.inverse() <<endl;
    std::cout<<tEigen<<endl;


    cv::Point pt ( 500+tEigen ( 0 ),700-tEigen ( 2 ) );
    cv::circle ( map2D,pt,1,cv::Scalar ( 200, 100, 0 ) );
    cv::imshow ( "map",map2D );
    cvWaitKey ( 1 );
//***********//

// project matches of previous image into 3d
    vector<Vector3d> points3D;
    float cu = systemPtr->camera.cx_;
    float cv = systemPtr->camera.cy_;
    float base = systemPtr->camera.base;

    cv::RNG rng ( time ( 0 ) );

    for ( int n =0; n<matches.size(); n++ ) {
        float uLp = matches[n].point[0].u;
        float vLp = matches[n].point[0].v;
        float uRp = matches[n].point[3].u;
        float vRp = matches[n].point[3].v;
// ///////////////////////////////////////////////
// 		Mat imgl,imgr,imglc,imgrc;//
// 		dataPrevious.leftImage.convertTo(imgl,CV_8UC3); ///
// 		dataPrevious.rightImage.convertTo(imgr,CV_8UC3); ///
// 		dataCurrent.leftImage.convertTo(imglc,CV_8UC3); ///
// 		dataCurrent.rightImage.convertTo(imgrc,CV_8UC3); ///
// 		cv::Scalar color(0,0,200);
// 		cv::Point pl(uLp,vLp);//
// 		cv::Point pr(uRp,vRp);//
// 		cv::Point plc(matches[n].point[1].u,matches[n].point[1].v);
// 		cv::Point prc(matches[n].point[2].u,matches[n].point[2].v);
// 		cv::circle(imgl,pl,8,color);
// 		cv::circle(imgr,pr,8,color);
// 		cv::circle(imglc,plc,8,color);
// 		cv::circle(imgrc,prc,8,color);
// 		cv::imshow("leftPoint",imgl);
// 		cv::imshow("rightPoint",imgr);
// 		cv::imshow("leftPointc",imglc);
// 		cv::imshow("rightPointc",imgrc);
// 		cvWaitKey(0);
// 		///////////////////////////////////////////////

        double d = max ( uLp - uRp,0.0001f );
        double x = ( uLp-cu ) *base/d;
        double y = ( vLp-cv ) *base/d;
        double z = focal_*base/d;
        Vector3d point3D ( x,y,z );
        points3D.push_back ( point3D );
    }
//1 point RANSAC
    int inlinNumber = 0;
    int bestIndex = 0;
    static std::default_random_engine e ( 0 );
    static std::uniform_int_distribution<int> u ( 0, points3D.size() );

// 	Vector3d tTemp = tEigen;
//     for ( int i =0; i<ransacIter; i++ ) {
//         int ind = u ( e );
//         Vector3d pointTemp = points3D[ind];
// 		SOFT::Point pLObserve = matches[ind].point[1];
// 		SOFT::Point pRObserve = matches[ind].point[2];
//         double res = 1000;
// 		Vector3d tUpdate = tTemp;
//
// 		//gauss算出t
//         Vector3d pL = R_Eigen*pointTemp;
//         double X=pL ( 0 ),Xr = pL ( 0 )-base,Y = pL ( 1 ),Z=pL ( 2 );
//         while ( res>1e-5 ) {
//             Matrix<double,3,4> Jacobian;
// 			double Z_tz = Z+tUpdate ( 2 );
//             double Z_tz_2 = Z_tz*Z_tz;
//             Jacobian ( 0,0 ) =1/Z_tz;
//             Jacobian ( 0,1 ) =0;
//             Jacobian ( 0,2 ) =1/Z_tz;
//             Jacobian ( 0,3 ) =0;
//
//             Jacobian ( 1,0 ) =0;
//             Jacobian ( 1,1 ) =1/Z_tz;
//             Jacobian ( 1,2 ) =0;
//             Jacobian ( 1,2 ) =1/Z_tz;
//
// 			Jacobian ( 2,0 ) =- ( X+tUpdate ( 0 ) ) /Z_tz_2;
// 			Jacobian ( 2,0 ) =- ( Y+tUpdate ( 1 ) ) /Z_tz_2;
// 			Jacobian ( 2,0 ) =- ( Xr+tUpdate ( 0 ) ) /Z_tz_2;
// 			Jacobian ( 2,0 ) =- ( Y+tUpdate ( 1 ) ) /Z_tz_2;
//
// 			Matrix3d A  = Jacobian*Jacobian.transpose();
// // 			std::cout<<"A="<<endl<<A<<endl<<endl;
//
// 			Vector2d pLPredict = systemPtr->camera.world2pixel(pointTemp,SE3(R_Eigen,tUpdate));
// 			Vector2d pRPredict = systemPtr->camera.world2pixel(pointTemp-Vector3d(base,0,0),SE3(R_Eigen,tUpdate));
// 			Vector4d resV(pLObserve.u-pLPredict(0),pLObserve.v-pLPredict(1),pRObserve.u-pRPredict(0),pRObserve.v-pRPredict(1));
// 			Vector3d b = Jacobian*resV;
// 			Vector3d tOld = tUpdate;
// // 			std::cout<<"Ainverse="<<endl<<A.inverse()<<endl<<endl;
// // 			std::cout<<"resV="<<endl<<resV<<endl<<endl;
//
// 			tUpdate = tUpdate+A.inverse()*b;
// 			res = (tUpdate-tOld).lpNorm<1>();
//         }
//         tTemp = tUpdate;
//         double tx,ty,tz;
//         //tx= ;ty =
//
// 		// 	判断inliner个数
//         int inlineNumTemp = 0;
//         SE3 transfer ( R_Eigen,tTemp );
//         for ( int indT = 0; indT<matches.size(); indT++ ) {
//             Vector2d pLPredict = systemPtr->camera.world2pixel ( points3D[indT],transfer );
//             Vector2d pRPredict = systemPtr->camera.world2pixel ( points3D[indT]-Vector3d ( base,0,0 ),transfer );
//             SOFT::Point pLObserve = matches[indT].point[1];
//             SOFT::Point pRObserve = matches[indT].point[2];
//             Vector4d resV ( pLObserve.u-pLPredict ( 0 ),pLObserve.v-pLPredict ( 1 ),pRObserve.u-pRPredict ( 0 ),pRObserve.v-pRPredict ( 1 ) );
//             double res = resV.lpNorm<1>();
//             if ( res<1e-4 ) {
//                 inlineNumTemp++;
//             }
//         }
//         if ( inlinNumber<inlineNumTemp ) {
//             inlinNumber=inlineNumTemp;
//             bestIndex = ind;
// 			tEigen = tTemp;
//         }
//     }
//
// 	cout<<"ttemp"<<tTemp<<endl;

}

void FeaturePointBox::UpdateRanges ( vector< MatchFeatures >& matches )
{
    for ( int ind=0; ind<matches.size(); ind++ ) {
        MatchFeatures match = matches[ind];
        for ( int i=0; i<4; i++ ) {
            int rangeId = match.point[i].binInd;
            Range temp = rangesAll[i][rangeId];
            if ( temp.pointsNumber==0 ) {
                temp.umax=0;
                temp.umin=0;
                temp.vmax=0;
                temp.vmin=0;
            }
            int uMinus = int ( match.point[i+1].u-match.point[i].u );
            int vMinus = int ( match.point[i+1].v-match.point[i].v );
            if ( uMinus<temp.umin ) {
                temp.umin=uMinus;
            }
            if ( uMinus>temp.umax ) {
                temp.umax=uMinus;
            }
            if ( vMinus<temp.vmin ) {
                temp.vmin=vMinus;
            }
            if ( vMinus>temp.vmax ) {
                temp.vmax=vMinus;
            }
            temp.pointsNumber++;
        }
    }
}

//从points2中找出point1 的匹配点,记录在point2 中,
int FeaturePointBox::Match ( Point &point1,Point &point2, vector< Point > &points2, vector< Range > &range )
{
    //find range
    int u1 = int ( point1.u );
    int v1 = int ( point1.v );
    int binu = ceil ( u1/binScale );
    int binv = ceil ( v1/binScale );
    int binInd = binWidth* ( binv-1 ) +binu-1;
    Range rangeTemp = range[binInd];
    point1.binInd = binInd;

    int uleft = point1.u+rangeTemp.umin;
    int uright = point1.u+rangeTemp.umax;
    int vleft = point1.v+rangeTemp.vmin;
    int vright = point1.v+rangeTemp.vmax;

    int SADmin = 1000000;
    VecDescriptor vMinus;

    for ( int i = 0; i<points2.size(); i++ ) {
        int u = points2[i].u,v = points2[i].v;
        if ( point1.type==points2[i].type && u>=uleft&&u<=uright&&v>=vleft&&v<=vright ) {
            vMinus = points2[i].descriptor-point1.descriptor;
            int SAD = vMinus.lpNorm<1>();
            if ( SAD<SADmin ) {
                SADmin = SAD;
                point2 = points2[i];
            }
        }
    }

    return SADmin;
}


void FeaturePointBox::NonMaximumSuppression ( int16_t* I_f1,int16_t*I_f2,vector<Point> &points,int32_t nms_n )
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

    for ( int32_t i=n+margin; i<width-n-margin; i+=n+1 ) {
        for ( int32_t j=n+margin; j<height-n-margin; j+=n+1 ) {

            f1mini = i;
            f1minj = j;
            f1maxi = i;
            f1maxj = j;
            f2mini = i;
            f2minj = j;
            f2maxi = i;
            f2maxj = j;

            addr     = getAddressOffsetImage ( i,j,bpl );
            f1minval = * ( I_f1+addr );
            f1maxval = f1minval;
            f2minval = * ( I_f2+addr );
            f2maxval = f2minval;

            for ( int32_t i2=i; i2<=i+n; i2++ ) {
                for ( int32_t j2=j; j2<=j+n; j2++ ) {
                    addr    = getAddressOffsetImage ( i2,j2,bpl );
                    currval = * ( I_f1+addr );
                    if ( currval<f1minval ) {
                        f1mini   = i2;
                        f1minj   = j2;
                        f1minval = currval;
                    } else if ( currval>f1maxval ) {
                        f1maxi   = i2;
                        f1maxj   = j2;
                        f1maxval = currval;
                    }
                    currval = * ( I_f2+addr );
                    if ( currval<f2minval ) {
                        f2mini   = i2;
                        f2minj   = j2;
                        f2minval = currval;
                    } else if ( currval>f2maxval ) {
                        f2maxi   = i2;
                        f2maxj   = j2;
                        f2maxval = currval;
                    }
                }
            }

            // f1 minimum
            for ( int32_t i2=f1mini-n; i2<=min ( f1mini+n,width-1-margin ); i2++ ) {
                for ( int32_t j2=f1minj-n; j2<=min ( f1minj+n,height-1-margin );
                        j2++ ) {
                    currval = * ( I_f1+getAddressOffsetImage ( i2,j2,bpl ) );
                    if ( currval<f1minval && ( i2<i || i2>i+n || j2<j || j2>j+n ) ) {
                        goto failed_f1min;
                    }
                }
            }
            if ( f1minval<=-tau ) {
                points.push_back ( Point ( f1mini,f1minj,f1minval,BlobMin ) );
            }
        failed_f1min:
            ;

            // f1 maximum
            for ( int32_t i2=f1maxi-n; i2<=min ( f1maxi+n,width-1-margin ); i2++ ) {
                for ( int32_t j2=f1maxj-n; j2<=min ( f1maxj+n,height-1-margin );
                        j2++ ) {
                    currval = * ( I_f1+getAddressOffsetImage ( i2,j2,bpl ) );
                    if ( currval>f1maxval && ( i2<i || i2>i+n || j2<j || j2>j+n ) ) {
                        goto failed_f1max;
                    }
                }
            }
            if ( f1maxval>=tau ) {
                points.push_back ( Point ( f1maxi,f1maxj,f1maxval,BlobMax ) );
            }
        failed_f1max:
            ;

            // f2 minimum
            for ( int32_t i2=f2mini-n; i2<=min ( f2mini+n,width-1-margin ); i2++ ) {
                for ( int32_t j2=f2minj-n; j2<=min ( f2minj+n,height-1-margin );
                        j2++ ) {
                    currval = * ( I_f2+getAddressOffsetImage ( i2,j2,bpl ) );
                    if ( currval<f2minval && ( i2<i || i2>i+n || j2<j || j2>j+n ) ) {
                        goto failed_f2min;
                    }
                }
            }
            if ( f2minval<=-tau ) {
                points.push_back ( Point ( f2mini,f2minj,f2minval,CornerMin ) );
            }
        failed_f2min:
            ;

            // f2 maximum
            for ( int32_t i2=f2maxi-n; i2<=min ( f2maxi+n,width-1-margin ); i2++ ) {
                for ( int32_t j2=f2maxj-n; j2<=min ( f2maxj+n,height-1-margin );
                        j2++ ) {
                    currval = * ( I_f2+getAddressOffsetImage ( i2,j2,bpl ) );
                    if ( currval>f2maxval && ( i2<i || i2>i+n || j2<j || j2>j+n ) ) {
                        goto failed_f2max;
                    }
                }
            }
            if ( f2maxval>=tau ) {
                points.push_back ( Point ( f2maxi,f2maxj,f2maxval,CornerMax ) );
            }
        failed_f2max:
            ;
        }
    }
}

void FeaturePointBox::ComputeDiscriptors ( int16_t* sobelVer,int16_t* sobleHor,vector< Point >& points )
{
    int32_t width  = systemPtr->params.width;
    int32_t height = systemPtr->params.height;
    int32_t bpl    = width;

    int addr;
    for ( int i = 0; i<points.size(); i++ ) {
        points[i].id = i;
        VecDescriptor temp;
        int u = int ( points[i].u );
        int v = int ( points[i].v );
        addr     = getAddressOffsetImage ( u,v,bpl );

        temp[0]=abs ( * ( sobelVer+addr-1*bpl-3 ) ) +abs ( * ( sobelVer+addr-1*bpl-3 ) );
        temp[1]=abs ( * ( sobelVer+addr-1*bpl-1 ) ) +abs ( * ( sobelVer+addr-1*bpl-1 ) );
        temp[2]=abs ( * ( sobelVer+addr-1*bpl+1 ) ) +abs ( * ( sobelVer+addr-1*bpl+1 ) );
        temp[3]=abs ( * ( sobelVer+addr-1*bpl+3 ) ) +abs ( * ( sobelVer+addr-1*bpl+3 ) );
        temp[4]=abs ( * ( sobelVer+addr+1*bpl-3 ) ) +abs ( * ( sobelVer+addr+1*bpl-3 ) );
        temp[5]=abs ( * ( sobelVer+addr+1*bpl-1 ) ) +abs ( * ( sobelVer+addr+1*bpl-1 ) );
        temp[6]=abs ( * ( sobelVer+addr+1*bpl+1 ) ) +abs ( * ( sobelVer+addr+1*bpl+1 ) );
        temp[7]=abs ( * ( sobelVer+addr+1*bpl+3 ) ) +abs ( * ( sobelVer+addr+1*bpl+3 ) );

        temp[8]=abs ( * ( sobelVer+addr-3*bpl-5 ) ) +abs ( * ( sobelVer+addr-3*bpl-5 ) );
        temp[9]=abs ( * ( sobelVer+addr-3*bpl-3 ) ) +abs ( * ( sobelVer+addr-3*bpl-3 ) );
        temp[10]=abs ( * ( sobelVer+addr-3*bpl-1 ) ) +abs ( * ( sobelVer+addr-3*bpl-1 ) );
        temp[11]=abs ( * ( sobelVer+addr-3*bpl+1 ) ) +abs ( * ( sobelVer+addr-3*bpl+1 ) );
        temp[12]=abs ( * ( sobelVer+addr-3*bpl+3 ) ) +abs ( * ( sobelVer+addr-3*bpl+3 ) );
        temp[13]=abs ( * ( sobelVer+addr-3*bpl+5 ) ) +abs ( * ( sobelVer+addr-3*bpl+5 ) );
        temp[14]=abs ( * ( sobelVer+addr+3*bpl-5 ) ) +abs ( * ( sobelVer+addr+3*bpl-5 ) );
        temp[15]=abs ( * ( sobelVer+addr+3*bpl-3 ) ) +abs ( * ( sobelVer+addr+3*bpl-3 ) );
        temp[16]=abs ( * ( sobelVer+addr+3*bpl-1 ) ) +abs ( * ( sobelVer+addr+3*bpl-1 ) );
        temp[17]=abs ( * ( sobelVer+addr+3*bpl+1 ) ) +abs ( * ( sobelVer+addr+3*bpl+1 ) );
        temp[18]=abs ( * ( sobelVer+addr+3*bpl+3 ) ) +abs ( * ( sobelVer+addr+3*bpl+3 ) );
        temp[19]=abs ( * ( sobelVer+addr+3*bpl+5 ) ) +abs ( * ( sobelVer+addr+3*bpl+5 ) );

        temp[20]=abs ( * ( sobelVer+addr-5*bpl-1 ) ) +abs ( * ( sobelVer+addr-5*bpl-1 ) );
        temp[21]=abs ( * ( sobelVer+addr-5*bpl+1 ) ) +abs ( * ( sobelVer+addr-5*bpl+1 ) );
        temp[22]=abs ( * ( sobelVer+addr+5*bpl-1 ) ) +abs ( * ( sobelVer+addr+5*bpl-1 ) );
        temp[23]=abs ( * ( sobelVer+addr+5*bpl+1 ) ) +abs ( * ( sobelVer+addr+5*bpl+1 ) );


        temp = temp/16;
        points[i].descriptor = temp;
    }
}

}


#include "common_include.h"
#include "featurePointBox.h"
#include "system.h"
using namespace Eigen;
using namespace g2o;
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
    focal_=systemPtr->camera->fx_;
    pp_=cv::Point2d ( systemPtr->camera->cx_,systemPtr->camera->cy_ );
    tPrevious2Current = SE3( );
    tCurrent2Word = Vector3d ( 0,0,0 );
    rCurrent2Word<<1,0,0,0,1,0,0,0,1;
    ransacIter = 200;
    map2D = Mat ( 1000, 1000, CV_8UC3, cv::Scalar ( 0, 0, 0 ) );
}
FeaturePointBox::~FeaturePointBox()
{

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
    MatchFeatures  matchtemp = matchFast[0];
    cout<<matchtemp.ppoints[0]->u<<endl;
    matchtemp.ppoints[4]->u = 10;
    cout<<matchtemp.ppoints[0]->u<<endl;
    cout<<matchFast[0].ppoints[0]->u<<endl;

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

void FeaturePointBox::RefineToSubPixel ( vector< MatchFeatures >& matches )
{
    for ( int i=0; i<matches.size(); i++ ) {
		Point *point0 = matches[i].ppoints[0];
// 		CalcuRefineDesc(u,v,soV,soH)
        for ( int ii = 1; ii<4; ii++ ) {
            Point *pointii = matches[i].ppoints[ii];
// 			CalcuRefineDesc(pointii)
			vector<double> fitPatch;
			Matrix<double,13,1> fitPatch::Zero();

			CalcuFitPatch(point0,pointii,fitPatch);

			Matrix<double,13,6> X::Zero();
			//qqXi
        }
    }
}

void FeaturePointBox::CalcuFitPatch ( Point* p0, Point* pi, Matrix<double,13,1> fitPatc )
{

}




double FeaturePointBox::CacuNCC ( Point *point0, Point *point1, cv::Mat &img0, cv::Mat &img1 )
{
    int size =16;
    double result;
    Matrix<int16_t,16,16> v0,v1;
// 	Matrix<double,256,1> vd0,vd1;

    cv::Mat piece0 = img0 ( cv::Rect ( point0->u-7,point0->v-7,16,16 ) );
    cv::Mat piece1 = img1 ( cv::Rect ( point1->u-7,point1->v-7,16,16 ) );
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
    vector<MatchFeatures> matchesTemp;
    Point *ppoint1,*ppoint2,*ppoint3,*ppoint00;
    if ( isDense ) {
        for ( int i=0; i<dataPrevious.pointsLeft.size(); i++ ) {
            Point *point0 = & ( dataPrevious.pointsLeft[i] );
            if ( Match ( point0,ppoint1, dataCurrent.pointsLeft,rangesAll[0] ) < matchSADThreshold )
                if ( CacuNCC ( point0,ppoint1,dataPrevious.leftImageInt16,dataCurrent.leftImageInt16 ) >NCCThreshold )
                    if ( Match ( ppoint1,ppoint2, dataCurrent.pointsRight,rangesAll[1] ) <matchSADThreshold )
                        if ( Match ( ppoint2,ppoint3, dataPrevious.pointsRight,rangesAll[2] ) <matchSADThreshold )
                            if ( Match ( ppoint3,ppoint00, dataPrevious.pointsLeft,rangesAll[3] ) <matchSADThreshold )
                                if ( ppoint00->id==point0->id ) {
                                    matchesTemp.push_back ( MatchFeatures ( point0,ppoint1,ppoint2,ppoint3 ) );
                                }
        }
        matchDense = matchesTemp;
    } else {
        for ( int i=0; i<dataPrevious.pointsLeftLess.size(); i++ ) {
            Point *point0 = & ( dataPrevious.pointsLeftLess[i] );
            if ( Match ( point0,ppoint1,dataCurrent.pointsLeftLess,rangesAll[0] ) < matchSADThreshold )
                if ( CacuNCC ( point0,ppoint1,dataPrevious.leftImageInt16,dataCurrent.leftImageInt16 ) > NCCThreshold )
                    if ( Match ( ppoint1,ppoint2,dataCurrent.pointsRightLess,rangesAll[1] ) < matchSADThreshold )
                        if ( Match ( ppoint2,ppoint3,dataPrevious.pointsRightLess,rangesAll[2] ) < matchSADThreshold )
                            if ( Match ( ppoint3,ppoint00,dataPrevious.pointsLeftLess,rangesAll[3] ) < matchSADThreshold )
                                if ( ppoint00->id==point0->id ) {
                                    matchesTemp.push_back ( MatchFeatures ( point0,ppoint1,ppoint2,ppoint3 ) );
                                }
        }
        matchFast = matchesTemp;
    }

    if ( matchesTemp.size() >5 ) {
        UpdateRanges ( matchesTemp );
        return true;
    } else {
        return false;
    }
}

double FeaturePointBox::CacuReprojectError ( MatchFeatures match, SE3 transform )
{
    float cu = systemPtr->camera->cx_;
    float cv = systemPtr->camera->cy_;
    float base = systemPtr->camera->base;
    float uLp = match.ppoints[0]->u;
    float vLp = match.ppoints[0]->v;
    float uRp = match.ppoints[3]->u;
    float vRp = match.ppoints[3]->v;

    double d = max ( uLp - uRp,0.0001f );
    double x = ( uLp-cu ) *base/d;
    double y = ( vLp-cv ) *base/d;
    double z = focal_*base/d;
    Vector3d point3D ( x,y,z );

    SOFT::Point* pLObserve = match.ppoints[1];
    SOFT::Point* pRObserve = match.ppoints[2];
    Vector2d pLPredict = systemPtr->camera->world2pixel ( point3D,transform );
    Vector2d pRPredict = systemPtr->camera->world2pixel ( point3D-Vector3d ( base,0,0 ),transform );

    Vector4d resV ( pLObserve->u-pLPredict ( 0 ),pLObserve->v-pLPredict ( 1 ),pRObserve->u-pRPredict ( 0 ),pRObserve->v-pRPredict ( 1 ) );

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
            cvPoint[ind][n].x=matches[n].ppoints[ind]->u;
            cvPoint[ind][n].y=matches[n].ppoints[ind]->v;
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
    E = cv::findEssentialMat ( px_cur_, px_ref_, focal_, pp_, cv::RANSAC, 0.999, 1.0, mask );
// 	cout<<mask<<endl;
    cv::recoverPose ( E, px_cur_, px_ref_, R, t, focal_, pp_,mask );

//***********//
    cv::cv2eigen ( R,rCurrent2Previous );
    cv::cv2eigen ( t,tCurrent2Previous );

    Matrix<double,Dynamic,1> maskV;
    cv::cv2eigen ( mask,maskV );
//	cout<<maskV<<endl;




// project matches of previous image into 3d
    vector<Vector3d> points3D,points3DC;
    float cu = systemPtr->camera->cx_;
    float cv = systemPtr->camera->cy_;
    float base = systemPtr->camera->base;

    cv::RNG rng ( time ( 0 ) );

    for ( int n =0; n<matches.size(); n++ ) {
        float uLp = matches[n].ppoints[0]->u;
        float vLp = matches[n].ppoints[0]->v;
        float uRp = matches[n].ppoints[3]->u;
        float vRp = matches[n].ppoints[3]->v;

        double d = max ( uLp - uRp,0.0001f );
        double x = ( uLp-cu ) *base/d;
        double y = ( vLp-cv ) *base/d;
        double z = focal_*base/d;
        Vector3d point3D ( x,y,z );
        points3D.push_back ( point3D );
    }

    for ( int n =0; n<matches.size(); n++ ) {
        float uLp = matches[n].ppoints[1]->u;
        float vLp = matches[n].ppoints[1]->v;
        float uRp = matches[n].ppoints[2]->u;
        float vRp = matches[n].ppoints[2]->v;

        double d = max ( uLp - uRp,0.0001f );
        double x = ( uLp-cu ) *base/d;
        double y = ( vLp-cv ) *base/d;
        double z = focal_*base/d;
        Vector3d point3D ( x,y,z );
        points3DC.push_back ( point3D );
    }




    /*
    //看关键点
    for(int i=0;i<matches.size();i++)
    {

    	Point point0 = matches[i].point[0];
    	Point point1 = matches[i].point[1];
    	Point point2 = matches[i].point[2];
    	Point point3 = matches[i].point[3];
    	cout<<point0.u<<" "<<point0.v<<" "<<point3.u<<" "<<point3.v<<endl;
    	cout<<point1.u<<" "<<point1.v<<" "<<point2.u<<" "<<point2.v<<endl;
    	cout<<points3D[i].transpose()<<endl;
    	cout<<points3DC[i].transpose()<<endl;
    	cout<<(points3D[i]-rCurrent2Previous* points3DC[i]).transpose()<<endl<<endl;

    	///////////////////////////////////////////////
    	Mat imgl,imgr,imglc,imgrc;//
    	dataPrevious.leftImage.convertTo(imgl,CV_8UC3); ///
    	dataPrevious.rightImage.convertTo(imgr,CV_8UC3); ///
    	dataCurrent.leftImage.convertTo(imglc,CV_8UC3); ///
    	dataCurrent.rightImage.convertTo(imgrc,CV_8UC3); ///
    	cv::Scalar color(200,200,200);
    	cv::Point pl(matches[i].point[0].u,matches[i].point[0].v);//
    	cv::Point pr(matches[i].point[3].u,matches[i].point[3].v);
    	cv::Point plc(matches[i].point[1].u,matches[i].point[1].v);
    	cv::Point prc(matches[i].point[2].u,matches[i].point[2].v);
    	cv::circle(imgl,pl,8,color);
    	cv::circle(imgr,pr,8,color);
    	cv::circle(imglc,plc,8,color);
    	cv::circle(imgrc,prc,8,color);
    	cv::imshow("leftPoint",imgl);
    	cv::imshow("rightPoint",imgr);
    	cv::imshow("leftPointc",imglc);
    	cv::imshow("rightPointc",imgrc);
    	cvWaitKey(0);
    	///////////////////////////////////////////////
    }





    //看每个点的 冲投影误差
    for (int i=0;i<matches.size();i++){
    Vector3d t = rCurrent2Previous*points3DC[i]-points3D[i];
    SE3 T(rCurrent2Previous.inverse(),-t);
    cout<<"t = " <<t<<endl;
    for(int n = 0;n<matches.size();n++)
    {
    	Point uvLObserve = matches[n].point[1];
    	Point uvRObserve = matches[n].point[2];
    	Vector3d XYZ = systemPtr->camera->world2camera(points3D[n],T);

    	Vector2d uvleftPredict = systemPtr->camera->camera2pixel(XYZ);
    	Vector2d uvRightPredict = systemPtr->camera->camera2pixel(XYZ-Vector3d(base,0,0));
    	Vector4d resV(uvLObserve.u-uvleftPredict(0),uvLObserve.v-uvleftPredict(1),uvRObserve.u-uvRightPredict(0),uvRObserve.v-uvRightPredict(1));
    	cout<<"resv "<<n<<endl<<resV<<endl;
    	double res  = resV.squaredNorm();
    	cout<<"resReproj = "<<res<<endl;
    }
    }*/







//1 point RANSAC
    int inlinNumber = 0;
    int bestIndex = 0;
    static std::default_random_engine e ( 0 );
    static std::uniform_int_distribution<int> u ( 0, points3D.size() );


    // 构建图优化，先设定g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,3> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block ( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose ( true );



    // 往图中增加顶点
    VertexPoseT * poset = new VertexPoseT();
    poset->setEstimate ( tCurrent2Previous );

    poset->setId ( 0 );
    optimizer.addVertex ( poset );

    // 往图中增加边
    vector<EdgeXYZt*> edges;

    for ( int i=0; i<matches.size(); i++ ) {

        int ind = u ( e );
        ind =i;
//             cout<<ind<<endl;
        VertexPoseT * xyz = new VertexPoseT();
        xyz->setEstimate ( points3D[ind] );
        xyz->setId ( 1+i );
        xyz->setMarginalized ( true );
        optimizer.addVertex ( xyz );

        EdgeXYZt* edge = new EdgeXYZt ( rCurrent2Previous,systemPtr->camera );
        edge->setId ( i );
        // 设置连接的顶点
        edge->setVertex ( 0, poset );
        edge->setVertex ( 1, xyz );

        SOFT::Point *p0 = matches[ind].ppoints[0];
        SOFT::Point *p1 = matches[ind].ppoints[1];
        SOFT::Point *p2 = matches[ind].ppoints[2];
        SOFT::Point *p3 = matches[ind].ppoints[3];

        Matrix<double,8,1> observe;
        observe<< p0->u,p0->v,p1->u,p1->v,p2->u,p2->v,p3->u,p3->v;
        edge->setMeasurement ( observe );
        // 信息矩阵：协方差矩阵之逆
        edge->setInformation ( Matrix<double,8,8>::Identity() *10 );
        edge->setRobustKernel ( new g2o::RobustKernelHuber() );
        optimizer.addEdge ( edge );

        edges.push_back ( edge );
    }
    cout<<"edge number : "<<edges.size() <<endl;
    // 执行优化
    cout<<"start 1st optimization"<<endl;
    optimizer.initializeOptimization();
    optimizer.optimize ( 10 );

    // 输出优化值
    tCurrent2Previous = poset->estimate();
    cout<<"estimated model: "<<tCurrent2Previous.transpose() <<endl;
    cout<<"edge number : "<<edges.size() <<endl;
//////////////////////////2

    vector<int> matcheSelect1st,matchesFalse;
    int inliers = 0;
    for ( int n=0; n<edges.size(); n++ ) {
        auto e = edges[n];
        cout<<e->id() <<" error= "<<e->L2 <<endl;
        cout<<"measurement "<<e->Measur.transpose() <<endl;
        cout<<"calcu       "<<e-> calcu.transpose() <<endl;
        cout<<"error       "<< ( e->Measur-e->calcu ).transpose() <<endl;

        optimizer.edges();
        if ( e->chi2() > 100 ) {
            optimizer.removeEdge ( e );
            matchesFalse.push_back ( n );
        } else {
            inliers++;
            matcheSelect1st.push_back ( n );
        }
    }

    cout<<"start 2nd optimization"<<endl;
    cout<<"edge number : "<<optimizer.edges().size() <<endl;

    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    tCurrent2Previous = poset->estimate();
    cout<<"estimated model: "<<tCurrent2Previous.transpose() <<endl;
////////////////////3
    vector<int> matcheSelect2st;
    for ( int n=0; n<matcheSelect1st.size(); n++ ) {
        auto e = edges[matcheSelect1st[n]];
        cout<<e->id() <<" error= "<<e->L2 <<endl;
        cout<<"measurement "<<e->Measur.transpose() <<endl;
        cout<<"calcu       "<<e-> calcu.transpose() <<endl;
        cout<<"error       "<< ( e->Measur-e->calcu ).transpose() <<endl;
        if ( e->chi2() > 3 ) {
            optimizer.removeEdge ( e );
            matchesFalse.push_back ( matcheSelect1st[n] );
        } else {
            inliers++;
            matcheSelect2st.push_back ( matcheSelect1st[n] );
        }
    }

    cout<<"start 3nd optimization"<<endl;
    cout<<"edge number : "<<optimizer.edges().size() <<endl;

    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    tCurrent2Previous = poset->estimate();
    cout<<"estimated model: "<<tCurrent2Previous.transpose() <<endl;



//     for ( int n=0; n<matcheSelect2st.size(); n++ ) {
//         auto e = edges[matcheSelect2st[n]];
//         cout<<e->id() <<" error= "<<e->L2 <<endl;
//         cout<<"measurement "<<e->Measur.transpose() <<endl;
//         cout<<"calcu       "<<e-> calcu.transpose() <<endl;
//         cout<<"error       "<< ( e->Measur-e->calcu ).transpose() <<endl;
//         if ( e->chi2() > 1 ) {
//             optimizer.removeEdge ( e );
//             matchesFalse.push_back ( n );
//         } else {
//             inliers++;
//             //matcheSelect2st.push_back ( n );
//         }
//     }


////***********//
    rCurrent2Word = rCurrent2Previous*rCurrent2Word;

    tCurrent2Word+=rCurrent2Word*tCurrent2Previous;
    std::cout<<rCurrent2Word<<endl;
    std::cout<<tCurrent2Word<<endl;
    cv::Point pt ( 500+tCurrent2Word ( 0 ),700-tCurrent2Word ( 2 ) );
    cv::circle ( map2D,pt,1,cv::Scalar ( 200, 100, 0 ) );
    cv::imshow ( "map",map2D );

    cvWaitKey ( 1 );
    cv::imwrite ( "map.jpg",map2D );

////***********//






    /*

        cout<<"match num select"<<matcheSelect1st.size() <<endl;
        //看关键点
        for ( int n=0; n<matcheSelect1st.size(); n++ ) {
            int i = matcheSelect1st[n];
            Point point0 = matches[i].point[0];
            Point point1 = matches[i].point[1];
            Point point2 = matches[i].point[2];
            Point point3 = matches[i].point[3];
            cout<<point0.u<<" "<<point0.v<<" "<<point3.u<<" "<<point3.v<<endl;
            cout<<point1.u<<" "<<point1.v<<" "<<point2.u<<" "<<point2.v<<endl;
            cout<<points3D[i].transpose() <<endl;
            cout<<points3DC[i].transpose() <<endl;
            auto e = edges[i];
            cout<<e->id() <<" error= "<<e->L2 <<endl;
            cout<<"measurement "<<e->Measur.transpose() <<endl;
            cout<<"calcu       "<<e-> calcu.transpose() <<endl;
            cout<<"error       "<< ( e->Measur-e->calcu ).transpose() <<endl;
            cout<< ( points3D[i]-rCurrent2Previous* points3DC[i] ).transpose() <<endl<<endl;

            ///////////////////////////////////////////////
            Matrix3d rotateGroundTruth;
    		Vector3d tGroundTruth( -0.0469029,-0.0283993,0.858694 );
            rotateGroundTruth<<    0.999998 , 0.000527263 , -0.00206694,
                              -0.000529651  ,   0.999999 , -0.00115487,
                              0.00206632  , 0.00115596 ,    0.999997;
            Vector2d uv0,uv1,uv2,uv3;
            Vector3d XYZ0,XYZ1,XYZ2,XYZ3,Vb ( 1,0,0 );
            XYZ0 = points3D[i];
            XYZ3 = XYZ0-Vb;
            XYZ1 = rotateGroundTruth.inverse() * ( XYZ0-tCurrent2Previous );
            XYZ2 = XYZ1-Vb;
            uv0 = systemPtr->camera->camera2pixel ( XYZ0 );
            uv1 = systemPtr->camera->camera2pixel ( XYZ1 );
            uv2 = systemPtr->camera->camera2pixel ( XYZ2 );
            uv3 = systemPtr->camera->camera2pixel ( XYZ3 );
            Mat img0,img3,img1,img2;//
            cvtColor ( dataPrevious.leftImage,img0,CV_GRAY2RGB );
            cvtColor ( dataPrevious.rightImage,img3,CV_GRAY2RGB );
            cvtColor ( dataCurrent.leftImage,img1,CV_GRAY2RGB );
            cvtColor ( dataCurrent.rightImage,img2,CV_GRAY2RGB );


            cv::Scalar color0 ( 200,0,0 );
            cv::Scalar color1 ( 0,200,0 );
            int rcircle = 2;
            cv::Point pl ( matches[i].point[0].u,matches[i].point[0].v ); //
            cv::Point pr ( matches[i].point[3].u,matches[i].point[3].v );
            cv::Point plc ( matches[i].point[1].u,matches[i].point[1].v );
            cv::Point prc ( matches[i].point[2].u,matches[i].point[2].v );


            cv::circle ( img0,pl,rcircle,color0 );
            cv::circle ( img3,pr,rcircle,color0 );
            cv::circle ( img1,plc,rcircle,color0 );
            cv::circle ( img2,prc,rcircle,color0 );

            cv::circle ( img0,cv::Point ( uv0 ( 0 ),uv0 ( 1 ) ),rcircle,color1 );
            cv::circle ( img3,cv::Point ( uv3 ( 0 ),uv3 ( 1 ) ),rcircle,color1 );
            cv::circle ( img1,cv::Point ( uv1 ( 0 ),uv1 ( 1 ) ),rcircle,color1 );
            cv::circle ( img2,cv::Point ( uv2 ( 0 ),uv2 ( 1 ) ),rcircle,color1 );

    		cv::imshow ( "img0",img0 );
    		cv::imshow ( "img3",img3 );
    		cv::imshow ( "img1",img1 );
    		cv::imshow ( "img2",img2 );
            cvWaitKey ( 0 );
            ///////////////////////////////////////////////
        }*/















//     for ( int i =0; i<ransacIter; i++ ) {
//         int ind = u ( e );
//         Vector3d pointTemp = points3D[ind];
//         SOFT::Point pLObserve = matches[ind].point[1];
//         SOFT::Point pRObserve = matches[ind].point[2];
// 		Vector4d observe ( pLObserve.u,pLObserve.v,pRObserve.u,pRObserve.v );
// 		Vector4d observe2 ( matches[ind+1].point[1].u,matches[ind+1].point[1].v,matches[ind+1].point[2].u,matches[ind+1].point[2].v );///
//
//         // 构建图优化，先设定g2o
//         // 每个误差项优化变量维度为3，误差值维度为1
//         typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,4>> Block;
//         // 线性方程求解器
//         Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
//         // 矩阵块求解器
//         Block* solver_ptr = new Block ( linearSolver );
//         // 梯度下降方法，从GN, LM, DogLeg 中选
// //         g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
// 		g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
//         // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
//         // 图模型
//         g2o::SparseOptimizer optimizer;
//         // 设置求解器
//         optimizer.setAlgorithm ( solver );
//         // 打开调试输出
//         optimizer.setVerbose ( true );
//
//         // 往图中增加顶点
//         VertexPoseT * v = new VertexPoseT();
//         v->setEstimate ( tCurrent2Previous);
//         v->setId ( 0 );
//         optimizer.addVertex ( v );
//
//         // 往图中增加边
// for (int i=0;i<190;i++){
// 	if(maskV(i)){
//         EdgePoseT* edge = new EdgePoseT ( points3D[i],rCurrent2Previous.inverse(),systemPtr->camera );
//         edge->setId ( i );
// 		// 设置连接的顶点
//         edge->setVertex ( 0, v );
//
// 		SOFT::Point pLObserve = matches[i].point[1];
// 		SOFT::Point pRObserve = matches[i].point[2];
// 		Vector4d observe ( pLObserve.u,pLObserve.v,pRObserve.u,pRObserve.v );
// 		// 观测数值
//         edge->setMeasurement ( observe );
// 		// 信息矩阵：协方差矩阵之逆
//         edge->setInformation ( Eigen::Matrix<double,4,4>::Identity());
//         optimizer.addEdge ( edge );
// }}
//         // 执行优化
//         cout<<"start optimization"<<endl;
//         chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//         optimizer.initializeOptimization();
//         optimizer.optimize ( 100 );
//         chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//         chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
//         cout<<"solve time cost = "<<time_used.count() <<" seconds. "<<endl;
// 		optimizer.clear();
//         // 输出优化值
//         tCurrent2Previous = v->estimate();
//         cout<<"estimated model: "<<tCurrent2Previous.transpose() <<endl;
//     }




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
            int rangeId = match.ppoints[i]->binInd;
            Range temp = rangesAll[i][rangeId];
            if ( temp.pointsNumber==0 ) {
                temp.umax=0;
                temp.umin=0;
                temp.vmax=0;
                temp.vmin=0;
            }
            int uMinus = int ( match.ppoints[i+1]->u-match.ppoints[i]->u );
            int vMinus = int ( match.ppoints[i+1]->v-match.ppoints[i]->v );
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
int FeaturePointBox::Match ( Point *&ppoint1,Point *&ppoint2, vector< Point > &points2, vector< Range > &range )
{
    //find range
    int u1 = int ( ppoint1->u );
    int v1 = int ( ppoint1->v );
    int binu = ceil ( u1/binScale );
    int binv = ceil ( v1/binScale );
    int binInd = binWidth* ( binv-1 ) +binu-1;
    Range rangeTemp = range[binInd];
    ppoint1->binInd = binInd;

    int uleft = ppoint1->u+rangeTemp.umin;
    int uright = ppoint1->u+rangeTemp.umax;
    int vleft = ppoint1->v+rangeTemp.vmin;
    int vright = ppoint1->v+rangeTemp.vmax;

    int SADmin = 1000000;
    VecDescriptor vMinus;

    for ( int i = 0; i<points2.size(); i++ ) {
        int u = points2[i].u,v = points2[i].v;
        if ( ppoint1->type==points2[i].type && u>=uleft&&u<=uright&&v>=vleft&&v<=vright ) {
            vMinus = points2[i].descriptor-ppoint1->descriptor;
            int SAD = vMinus.lpNorm<1>();
            if ( SAD<SADmin ) {
                SADmin = SAD;
                ppoint2 = & ( points2[i] );
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

void FeaturePointBox::CalcuDescriptors ( int16_t* sobelVer,int16_t* sobelHor,vector< Point >& points )
{
    int32_t width  = systemPtr->params.width;
    int32_t height = systemPtr->params.height;
    int32_t bpl    = width;

    int addr;
    for ( int i = 0; i<points.size(); i++ ) {
        points[i].id = i;
        points[i].sobelV = sobelVer;
        points[i].sobelH = sobelHor;
        VecDescriptor tempForMatch ;
        int u = int ( points[i].u );
        int v = int ( points[i].v );
        addr     = getAddressOffsetImage ( u,v,bpl );
//ComputeDiscriptors for matching
        tempForMatch[0]=abs ( * ( sobelVer+addr-1*bpl-3 ) ) +abs ( * ( sobelHor+addr-1*bpl-3 ) );
        tempForMatch[1]=abs ( * ( sobelVer+addr-1*bpl-1 ) ) +abs ( * ( sobelHor+addr-1*bpl-1 ) );
        tempForMatch[2]=abs ( * ( sobelVer+addr-1*bpl+1 ) ) +abs ( * ( sobelHor+addr-1*bpl+1 ) );
        tempForMatch[3]=abs ( * ( sobelVer+addr-1*bpl+3 ) ) +abs ( * ( sobelHor+addr-1*bpl+3 ) );
        tempForMatch[4]=abs ( * ( sobelVer+addr+1*bpl-3 ) ) +abs ( * ( sobelHor+addr+1*bpl-3 ) );
        tempForMatch[5]=abs ( * ( sobelVer+addr+1*bpl-1 ) ) +abs ( * ( sobelHor+addr+1*bpl-1 ) );
        tempForMatch[6]=abs ( * ( sobelVer+addr+1*bpl+1 ) ) +abs ( * ( sobelHor+addr+1*bpl+1 ) );
        tempForMatch[7]=abs ( * ( sobelVer+addr+1*bpl+3 ) ) +abs ( * ( sobelHor+addr+1*bpl+3 ) );

        tempForMatch[8]=abs ( * ( sobelVer+addr-3*bpl-5 ) ) +abs ( * ( sobelHor+addr-3*bpl-5 ) );
        tempForMatch[9]=abs ( * ( sobelVer+addr-3*bpl-3 ) ) +abs ( * ( sobelHor+addr-3*bpl-3 ) );
        tempForMatch[10]=abs ( * ( sobelVer+addr-3*bpl-1 ) ) +abs ( * ( sobelHor+addr-3*bpl-1 ) );
        tempForMatch[11]=abs ( * ( sobelVer+addr-3*bpl+1 ) ) +abs ( * ( sobelHor+addr-3*bpl+1 ) );
        tempForMatch[12]=abs ( * ( sobelVer+addr-3*bpl+3 ) ) +abs ( * ( sobelHor+addr-3*bpl+3 ) );
        tempForMatch[13]=abs ( * ( sobelVer+addr-3*bpl+5 ) ) +abs ( * ( sobelHor+addr-3*bpl+5 ) );
        tempForMatch[14]=abs ( * ( sobelVer+addr+3*bpl-5 ) ) +abs ( * ( sobelHor+addr+3*bpl-5 ) );
        tempForMatch[15]=abs ( * ( sobelVer+addr+3*bpl-3 ) ) +abs ( * ( sobelHor+addr+3*bpl-3 ) );
        tempForMatch[16]=abs ( * ( sobelVer+addr+3*bpl-1 ) ) +abs ( * ( sobelHor+addr+3*bpl-1 ) );
        tempForMatch[17]=abs ( * ( sobelVer+addr+3*bpl+1 ) ) +abs ( * ( sobelHor+addr+3*bpl+1 ) );
        tempForMatch[18]=abs ( * ( sobelVer+addr+3*bpl+3 ) ) +abs ( * ( sobelHor+addr+3*bpl+3 ) );
        tempForMatch[19]=abs ( * ( sobelVer+addr+3*bpl+5 ) ) +abs ( * ( sobelHor+addr+3*bpl+5 ) );

        tempForMatch[20]=abs ( * ( sobelVer+addr-5*bpl-1 ) ) +abs ( * ( sobelHor+addr-5*bpl-1 ) );
        tempForMatch[21]=abs ( * ( sobelVer+addr-5*bpl+1 ) ) +abs ( * ( sobelHor+addr-5*bpl+1 ) );
        tempForMatch[22]=abs ( * ( sobelVer+addr+5*bpl-1 ) ) +abs ( * ( sobelHor+addr+5*bpl-1 ) );
        tempForMatch[23]=abs ( * ( sobelVer+addr+5*bpl+1 ) ) +abs ( * ( sobelHor+addr+5*bpl+1 ) );

        tempForMatch = tempForMatch/16;
        points[i].descriptor = tempForMatch;

//         //Compute small Discriptors for refine
//         Eigen::Matrix<int16_t,15,1> tempForRefine;
//         tempForRefine[0]=abs ( * ( sobelVer+addr-2*bpl ) ) +abs ( * ( sobleHor+addr-2*bpl ) );
//         tempForRefine[1]=abs ( * ( sobelVer+addr-1*bpl-2 ) ) +abs ( * ( sobleHor+addr-1*bpl-2 ) );
//         tempForRefine[2]=abs ( * ( sobelVer+addr-1*bpl-1 ) ) +abs ( * ( sobleHor+addr-1*bpl-1 ) );
//         tempForRefine[3]=abs ( * ( sobelVer+addr-1*bpl ) ) +abs ( * ( sobleHor+addr-1*bpl ) );
//         tempForRefine[4]=abs ( * ( sobelVer+addr-1*bpl+1 ) ) +abs ( * ( sobleHor+addr-1*bpl+1 ) );
//         tempForRefine[5]=abs ( * ( sobelVer+addr-1*bpl+2 ) ) +abs ( * ( sobleHor+addr-1*bpl+2 ) );
//
//         tempForRefine[6]=abs ( * ( sobelVer+addr-1 ) ) +abs ( * ( sobleHor+addr-1 ) );
//         tempForRefine[7]=3*abs ( * ( sobelVer+addr ) ) +abs ( * ( sobleHor+addr ) );
//         tempForRefine[8]=abs ( * ( sobelVer+addr+1 ) ) +abs ( * ( sobleHor+addr+1 ) );
//
//         tempForRefine[9]=abs ( * ( sobelVer+addr+1*bpl-2 ) ) +abs ( * ( sobleHor+addr+1*bpl-2 ) );
//         tempForRefine[10]=abs ( * ( sobelVer+addr+1*bpl-1 ) ) +abs ( * ( sobleHor+addr+1*bpl-1 ) );
//         tempForRefine[11]=abs ( * ( sobelVer+addr+1*bpl ) ) +abs ( * ( sobleHor+addr+1*bpl ) );
//         tempForRefine[12]=abs ( * ( sobelVer+addr+1*bpl+1 ) ) +abs ( * ( sobleHor+addr+1*bpl+1 ) );
//         tempForRefine[13]=abs ( * ( sobelVer+addr+1*bpl+2 ) ) +abs ( * ( sobleHor+addr+1*bpl+2 ) );
//         tempForRefine[14]=abs ( * ( sobelVer+addr+2*bpl ) ) +abs ( * ( sobleHor+addr+2*bpl ) );
//
//
//         tempForRefine = tempForRefine/16;
//         points[i].descriptorRefine = tempForRefine;
    }
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

    int16_t* pLeftSobleV = ( int16_t* ) dataCurrent.leftSobelVer.data;
    int16_t* pLeftSobleH = ( int16_t* ) dataCurrent.leftSobelHor.data;
    int16_t* pRightSobleV = ( int16_t* ) dataCurrent.rightSobelVer.data;
    int16_t* pRightSobleH = ( int16_t* ) dataCurrent.RightSobelHor.data;


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
    // 		cv::circle ( dataCurrent.rightImage,center,2,cv::Scalar ( 255,0,0 ) );
    // 	}
    // 	imshow ( "pointsRight",dataCurrent.rightImage );
    // 	cvWaitKey(0);
    // //#################################


    //computediscriptors
    //左图
    CalcuDescriptors ( pLeftSobleV,pLeftSobleH,dataCurrent.pointsLeftLess );
    CalcuDescriptors ( pLeftSobleV,pLeftSobleH,dataCurrent.pointsLeft );
    //右图

    CalcuDescriptors ( pRightSobleV,pRightSobleH,dataCurrent.pointsRightLess );
    CalcuDescriptors ( pRightSobleV,pRightSobleH,dataCurrent.pointsRight );
    return true;

}

}

































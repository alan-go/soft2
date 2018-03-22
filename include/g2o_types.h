#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include "common_include.h"
#include "camera.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>

using namespace Eigen;
namespace SOFT
{
	class VertexPoseT: public g2o::BaseVertex<3, Eigen::Vector3d>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		virtual void setToOriginImpl() // 重置
		{
			_estimate << 0,0,0;
		}

		virtual void oplusImpl( const double* update ) // 更新
		{
			_estimate += Eigen::Vector3d(update);
		}
		// 存盘和读盘：留空
		virtual bool read( istream& in ) {}
		virtual bool write( ostream& out ) const {}
	};

/*
	class EdgeXYZt: public g2o::BaseBinaryEdge<8,Matrix<double,8,1>,VertexPoseT,VertexPoseT>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		EdgeXYZt( ): BaseBinaryEdge() {
			vb<<base,0,0;
		}
		// 计算曲线模型误差
		void computeError() {
			const VertexPoseT* vt = static_cast<const VertexPoseT*> ( _vertices[0] );
			const VertexPoseT* vXYZ = static_cast<const VertexPoseT*> ( _vertices[1] );
			Eigen::Vector3d t = vt->estimate();
			Eigen::Vector3d XYZ0 = vXYZ->estimate();

			Eigen::Vector3d XYZ3 = XYZ0-vb;

			Eigen::Vector3d XYZ1,XYZ2;
			XYZ1 = XYZ0-t;
			XYZ2 = XYZ1-vb;

			Eigen::Vector2d uv0,uv3,uv1,uv2;


			uv0 = Camera2pixel ( XYZ0 );
			uv3 = Camera2pixel ( XYZ3 );
			uv1 = Camera2pixel ( XYZ1 );
			uv2 = Camera2pixel ( XYZ2 );

			Eigen::Matrix<double,8,1> calcu;
			calcu<<uv0 ( 0 ),uv0 ( 1 ),uv1 ( 0 ),uv1 ( 1 ),uv2 ( 0 ),uv2 ( 1 ),uv3 ( 0 ),uv3 ( 1 );

			double coefW =1;

			_error = _measurement-calcu;

			cout<<_error.transpose()<<endl;
			cout<<t.transpose()<<endl;
			//    cout<<_error.transpose() <<endl;
			// 		cout<<t.transpose()<<endl;
		}

		Vector2d Camera2pixel(Vector3d p)
		{
			double u = focal*p(0)/p(2)+cx;
			double v = focal*p(1)/p(2)+cy;
			return Vector2d(u,v);
		}
		virtual bool read( istream& in ) {}
		virtual bool write( ostream& out ) const {}
	public:
		double _x;  // x 值， y 值为 _measurement
		Vector3d point;
		double focal = 718.856;
		double cx =  607.1928;
		double cy= 185.2157;
		double base= 1.0;
		Vector3d vb;
	};*/








	class EdgeXYZt: public g2o::BaseBinaryEdge<8 ,Eigen::Matrix<double,8,1>,VertexPoseT,VertexPoseT>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		EdgeXYZt ( Eigen::Matrix3d rotate, SOFT::Camera* camera ) : BaseBinaryEdge(),
		rotate ( rotate ),camera ( camera ) {}

		void computeError() {
			const VertexPoseT* vt = static_cast<const VertexPoseT*> ( _vertices[0] );
			const VertexPoseT* vXYZ = static_cast<const VertexPoseT*> ( _vertices[1] );
			Eigen::Vector3d t = vt->estimate();
			Eigen::Vector3d XYZ0 = vXYZ->estimate();

			Eigen::Vector3d XYZ3 = XYZ0-Vector3d ( camera->base,0,0 );

			Eigen::Vector3d XYZ1,XYZ2;
			XYZ1 = rotate.inverse() * XYZ0-t;
			XYZ2 = XYZ1-Vector3d ( camera->base,0,0 );

			Eigen::Vector2d uv0,uv3,uv1,uv2;


			uv0 = camera->camera2pixel ( XYZ0 );
			uv3 = camera->camera2pixel ( XYZ3 );
			uv1 = camera->camera2pixel ( XYZ1 );
			uv2 = camera->camera2pixel ( XYZ2 );

			calcu<<uv0 ( 0 ),uv0 ( 1 ),uv1 ( 0 ),uv1 ( 1 ),uv2 ( 0 ),uv2 ( 1 ),uv3 ( 0 ),uv3 ( 1 );

			double coefW =1;

			_error = _measurement-calcu;
// 			   cout<<_error.transpose() <<endl;
// 				cout<<t.transpose()<<endl;
			L2 = _error.squaredNorm();
		}

		virtual bool read ( istream& in ) {}
		virtual bool write ( ostream& out ) const {}
	public:
		Eigen::Matrix3d rotate;
		Camera* camera;
		double L2;
		Eigen::Matrix<double,8,1> calcu;

	};

















// 误差模型 模板参数：观测值维度，类型，连接顶点类型


class EdgePoseT: public g2o::BaseUnaryEdge<4,Eigen::Vector4d,VertexPoseT>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgePoseT ( Eigen::Vector3d point, Eigen::Matrix3d rotate, SOFT::Camera* camera ) : BaseUnaryEdge(),
        point ( point ),rotate ( rotate ),camera ( camera ) {}
    // 计算曲线模型误差
    void computeError() {
        const VertexPoseT* v = static_cast<const VertexPoseT*> ( _vertices[0] );
        const Eigen::Vector3d t = v->estimate();
        Eigen::Vector3d pointCameraLeft,pointCameraRight;
        Eigen::Vector2d uvLeft,uvRight;

        pointCameraLeft = rotate*point-t;
        pointCameraRight = pointCameraLeft-Vector3d ( camera->base,0,0 );
        uvLeft = camera->camera2pixel ( pointCameraLeft );
        uvRight = camera->camera2pixel ( pointCameraRight );

        double coefW =1;
        Eigen::Vector4d res ( _measurement ( 0 )-uvLeft ( 0 ),_measurement ( 1 )-uvLeft ( 1 ),_measurement ( 2 )-uvRight ( 0 ),_measurement ( 3
)-uvRight ( 1 ) );

        _error = coefW*res;
    }
//     void linearizeOplus();

    virtual bool read ( istream& in ) {}
    virtual bool write ( ostream& out ) const {}
public:
// 		measurement 是观测值
    Eigen::Vector3d point;
    Eigen::Matrix3d rotate;
    Camera* camera;
};










class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
};

// only to optimize the pose, no point
class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Error: measure = R*point+t
    virtual void computeError();
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
    Vector3d point_;
};

class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual void computeError();
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};
    
    Vector3d point_;
    Camera* camera_;
};

}


#endif // MYSLAM_G2O_TYPES_H

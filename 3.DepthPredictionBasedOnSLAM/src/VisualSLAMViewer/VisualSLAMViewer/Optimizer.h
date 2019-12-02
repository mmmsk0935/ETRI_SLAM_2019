#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include "Eigen/StdVector"
#include "Eigen/Dense"

#include "armadillo"
#include "Frame.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

class CVertexPlane : public g2o::BaseVertex<4, Eigen::Vector4d>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	CVertexPlane() {};
	~CVertexPlane() {};

	virtual bool read(std::istream& /*is*/)
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual bool write(std::ostream& /*os*/) const
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual void setToOriginImpl()
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
	}

	virtual void oplusImpl(const double* update)
	{
		Eigen::Vector4d::ConstMapType v(update);
		_estimate += v;
	}
};

class CEdgePointOnPlaneOnlyPose : public g2o::BaseUnaryEdge<1, Eigen::Vector3d, CVertexPlane>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	CEdgePointOnPlaneOnlyPose() {};
	~CEdgePointOnPlaneOnlyPose() {};

	virtual bool read(std::istream& /*is*/)
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual bool write(std::ostream& /*os*/) const
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual void oplusImpl(const double* update)
	{
		const CVertexPlane* plane = static_cast<const CVertexPlane*>(vertex(0));

		const Eigen::Vector3d& normal = plane->estimate().head<3>();
		const double &distance = plane->estimate()(3);
		_error(0) = (normal.dot(measurement()) + distance)/normal.norm();
	}
private:

};

class CEdgePointOnPlane : public g2o::BaseBinaryEdge<1, double, CVertexPlane, g2o::VertexSBAPointXYZ>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	CEdgePointOnPlane() {};
	~CEdgePointOnPlane() {};

	virtual bool read(std::istream& /*is*/)
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual bool write(std::ostream& /*os*/) const
	{
		std::cerr << __PRETTY_FUNCTION__ << " not implemented yet" << std::endl;
		return false;
	}

	virtual void computeError()
	{
		const CVertexPlane* plane = static_cast<const CVertexPlane*>(_vertices[0]);
		const g2o::VertexSBAPointXYZ* point = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[1]);
		
		const Eigen::Vector3d& normal = plane->estimate().head<3>();
		const double &distance = plane->estimate()(3);
		
		_error(0) = (normal.dot(point->estimate()) + distance)/normal.norm();
	}
private:

};

class COptimizer
{
public:
	COptimizer();
	~COptimizer();

	static	g2o::SE3Quat	ConvertARMAtoSE3(const arma::mat& R, const arma::vec& t);
	static	arma::mat		ConvertSE3toARMA(const g2o::SE3Quat& se3);
	static	arma::vec		ConvertVector3dtoARMA(const Eigen::Vector3d& pos);
	static	arma::vec		ConvertVector4dtoARMA(const Eigen::Vector4d& abcd);

	static	Eigen::Matrix<double, 3, 1>	ConvertARMAtoVector3d(const arma::vec& pos);
	static	Eigen::Matrix<double, 4, 1>	ConvertARMAtoVector4d(const arma::vec& abcd);

	static	void	OptimizePose(CFrame& frame);
	static	void	OptimizeInitStructure(CSLAMKeyFrame* pPrevKeyFrame, CSLAMKeyFrame* pCurrKeyFrame);
};


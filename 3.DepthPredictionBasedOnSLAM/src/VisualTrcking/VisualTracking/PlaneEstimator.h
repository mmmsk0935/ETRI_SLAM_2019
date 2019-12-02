#pragma once

#include <ppl.h>
#include <assert.h>
#include <concurrent_vector.h>
#include <random>
#include "armadillo"

#ifndef PLANE_SAMPLE_COUNT
#define	PLANE_SAMPLE_COUNT	3
#endif // !PLANE_SAMPLE_COUNT


class CPlaneEstimator
{
private:
	arma::mat	InitWorldData;	///< the 3xN initial world point for plane estimation
	arma::vec	PlaneNormal;	///< the 3x1 normal vector of the plane

	double	m_dbDistance;	///< the distance from the origin
	double	m_dbThreshold;	///< the tolerance of the distance from the plane to a point

	Concurrency::concurrent_vector<int>	m_vInlierDataIdx;	///< the inlier data index list
	int	m_nInlierDataCount;	///< the number of inlier data

	std::mt19937	en;	///< the random number generator engine
public:
	CPlaneEstimator();
	~CPlaneEstimator();

	///@	brief	Set the initial 3D world point for plane estimation
	///@	param	[in]	WorldData	: 3xN 3D world point
	void	SetInitData(const arma::mat& WorldData);

	///@	brief	Set the tolerance of the distance error from the plane to a point
	///@	param	[in]	dbThreshold	: the distance error threshold in meter
	void	SetThreshold(double dbThreshold = 0.1);

	///@	brief	Estimate the plane by RANSAC
	///@	param	[in]	nRansacIteration	: the number of iteration for RANSAC <br>
	///@			[in]	bUseCUDA			: if it be set to true, the process can be performed on GPU
	void	EstimatePlaneByRANSAC(int nRansacIteration = 500, bool bUseCUDA = false);

	///@	brief	Get the normal vector of the estimated plane
	const arma::vec&	GetNormalVector(void);

	///@	brief	Get the distance from the origin to the estimate plane
	double	GetDistance(void);

	///@	brief	Get the number of inlier data
	int		GetInlierDataCount(void);

	///@	brief	Get the inlier data index list
	const Concurrency::concurrent_vector<int>&	GetInlierDataIndices(void);

private:
	///@	brief	Select the sample data for plane estimation by RANSAC
	///@	param	[out]	pSampleDataIdx	: the index list of a sample data
	void	SetSampleData(int* pSampleDataIdx);

	///@	brief	Estimate plane using selected sample data
	///@	param	[out]	EPlaneNormal, dbDistance	: the estimated plane normal and distance <br>
	///@			[in]	pSampleDataIdx				: the index list of sample data
	///@	return	true	: success <br>
	///@			false	: otherwise(if the sample points are colinear)
	bool	EstimatePlaneFromSample(arma::vec& EPlaneNormal, double& dbDistance, const int* pSampleDataIdx);

	///@	brief	Calculate the number of inlier data by the given estimated plane parameters
	///@	param	[in]	EPlaneNormal, dbDistance	: the estimated plane normal and distance
	///@	return	the number of inlier data
	int		ComputeInlierDataCount(const arma::vec& EPlaneNormal, double dbDistance);

	///@	brief	Calculate the inlier data
	void	ComputeInlierData(void);
};


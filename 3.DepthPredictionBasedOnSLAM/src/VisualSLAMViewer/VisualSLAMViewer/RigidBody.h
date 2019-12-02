#pragma once

#include <vector>
#include <ppl.h>
#include <concurrent_vector.h>
#include <random>
#include "armadillo"

#ifndef RIGID_BODY_SAMPLE_COUNT
#define	RIGID_BODY_SAMPLE_COUNT	3
#endif // !RIGID_BODY_SAMPLE_COUNT

//#define	E_SYMMETRIC_DISTANCE

class CRigidBody
{
private:
	arma::mat	SrcInitWorldPoint, DstInitWorldPoint;	///< the 3xN initial source and destination 3D world point set
	arma::mat	SrcInitImagePoint, DstInitImagePoint;	///< the 3xN initial source and destination 2D image point set
	arma::mat	SrcK, DstK;	///< the 3x3 source and destination camera matrix
	
	arma::mat	R;	///< 3x3 SE(3) rotation matrix
	arma::vec	T;	///< 3x1 trasnlation vector
	arma::vec	w;	///< parameterization of R by Rodriguez formula

	double	m_dbThreshold;	///< the tolerance of the reprojection error

	int	m_nInlierDataCount;	///< the number of inlier data
	std::vector<int>	m_vInlierDataIdx;	///< the index list of inlier data

	std::mt19937	en;	///< the random number generator engine
public:
	CRigidBody();
	~CRigidBody();

	///@	brief	Set the initial data set for Euclidean transformation
	///@	param	[in]	SrcWorldData, SrcImageData	: the 3d world point and 2d source image point set <br>
	///@			[in]	DstWorldData, DstImageData	: the 3d world point and 2d destination image point set
	void	SetInitData(const arma::mat& SrcWorldData, const arma::mat& SrcImageData,
						const arma::mat& DstWorldData, const arma::mat& DstImageData);
	
	///@	brief	Set the camera parameter
	///@	param	[in]	SrcIntrinsic, DstIntrinsic	: the source and destination 3x3 camera parameter
	void	SetIntrinsic(const arma::mat& SrcIntrinsic, const arma::mat& DstIntrinsic);

	///@	brief	Set the tolerance of the reprojection distance error in pixel
	///@	param	[in]	dbThreshold	: the reprojection error tolerance
	void	SetThreshold(double dbThreshold = 1.0);

	///@	brief	Estimate the Euclidean transformation by RANSAC
	///@	param	[in]	nRansacIteration	: the number of iteration for RANSAC <br>
	///@			[in]	bUseCUDA			: if it be set to true, the process can be performed on GPU
	void	EstimateRigidBodyTransformationByRANSAC(int nRansacIteration = 500, bool bUseCUDA = false);

	///@	brief	Get the 3x3 SE(3) group rotation matrix
	const arma::mat&	GetR(void);

	///@	brief	Get the 3x1 translation vector
	const arma::vec&	GetT(void);

	///@	brief	Get the number of inlier data
	int	GetInlierDataCount(void);

	///@	brief	Get the index list of inlier data
	const std::vector<int>&	GetInlierDataIndices(void);
private:
	///@	brief	Select the sample data
	///@	param	[out]	pSampleDataIdx	: the index list of sample data
	void	SetSampleData(int* pSampleDataIdx);

	///@	brief	Estimate the Euclidean transformation using the sample data
	///@	param	[out]	ER, ET			: the estimated Euclidean transformation(rotation matrix and translation vector) <br>
	///@			[in]	pSampleDataIdx	: the index list of sample data
	bool	EstimateRigidBodyTransformationFromSample(arma::mat& ER, arma::vec& ET, const int* pSampleDataIdx);

	///@	brief	Calculate the number of inlier data
	///@	param	[in]	ER, ET	: the estimated Euclidean transformation(rotation matrix and translation vector) <br>
	///@	return	The number of inlier data
	int		ComputeInlierDataCount(const arma::mat& ER, const arma::vec& ET);

	///@	brief	Calculate the inlier data for the Euclidean transformation by MLE(maximum likelihood estimation) <br>
	///@			The index of inlier data is stored in m_vInlierDataIdx
	void	ComputeInlierData(void);
};
#pragma once

#include <random>
#include <vector>
#include <ppl.h>
#include <assert.h>
#include <concurrent_vector.h>
#include "armadillo"

#ifndef EIGHT_POINT_SAMPLE_COUNT
#define	EIGHT_POINT_SAMPLE_COUNT	8
#endif // !EIGHT_POINT_SAMPLE_COUNT

#define	SOLVE_F_BY_SVD
//#define	F_SYMMETRIC_DISTANCE

class CEightPoint
{
private:
	bool	m_bUseInitK;

	arma::mat	SrcInitData;	///< 3xN source input data
	arma::mat	DstInitData;	///< 3xN destination input data
	arma::mat	SrcNormData;	///< 3xN normalized input data
	arma::mat	DstNormData;	///< 3xN normlaized input data
	arma::mat	SrcK, DstK;		///< 3x3 intrinsic parameter of source and destination camera
	arma::mat	E;				///< 3x3 estimated essential matrix
	arma::mat	F;				///< the estimated fundamental matrix
	arma::vec	e1;				///< the epipole for right camera
	arma::vec	e2;				///< the epipole for left camera
	arma::mat	R;				///< estimated rotation matrix
	arma::vec	t;				///< estimated translation vector

	double	m_dbThreshold;	///< distance threshold from a epipolar line to a point
	double	m_dbLambda;		
	double	m_dbScore;

	int		m_nInlierDataCount;	///< # of inlier data

	std::vector<int>	m_vInlierDataIdx;	///< inlier data index list

	std::mt19937 en;	///< random number generator engine
public:
	CEightPoint();
	~CEightPoint();

	void	SetK(const arma::mat& SrcIntrinsic, const arma::mat& DstIntrinsic);

	///@	brief	set initial data
	///@	param	[in]	pSrcData		: the point set of source scene(x1, y1) <br>
	///@			[in]	pDstData		: the point set of destination scene(x2, y2) <br>
	///@			[in]	nInitDataCount	: the number of corresponding points <br>
	void	SetInitData(double pSrcData[][2], double pDstData[][2], int nInitDataCount);

	///@	brief	set initial data
	///@	param	[in]	SrcData		: the point set of source scene <br>
	///@			[in]	DstData		: the point set of destination scene <br>
	void	SetInitData(const arma::mat& SrcData, const arma::mat& DstData);

	///@	 brief	set the tolerance error of epiolar geometry
	///@	 param	[in]	dbThreshold	:  the distance threshold from a epipolar line to a feature point
	void	SetThreshold(double dbThreshold = 1.0, double dbLambda = 1.96);

	///@	brief	estimate fundamental matrix using RANSAC
	///@	param	[in]	nRansacIteration	: the maximum number of RANSAC iteration <br>
	///@			[in]	bUseCUDA			: if it be set to true, the process can be performed on GPU
	void	EstimateFundamentalMatrixByRANSAC(int nRansacIteration = 500);

	bool	EstimateRelativePose(void);

	///@	brief	Get the number of inlier data
	int		GetInlierDataCount(void);

	///@	brief	Get the inlier data index list
	const std::vector<int>&	GetInlierDataIndices(void);

	///@	brief	Get the fundamental matrix
	const arma::mat&	GetF(void);

	///@	brief	Get the rotation matrix
	const arma::mat&	GetR(void);

	///@	brief	Get the translation vector
	const arma::vec&	GetT(void);
	
	///@	brief	Get the fundamental matrix score <br>
	double	GetScore(void);

	///@	brief	Get the epipoles
	///@	param	[out]	ep1, ep2	: the epipole for left and right camera, respectively
	void	GetEpipoles(arma::vec& ep1, arma::vec& ep2);

private:
	///@	brief	set samples from initial data set for RANSAC
	///@	param	[out]	pSampleDataIdx	: the index list of sample data
	void	SelectSampleData(int* pSampleDataIdx);

	///@	brief	estimate fundamental matrix from given samples by solving linear equation
	///@	param	[out]	EF	: the 3x3 estimated fundamental matrix <br>
	///@			[in]	SrcNormData, DstNormData	: the normalized sample data of source and destination, respectively
	///@	return	true	: the estimation is sucees <br>
	///@			false	: otherwise
	bool	EstimateFundamentalMatrixFromSample(arma::mat& EF, const arma::mat& SrcNormData, const arma::mat& DstNormData);

	///@	brief	normalize the given data(isotropic scaling)
	void	NormalizeSampleData(arma::mat& SrcNormData, arma::mat& DstNormData, arma::mat& SrcT, arma::mat& DstT, const int* pSampleDataIdx);

	///@	brief	denormalize a estimated fundamental matrix
	///@	param	[in, out]	EF	: a estimated fundamental matrix
	void	Denormalize(arma::mat& EF, const arma::mat& SrcT, const arma::mat& DstT);

	///@	brief	enforce the constraints of fundamental matrix(rank 2)
	///@	param	[in, out]	EF	: a estimated fundamental matrix
	void	EnforceFundamentalMatrixConstraint(arma::mat& EF);

	///@	brief	calculate the number of inlier data
	///@	param	[in]	EF	: a estimated fundamental matrix
	///@	return	the number of inlier data
	int		ComputeInlierDataCount(const arma::mat& EF);

	///@	brief	calculate the inlier data by the estimated fundamental matrix
	void	ComputeInlierData(void);

	///@	brief	calculate the sampson distance
	///@	param	EF	: a estimated fundamental matrix
	///@	param	x1	: a feature point in left image
	///@	param	x2	: the corresponding feature point in right image
	double	ComputeSampsonDistance(arma::mat& EF, arma::mat& x1, arma::mat& x2);

	///@	brief	calculate the symmetric distance
	///@	param	EF	: a estimated fundamental matrix
	///@	param	x1	: a feature point in left image
	///@	param	x2	: the corresponding feature point in right image
	double	ComputeSymmetricDistance(arma::mat& EF, arma::mat& x1, arma::mat& x2);

	///@	brief	compute epipole from fundamental matrix
	void	ComputeEpipole(void);

	///@	brief	get detph from the current estimated camera pose
	///@	param	X	: a two-view reconstructed 3D world point
	///@	param	P	: the current estimated camera pose by fundamental(or essential) matrix decomposition
	///@	return	depth
	double	GetDepth(const arma::vec& X, const arma::mat& P);

	///@	brief	get the number of tow-view reconstructed 3D world points in front of the given two camera poses
	///@	param	P1, P2	: the current estimated camera poses
	///@	return	the number of 3D world points
	int		Get3DPointCountInFrontOfCamera(const arma::mat& P1, const arma::mat& P2);
};
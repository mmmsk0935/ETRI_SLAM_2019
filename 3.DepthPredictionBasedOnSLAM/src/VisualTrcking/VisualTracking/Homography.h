#pragma once

#include <random>
#include <ppl.h>
#include <assert.h>
#include <concurrent_vector.h>
#include "armadillo"

#ifndef HOMOGRAPHY_SAMPLE_COUNT
#define	HOMOGRAPHY_SAMPLE_COUNT	4
#endif // !HOMOGRAPHY_SAMPLE_COUNT

//#define	SOLVE_H_BY_SVD

//#define	H_SYMMETRIC_DISTANCE

class CHomography
{
private:
	arma::mat	H;			///< 3x3 projective transformation matrix(homography)
	arma::mat	SrcNormData, DstNormData;	///< normialized data
	arma::mat	SrcInitData;	///< the source input data set(3xN)
	arma::mat	DstInitData;	///< the destination input data set(3xN)

	/* The input data set can be transformed that the centroid of the refernce points is at origin of the cooridnates and RMS distance of the points from the origin is equal to sqrt(2) */
	arma::mat	SrcK;		///< the 3x3 transformation for source data
	arma::mat	DstK;		///< the 3x3 transformation for destination data

	/*	If the source input data from a plane, the pose can be estimated by EstimateMotionFromHomography() method	*/
	arma::mat	R;			///< The 3x3 SE(3) rotation matrix
	arma::vec	t;			///< The 3x1 translation vector

	int	m_nInlierDataCount;	///< the number of inlier data
	Concurrency::concurrent_vector<int>	m_vInlierDataIdx;	///< the inlier data indices
	Concurrency::concurrent_vector<int>	m_vOutlierDataIdx;
	double	m_dbThreshold;	///< the reporjection threshold in pixel
	double	m_dbLambda;

	std::mt19937	en;	///< random number generator engine
public:
	CHomography();
	~CHomography();

	///@	brief	Set the transformation matrix of source and destination, they is consisted of the translation and scaling
	///@	param	[in]	SrcIntrinsic, DstIntrinsic	: the source and destination transformation matrix, respectively.
	void	SetIntrinsic(arma::mat& SrcIntrinsic, arma::mat& DstIntrinsic);

	///@	brief	Set the initial data set
	///@	param	[in]	SrcData, DstData	: the 3xN source and destination input data set, respectively.
	void	SetInitData(const arma::mat& SrcData, const arma::mat& DstData);

	///@	brief	Set the reprojection threshold
	///@	param	[in]	dbThreshold	: the tolerance of the RMS reprojection distance error
	void	SetThreshold(double dbThreshold = 1.0, double dbLambda = 1.96);
	
	///@	brief	Estimate the 3x3 projective transformation(homography) by RANSAC
	///@	param	[in]	nRansacInteration	: the number of iteration for RANSAC
	///@			[in]	bUseCUDA			: if it be set to true, the process can be performed on GPU
	void	EstimateHomographyByRANSAC(int nRansacIteration = 500, bool bUseCUDA = false);

	///@	brief	Refine the homography using the inlier data set by LM-optimization
	//void	RefineHomographyWithInlierData(void);

	bool	ExtractCameraMotionFromHomography(const arma::mat& K);
	///@	brief	Estimate camera motion from the homography. <br>
	///@			If the source input data set is on a plane, the camera pose can be esitmated.
	///@	param	[in] K	: the intrinsic parameter of a camera.
	bool	EstimateRelativePose(const arma::mat& K);

	bool	GenerateRelativePoseHyphothesis(const arma::mat& K, std::vector<arma::mat>& vR, std::vector<arma::vec>& vT, std::vector<arma::vec>& vN);

	///@	brief	Get the 3x3 SE(3) rotation matrix
	arma::mat&	GetR(void);

	///@	brief	Get the 3x1 translation vector
	arma::vec&	GetT(void);

	///@	brief	Get the 3x3 projective transformation
	arma::mat&	GetHomography(void);

	///@	brief	Get the number of inlier data
	int	GetInlierDataCount(void);

	///@	brief	Get the index list of inlier data
	Concurrency::concurrent_vector<int>&	GetInlierDataIndices(void);

	Concurrency::concurrent_vector<int>&	GetOutlierDataIndices(void);

private:
	///@	brief	Normalize the sample data set as described in Multiple View Geomtry. <br>
	///@			The input data set can be transformed that the centroid of the refernce points is at origin of the cooridnates <br>
	///@			and RMS distance of the points from the origin is equal to sqrt(2).
	///@	param	[out]	SrcNormData, DstNormData	: the normalized sample data of source and destination, respectively <br>
	///@			[out]	SrcT, DstT					: the 3x3 transformation matrix of source and destination, respectively <br>
	///@			[in]	pSampleDataIdx				: the index list of sample data
	void	NormalizeSampleData(arma::mat& SrcNormData, arma::mat& DstNormData, 
								arma::mat& SrcT, arma::mat& DstT, const int* pSampleDataIdx);

	///@	brief	Denormalize the estimated homography. <br>
	///@			EH' = DstT^(-1)HSrcT.
	void	DenormalizeHomography(arma::mat& EH, const arma::mat& SrcT, const arma::mat& DstT);

	///@	brief	Select sample data from initial data set
	///@	param	[out]	pSampleIdx	: the index list of sample data
	void	SetSampleData(int* pSampleIdx);

	///@	brief	Estimate homography from sample data. <br>
	///@			The homography can be parameterized by m' = Hm. 
	///@			If SOLVE_H_BY_SVD is activated, the linear equation Ax = 0 is solved by SVD.
	///@			Otherwise, it can be solved by Ax = b, since the H(3, 3) can be set to 1.
	///@	param	[out]	EH			: the 3x3 estimated homography <br>
	///@			[in]	SrcNormData, DstNormData	: the normalized sample data of source and destination, respectively.
	///@	return	true	: the linear equation can be solved. <br>
	///@			false	: otherwise(if the equation is Ax = b, there is no inverse of A)
	bool	EstimateHomographyFromSample(arma::mat& EH, const arma::mat& SrcNormData, const arma::mat& DstNormData);

	///@	brief	Calculate the number of inlier data. <br>
	///@			If H_SYMMETRIC_DISTANCE is activated, d(m', Hm) + d(H^-1m', m). <br>
	///@			otherwise, d(m', Hm).
	///@	param	[in]	EH	: the 3x3 estimate homography by sample data
	///@	return	the number of inlier data
	int		ComputeInlierDataCount(arma::mat& EH);

	///@	brief	Calculate the inlier data as same method in ComputeInlierDataCount method. <br>
	///@			The inlier data indices are stored to m_vInlierDataIdx.
	void	ComputeInlierData(void);

	double	GetDepth(const arma::vec& X, const arma::mat& P);

	int		Get3DPointCountInFrontOfCamera(const arma::mat& P1, const arma::mat& P2);
};
#pragma once

#include <iostream>
#include <vector>
#include <ppl.h>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "armadillo"

#include "MapPoint.h"
#include "VisualSLAMConfigure.h"
#include "SiftGPUWrapper.h"

#ifndef GRID_SIZE
#define	IMAGE_GRID_ROWS	10
#define	IMAGE_GRID_COLS	10
#define	IMAGE_BORDER_SIZE	10
#endif // !GRID_SIZE

class CMapPoint;
class CFrame
{
public:
	static	arma::mat	K;
	static	arma::vec	dist;
private:
	arma::mat	R;	///< point transformation from world to camera coordinates
	arma::vec	t;
	cv::Mat	m_cvColorImage, m_cvDepthImage;
	std::vector<SiftKeyPoint>	m_vSiftKeyPoints;
	std::vector<float>			m_vSiftDescriptors;

	std::vector<cv::KeyPoint>	m_vORBKeyPoints;
	cv::Mat	m_cvORBDescriptors;

	static	double	m_dbMinX, m_dbMaxX, m_dbMinY, m_dbMaxY;
	static	double	m_dbInvGridColSize;
	static	double	m_dbInvGridRowSize;

	std::map<int, CMapPoint *>	m_mMapPointConnections;

	std::vector<int>	m_vPointFeatureIdxInGrid[IMAGE_GRID_ROWS][IMAGE_GRID_COLS];
	std::vector<WorldPoint>	m_vLocalMapPoints;
	std::map<int, int>	m_mLocalMapPointConnections;
public:
	CFrame();
	CFrame(CFrame& frame);	///< copy constructor
	CFrame(const cv::Mat& cvColorImage, const cv::Mat& cvDepthImage);
	~CFrame();

	void	SetImage(const cv::Mat& cvColorImage, const cv::Mat& cvDepthImage);
	void	ExtractSiftFeatures(void);
	void	ExtractORBFeatures(void);

	std::vector<SiftKeyPoint>&	GetSiftKeyPoints() {
		return m_vSiftKeyPoints;
	};
	std::vector<float>&	GetSiftDescriptors() {
		return m_vSiftDescriptors;
	}

	void	SetR(const arma::mat& R);
	void	SetT(const arma::vec& t);

	arma::mat	GetR(void) {
		return R;
	}
	arma::vec	GetT(void) {
		return t;
	}

	std::vector<cv::KeyPoint>&	GetORBKeyPoints() {
		return m_vORBKeyPoints;
	}
	cv::Mat&	GetORBDescriptors() {
		return m_cvORBDescriptors;
	}

	///@ brief
	///@ params imgSize: 0 column for left top
	///@				 1 column for right top
	///@				 2 column for left bottom
	///@				 3 column for right bottom
	static	void	ComputeImageBound(const arma::mat& imgSize);

	CFrame& operator=(CFrame& frame);	///< assignment operator overloading
	//const CFrame& operator=(const CFrame frame);

	cv::Mat	GetCurrFrame(void);

	void	EstimateLocalScene(void);
	std::vector<WorldPoint>&	GetLocalMapPoints(void) {
		return m_vLocalMapPoints;
	}
	std::map<int, CMapPoint *>&	GetTrackedMapPoints(void) {
		return m_mMapPointConnections;
	}

	void	AddMapPointConnection(int nIdx, CMapPoint* pMapPoint);

	std::map<int, int>&	GetLocalMapPointConnections(void) {
		return m_mLocalMapPointConnections;
	}

	std::vector<int>	GetProjectionMatchCandidates(const arma::vec& x);
};
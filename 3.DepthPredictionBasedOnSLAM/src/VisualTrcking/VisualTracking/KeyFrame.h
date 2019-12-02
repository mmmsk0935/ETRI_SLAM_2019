#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <set>

#include "VisualSLAMConfigure.h"
#include "Frame.h"
#include "MapPoint.h"

class CMapPoint;
class CFrame;
class CKeyFrame
{
private:
	static	arma::mat	K;
	arma::mat	R;
	arma::vec	t;
	arma::mat	P;

	cv::Mat	m_cvColorImage;

	std::vector<SiftKeyPoint>	m_vSiftKeyPoints;
	std::vector<float>			m_vSiftDescriptors;

	std::vector<cv::KeyPoint>	m_vORBKeyPoints;
	cv::Mat						m_cvORBDescriptors;

	std::map<int, CMapPoint *>	m_mMapPointConnections;

	std::map<int, int>	m_mLocalMapPointConnections;
	std::vector<WorldPoint>	m_vLocalMapPoints;

	std::set<CKeyFrame *>	m_sAdjacentKeyFrames;
public:
	CKeyFrame(CFrame& frame);
	~CKeyFrame();

	void	AddMapPointConnection(int nPointFeatureIdx, CMapPoint* pMapPoint);

	std::vector<float>&	GetSiftDescriptors(void) {
		return m_vSiftDescriptors;
	}
	std::vector<SiftKeyPoint>&	GetSiftKeyPoints(void) {
		return m_vSiftKeyPoints;
	}

	std::map<int, CMapPoint *>&	GetMapPointConnections(void) {
		return m_mMapPointConnections;
	}

	std::map<int, int>&	GetLocalMapPointConnections(void) {
		return m_mLocalMapPointConnections;
	}

	std::vector<WorldPoint>&	GetLocalMapPoints(void) {
		return m_vLocalMapPoints;
	}

	cv::Mat	GetColorImage(void) {
		return m_cvColorImage.clone();
	}

	void	AddAdjacentKeyFrame(CKeyFrame* pAdjacentKeyFrame);

	std::set<CKeyFrame *>	GetAdjacentKeyFrames(void);

	arma::mat	GetP(void) {
		return P;
	}

	arma::mat	GetR(void) {
		return R;
	}

	arma::vec	GetT(void) {
		return t;
	}
};
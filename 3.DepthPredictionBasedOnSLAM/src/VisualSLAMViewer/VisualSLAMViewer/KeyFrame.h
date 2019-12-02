#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <set>

#include "VisualSLAMConfigure.h"
#include "Frame.h"
#include "MapPoint.h"
#include "MapLine.h"
#include "MapPlane.h"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "opencv2/line_descriptor.hpp"

class CMapPoint;
class CFrame;
class CMapLine;
class CMapPlane;

class CSLAMKeyFrame
{
public:
	static	arma::mat	K;
	static	double	m_dbMinX, m_dbMaxX, m_dbMinY, m_dbMaxY;
	static	double	m_dbInvGridColSize, m_dbInvGridRowSize;
	static	int	m_nKeyFrameCount;
private:
	int	m_nKeyFrameID;
	int	m_nFrameNumber;

	boost::mutex	m_mutexBad;
	bool	m_bIsBad;

	boost::mutex	m_mutexR;
	arma::mat	R;

	boost::mutex	m_mutexT;
	arma::vec	t;

	boost::mutex	m_mutexP;
	arma::mat	P;

	cv::Mat	m_cvColorImage;

	std::vector<SiftKeyPoint>	m_vSiftKeyPoints;
	std::vector<float>			m_vSiftDescriptors;

	std::vector<cv::KeyPoint>	m_vORBKeyPoints;
	cv::Mat						m_cvORBDescriptors;

	std::vector<cv::line_descriptor::KeyLine>	m_vKeyLines;
	cv::Mat										m_cvKeyLineDescriptors;

	boost::mutex	m_mutexMapPointConnections;
	std::map<int, CMapPoint *>	m_mMapPointConnections;

	boost::mutex	m_mutexMapLineConnections;
	std::map<int, CMapLine *>	m_mMapLineConnections;

	std::map<int, int>	m_mLocalMapPointConnections;
	std::vector<WorldPoint>	m_vLocalMapPoints;

	boost::mutex	m_mutexAdjacentKeyFrames;
	std::set<CSLAMKeyFrame *>	m_sAdjacentKeyFrames;

	boost::mutex	m_mutexMapPlanes;
	std::vector<CMapPlane*>	m_vObservedMapPlanes;

	std::vector<int>	m_vPointFeatureIdxInGrid[IMAGE_GRID_ROWS][IMAGE_GRID_COLS];

public:
	CSLAMKeyFrame(CFrame& frame);
	~CSLAMKeyFrame();

	int	GetKeyFrameID(void) {
		return m_nKeyFrameID;
	}

	void	SetBadFlag(void) {
		boost::mutex::scoped_lock lock(m_mutexBad);
		m_bIsBad = true;
	}

	bool	IsBad(void) {
		boost::mutex::scoped_lock lock(m_mutexBad);
		return m_bIsBad;
	}

	void	AddObservedMapPlane(CMapPlane* pMapPlane) {
		boost::mutex::scoped_lock lock(m_mutexMapPlanes);
		m_vObservedMapPlanes.push_back(pMapPlane);
	}

	std::vector<CMapPlane*>	GetObservedMapPlanes(void) {
		boost::mutex::scoped_lock lock(m_mutexMapPlanes);
		return m_vObservedMapPlanes;
	}

	void	AddMapPointConnection(int nPointFeatureIdx, CMapPoint* pMapPoint);

	void	AddMapLineConnection(int nLineFeatureIdx, CMapLine* pMapLine);

	std::vector<float>&	GetSiftDescriptors(void) {
		return m_vSiftDescriptors;
	}
	std::vector<SiftKeyPoint>&	GetSiftKeyPoints(void) {
		return m_vSiftKeyPoints;
	}

	std::map<int, CMapPoint *>	GetMapPointConnections(void) {
		boost::mutex::scoped_lock lock(m_mutexMapPointConnections);
		return m_mMapPointConnections;
	}

	std::map<int, CMapLine *>	GetMapLineConnections(void) {
		boost::mutex::scoped_lock lock(m_mutexMapLineConnections);
		return m_mMapLineConnections;
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

	std::vector<cv::line_descriptor::KeyLine>&	GetKeyLines(void) {
		return m_vKeyLines;
	}

	cv::Mat	GetKeyLineDescriptors(void) {
		return m_cvKeyLineDescriptors;
	}

	void	AddAdjacentKeyFrame(CSLAMKeyFrame* pAdjacentKeyFrame);

	std::set<CSLAMKeyFrame *>	GetAdjacentKeyFrames(void);

	arma::mat	GetP(void) {
		boost::mutex::scoped_lock lock(m_mutexP);
		return P;
	}

	void	SetR(const arma::mat& _R) {
		boost::mutex::scoped_lock lock(m_mutexR);
		R = _R;
	}

	void	SetT(const arma::vec& _t) {
		boost::mutex::scoped_lock lock(m_mutexT);
		t = _t;
	}

	void	SetP(const arma::mat& _P) {
		boost::mutex::scoped_lock lock(m_mutexP);
		P = _P;
	}

	arma::mat	GetR(void) {
		boost::mutex::scoped_lock lock(m_mutexR);
		return R;
	}

	arma::vec	GetT(void) {
		boost::mutex::scoped_lock lock(m_mutexT);
		return t;
	}

	std::vector<int>	GetPointFeatureIdxInPlane(double dbMinX, double dbMaxX, double dbMinY, double dbMaxY);
};
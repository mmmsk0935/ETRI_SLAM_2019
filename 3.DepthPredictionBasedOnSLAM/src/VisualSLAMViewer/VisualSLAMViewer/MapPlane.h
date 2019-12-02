#pragma once

#include <iostream>
#include <vector>

#include "armadillo"
#include "PlaneEstimator.h"

#include "KeyFrame.h"
#include "MapPoint.h"
#include "MapLine.h"

#include "mycv.h"

class CMapPoint;
class CMapLine;
class CSLAMKeyFrame;

class CMapPlane
{
private:
	boost::mutex	m_mutexParams;
	arma::vec	params;

	boost::mutex	m_mutexMapPoints;
	std::vector<CMapPoint *>	m_vMapPoints;

	boost::mutex	m_mutexMapLines;
	std::vector<CMapLine *>		m_vMapLines;

	std::vector<arma::vec> m_vConvexHull;

	cv::Scalar	m_cvColor;

	boost::mutex	m_mutexObservedKeyFrames;
	std::set<CSLAMKeyFrame*>	m_sObservedKeyFrames;

	static	cv::RNG	rng;
public:
	CMapPlane();
	CMapPlane(arma::vec& abcd);
	~CMapPlane();

	std::set<CSLAMKeyFrame*>	GetObservedKeyFrames(void) {
		boost::mutex::scoped_lock lock(m_mutexObservedKeyFrames);
		return m_sObservedKeyFrames;
	}

	void	AddObservedKeyFrame(CSLAMKeyFrame* pObservedFrame) {
		boost::mutex::scoped_lock lock(m_mutexObservedKeyFrames);
		m_sObservedKeyFrames.insert(pObservedFrame);
	}

	void	AddMapPoint(CMapPoint* pMapPoint);
	void	UpdateParameter(void);

	void	AddMapLine(CMapLine* pMapLine);

	void	SetPlaneEquation(arma::vec abcd) {
		boost::mutex::scoped_lock lock(m_mutexParams);
		params = abcd;
	}

	void	SetColor(cv::Scalar color);
	cv::Scalar	GetColor(void);

	std::vector<CMapPoint *>	GetMapPoints(void) {
		boost::mutex::scoped_lock lock(m_mutexMapPoints);
		return m_vMapPoints;
	}
	std::vector<arma::vec>&	GetConvexHull(void) {
		return m_vConvexHull;
	}
	arma::vec	GetParameters(void) {
		boost::mutex::scoped_lock lock(m_mutexParams);
		return params;
	}
};
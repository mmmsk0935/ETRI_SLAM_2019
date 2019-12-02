#pragma once

#include "armadillo"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "VisualSLAMConfigure.h"
#include "KeyFrame.h"
#include "MapPlane.h"

class CSLAMKeyFrame;
class CFrame;
class CMapPlane;

class CMapPoint
{
private:
	boost::mutex	m_mutexPos;
	arma::vec	X;

	boost::mutex	m_mutexVariance;
	double	m_dbAngleVariance;

	boost::mutex	m_mutexKeyFrames;
	std::map<CSLAMKeyFrame*, int>	m_mObservedKeyFrame;

	boost::mutex	m_mutexMapPlane;
	CMapPlane*	m_pMapPlane;
public:
	CMapPoint(WorldPoint pos);
	CMapPoint(const arma::vec& pos);
	~CMapPoint();

	void	SetPosition(const arma::vec& pos);
	
	arma::vec	GetPosition(void);
	
	std::map<CSLAMKeyFrame*, int>	GetObservedKeyFrames();

	void	AddObservedKeyFrame(CSLAMKeyFrame *pKeyFrame, int nPointFeatureIdx);

	CMapPlane*	GetMapPlane(void) {
		boost::mutex::scoped_lock lock(m_mutexMapPlane);
		return m_pMapPlane;
	}

	void	SetMapPlane(CMapPlane* pNewMapPlane) {
		boost::mutex::scoped_lock lock(m_mutexMapPlane);
		m_pMapPlane = pNewMapPlane;
	}
};


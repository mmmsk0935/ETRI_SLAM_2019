#pragma once

#include <iostream>
#include <vector>
#include <set>

#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "MapPoint.h"
#include "MapPlane.h"
#include "MapLine.h"

class CWorldMap
{
private:
	boost::mutex	m_mutexMapPoints;
	std::set<CMapPoint*>	m_sMapPoints;

	boost::mutex	m_mutexMapLines;
	std::set<CMapLine*>		m_sMapLines;

	boost::mutex	m_mutexMapPlanes;
	std::set<CMapPlane*>	m_sMapPlanes;

	boost::mutex	m_mutexKeyFrames;
	std::set<CSLAMKeyFrame*>	m_sKeyFrames;
public:
	CWorldMap();
	~CWorldMap();

	void	Clear(void);

	void	AddMapPoint(CMapPoint *pNewMapPoint);
	void	AddKeyFrame(CSLAMKeyFrame* pNewKeyFrame);
	void	AddMapLine(CMapLine* pNewMapLine);
	void	AddMapPlane(CMapPlane* pNewMapPlane);
	
	bool	IsEmptyKeyFrames(void) {
		boost::mutex::scoped_lock lock(m_mutexKeyFrames);
		return m_sKeyFrames.empty();
	}
	std::set<CMapPlane*>	GetMapPlanes(void) {
		boost::mutex::scoped_lock lock(m_mutexMapPlanes);
		return m_sMapPlanes;
	}
	std::set<CMapPoint*>	GetMapPoints(void) {
		boost::mutex::scoped_lock lock(m_mutexMapPoints);
		return m_sMapPoints;
	}
	std::set<CMapLine*>	GetMapLines(void) {
		boost::mutex::scoped_lock lock(m_mutexMapLines);
		return m_sMapLines;
	}
private:
	void	ClearMapPoints(void);
	void	ClearMapLines(void);
	void	ClearMapPlanes(void);
};


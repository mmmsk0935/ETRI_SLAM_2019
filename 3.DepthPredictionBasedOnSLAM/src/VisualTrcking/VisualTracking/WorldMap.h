#pragma once

#include "MapPoint.h"
#include "MapPlane.h"
#include "MapLine.h"

#include <iostream>
#include <vector>
#include <set>

class CWorldMap
{
private:
	std::set<CMapPoint*>	m_sMapPoints;
	std::set<CMapLine*>		m_sMapLines;
	std::set<CMapPlane*>	m_sMapPlanes;

	std::set<CKeyFrame*>	m_sKeyFrames;
public:
	CWorldMap();
	~CWorldMap();

	void	Clear(void);

	void	AddMapPoint(CMapPoint *pNewMapPoint);
	void	AddKeyFrame(CKeyFrame* pNewKeyFrame);

	bool	IsEmptyKeyFrames(void) {
		return m_sKeyFrames.empty();
	}
private:
	void	ClearMapPoints(void);
	void	ClearMapLines(void);
	void	ClearMapPlanes(void);
};


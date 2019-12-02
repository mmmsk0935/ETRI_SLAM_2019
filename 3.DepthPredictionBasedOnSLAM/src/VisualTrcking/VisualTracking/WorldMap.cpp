#include "WorldMap.h"

CWorldMap::CWorldMap()
{
}

CWorldMap::~CWorldMap()
{
}

void	CWorldMap::Clear(void)
{
	ClearMapPoints();
	ClearMapLines();
	ClearMapPlanes();
}

void	CWorldMap::ClearMapPoints(void)
{
	std::set<CMapPoint*>::iterator iter = m_sMapPoints.begin();
	std::set<CMapPoint*>::iterator end = m_sMapPoints.end();

	while (iter != end) {
		delete (*iter);
		iter++;
	}

	m_sMapPoints.clear();
}

void	CWorldMap::ClearMapPlanes(void)
{
	std::set<CMapPlane*>::iterator iter = m_sMapPlanes.begin();
	std::set<CMapPlane*>::iterator end = m_sMapPlanes.end();

	while (iter != end) {
		delete (*iter);
		iter++;
	}

	m_sMapPlanes.clear();
}

void	CWorldMap::ClearMapLines(void)
{
	std::set<CMapLine*>::iterator iter = m_sMapLines.begin();
	std::set<CMapLine*>::iterator end = m_sMapLines.end();

	while (iter != end) {
		delete (*iter);
		iter++;
	}

	m_sMapLines.clear();
}

void	CWorldMap::AddMapPoint(CMapPoint *pNewMapPoint)
{
	m_sMapPoints.insert(pNewMapPoint);
}

void	CWorldMap::AddKeyFrame(CKeyFrame* pNewKeyFrame)
{
	m_sKeyFrames.insert(pNewKeyFrame);
}
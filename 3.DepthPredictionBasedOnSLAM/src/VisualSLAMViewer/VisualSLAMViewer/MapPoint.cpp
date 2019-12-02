#include "stdafx.h"
#include "MapPoint.h"

CMapPoint::CMapPoint(WorldPoint pos)
	: m_pMapPlane(nullptr)
{
	boost::mutex::scoped_lock lock(m_mutexPos);
	X.set_size(3);
	X.at(0) = pos.x;
	X.at(1) = pos.y;
	X.at(2) = pos.z;
}

CMapPoint::CMapPoint(const arma::vec& pos)
	: m_pMapPlane(nullptr)
{
	boost::mutex::scoped_lock lock(m_mutexPos);
	X.set_size(3);
	X.at(0) = pos.at(0);
	X.at(1) = pos.at(1);
	X.at(2) = pos.at(2);
}

CMapPoint::~CMapPoint()
{
}

void	CMapPoint::SetPosition(const arma::vec& pos)
{
	boost::mutex::scoped_lock lock(m_mutexPos);
	X.set_size(3);
	X.at(0) = pos.at(0);
	X.at(1) = pos.at(1);
	X.at(2) = pos.at(2);
}

arma::vec	CMapPoint::GetPosition(void)
{
	boost::mutex::scoped_lock lock(m_mutexPos);
	return X;
}

std::map<CSLAMKeyFrame*, int>	CMapPoint::GetObservedKeyFrames()
{
	boost::mutex::scoped_lock lock(m_mutexKeyFrames);
	return m_mObservedKeyFrame;
}

void	CMapPoint::AddObservedKeyFrame(CSLAMKeyFrame *pKeyFrame, int nPointFeatureIdx)
{
	boost::mutex::scoped_lock lock(m_mutexKeyFrames);
	if (m_mObservedKeyFrame.find(pKeyFrame) == m_mObservedKeyFrame.end())
		m_mObservedKeyFrame[pKeyFrame] = nPointFeatureIdx;
}
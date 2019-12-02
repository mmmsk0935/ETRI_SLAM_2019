#include "stdafx.h"
#include "MapLine.h"

CMapLine::CMapLine()
{
}

CMapLine::CMapLine(arma::vec& pt1, arma::vec& pt2, arma::vec& d)
	: x1(pt1), x2(pt2), dir(d)
{

}

CMapLine::~CMapLine()
{
}

void	CMapLine::GetPosition(arma::vec& pt1, arma::vec& pt2)
{
	boost::mutex::scoped_lock lock(m_mutexPos);
	pt1.at(0) = x1.at(0);
	pt1.at(1) = x1.at(1);
	pt1.at(2) = x1.at(2);

	pt2.at(0) = x2.at(0);
	pt2.at(1) = x2.at(1);
	pt2.at(2) = x2.at(2);
}

std::map<CSLAMKeyFrame*, int>	CMapLine::GetObservedKeyFrames()
{
	boost::mutex::scoped_lock lock(m_mutexKeyFrames);
	return m_mObservedKeyFrame;
}

void	CMapLine::AddObservedKeyFrame(CSLAMKeyFrame *pKeyFrame, int nLineFeatureIdx)
{
	boost::mutex::scoped_lock lock(m_mutexKeyFrames);
	m_mObservedKeyFrame[pKeyFrame] = nLineFeatureIdx;
}
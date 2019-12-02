#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <set>

#include "armadillo"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "KeyFrame.h"

class CSLAMKeyFrame;
class CMapLine
{
private:
	boost::mutex	m_mutexPos;
	arma::vec	x1, x2, dir;

	boost::mutex	m_mutexKeyFrames;
	std::map<CSLAMKeyFrame*, int>	m_mObservedKeyFrame;
public:
	CMapLine();
	CMapLine(arma::vec& pt1, arma::vec& pt2, arma::vec& d);
	~CMapLine();

	void	GetPosition(arma::vec& pt1, arma::vec& pt2);

	std::map<CSLAMKeyFrame*, int>	GetObservedKeyFrames();

	void	AddObservedKeyFrame(CSLAMKeyFrame *pKeyFrame, int nLineFeatureIdx);
};


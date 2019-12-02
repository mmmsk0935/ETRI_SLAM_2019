#include "KeyFrame.h"

arma::mat	CKeyFrame::K = arma::mat(3, 3);

CKeyFrame::CKeyFrame(CFrame& frame)
{
	std::vector<SiftKeyPoint>& vSiftKeyPoints = frame.GetSiftKeyPoints();
	std::vector<float>& vSiftDescriptors = frame.GetSiftDescriptors();
	std::vector<cv::KeyPoint>& vORBKeyPoints = frame.GetORBKeyPoints();
	cv::Mat& cvORBDescriptors = frame.GetORBDescriptors();
	std::vector<WorldPoint>& vLocalMapPoints = frame.GetLocalMapPoints();
	std::map<int, int>& mLocalMapPointConnections = frame.GetLocalMapPointConnections();
	std::map<int, CMapPoint*>& mMapPointConnections = frame.GetTrackedMapPoints();

	if (!vSiftKeyPoints.empty()) {
		m_vSiftKeyPoints.resize(vSiftKeyPoints.size());
		std::copy(vSiftKeyPoints.begin(), vSiftKeyPoints.end(), m_vSiftKeyPoints.begin());
	}

	if (!vSiftDescriptors.empty()) {
		m_vSiftDescriptors.resize(vSiftDescriptors.size());
		std::copy(vSiftDescriptors.begin(), vSiftDescriptors.end(), m_vSiftDescriptors.begin());
	}

	if (!vORBKeyPoints.empty()) {
		m_vORBKeyPoints.resize(vORBKeyPoints.size());
		std::copy(vORBKeyPoints.begin(), vORBKeyPoints.end(), m_vORBKeyPoints.begin());
	}

	if (!cvORBDescriptors.empty())
		m_cvORBDescriptors = cvORBDescriptors.clone();

	if (!vLocalMapPoints.empty()) {
		m_vLocalMapPoints.resize(vLocalMapPoints.size());
		std::copy(vLocalMapPoints.begin(), vLocalMapPoints.end(), m_vLocalMapPoints.begin());
	}

	if (!mLocalMapPointConnections.empty()) {
		m_mLocalMapPointConnections.clear();
		std::map<int, int>::iterator iter = mLocalMapPointConnections.begin();
		std::map<int, int>::iterator end = mLocalMapPointConnections.end();
		for (; iter != end; iter++)
			m_mLocalMapPointConnections[iter->first] = iter->second;
	}

	if (!mMapPointConnections.empty()) {
		m_mMapPointConnections.clear();
		std::map<int, CMapPoint *>::iterator iter = mMapPointConnections.begin();
		std::map<int, CMapPoint *>::iterator end = mMapPointConnections.end();
		for (; iter != end; iter++)
			m_mMapPointConnections[iter->first] = iter->second;
	}

	R = frame.GetR();
	t = frame.GetT();
	P.set_size(3, 4);

	P(arma::span(0, 2), arma::span(0, 2)) = K * R;
	P(arma::span(0, 2), arma::span(3, 3)) = K * t;
}

CKeyFrame::~CKeyFrame()
{
}

void	CKeyFrame::AddMapPointConnection(int nPointFeatureIdx, CMapPoint* pMapPoint)
{
	if (m_mMapPointConnections.find(nPointFeatureIdx) == m_mMapPointConnections.end())
		m_mMapPointConnections[nPointFeatureIdx] = pMapPoint;
}

void	CKeyFrame::AddAdjacentKeyFrame(CKeyFrame* pAdjacentKeyFrame)
{
	m_sAdjacentKeyFrames.insert(pAdjacentKeyFrame);
}

std::set<CKeyFrame *>	CKeyFrame::GetAdjacentKeyFrames(void)
{
	return m_sAdjacentKeyFrames;
}
#include "stdafx.h"
#include "KeyFrame.h"

arma::mat	CSLAMKeyFrame::K = arma::mat(3, 3);
double		CSLAMKeyFrame::m_dbMinX = 0;
double		CSLAMKeyFrame::m_dbMaxX = 0;
double		CSLAMKeyFrame::m_dbMinY = 0;
double		CSLAMKeyFrame::m_dbMaxY = 0;
double		CSLAMKeyFrame::m_dbInvGridColSize = 0;
double		CSLAMKeyFrame::m_dbInvGridRowSize = 0;
int			CSLAMKeyFrame::m_nKeyFrameCount = 0;

CSLAMKeyFrame::CSLAMKeyFrame(CFrame& frame)
	: m_nKeyFrameID(0)
	, m_bIsBad(false)
{
	std::vector<SiftKeyPoint>& vSiftKeyPoints = frame.GetSiftKeyPoints();
	std::vector<float>& vSiftDescriptors = frame.GetSiftDescriptors();
	std::vector<cv::KeyPoint>& vORBKeyPoints = frame.GetORBKeyPoints();
	cv::Mat& cvORBDescriptors = frame.GetORBDescriptors();
	std::vector<WorldPoint>& vLocalMapPoints = frame.GetLocalMapPoints();
	std::map<int, int>& mLocalMapPointConnections = frame.GetLocalMapPointConnections();
	std::map<int, CMapPoint*>& mMapPointConnections = frame.GetTrackedMapPoints();
	std::vector<cv::line_descriptor::KeyLine>& vKeyLines = frame.GetKeyLines();
	cv::Mat cvKeyLineDescriptors = frame.GetKeyLineDescriptors();

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

	if (!vKeyLines.empty()) {
		m_vKeyLines.resize(vKeyLines.size());
		std::copy(vKeyLines.begin(), vKeyLines.end(), m_vKeyLines.begin());
	}
	if (!cvKeyLineDescriptors.empty())
		m_cvKeyLineDescriptors = cvKeyLineDescriptors.clone();

	if (!frame.GetCurrFrame().empty()) {
		m_cvColorImage = frame.GetCurrFrame();
	}

	for (int i = 0; i < IMAGE_GRID_ROWS; i++) {
		for (int j = 0; j < IMAGE_GRID_COLS; j++) {
			m_vPointFeatureIdxInGrid[i][j] = frame.GetPointFeatureIdxInGrid(i, j);
		}
	}

	m_vObservedMapPlanes.clear();
	std::vector<CMapPlane*> vObservedMapPlanes = frame.GetObservedMapPlanes();
	for (int i = 0; i < (int)vObservedMapPlanes.size(); i++)
		m_vObservedMapPlanes.push_back(vObservedMapPlanes[i]);
	
	std::map<int, CMapLine*> mMapLineConnections = frame.GetTrackedMapLines();
	if (!mMapLineConnections.empty()) {
		std::map<int, CMapLine*>::iterator iter = mMapLineConnections.begin();
		std::map<int, CMapLine*>::iterator end = mMapLineConnections.end();
		for (; iter != end; iter++)
			m_mMapLineConnections[iter->first] = iter->second;
	}

	R = frame.GetR();
	t = frame.GetT();
	P.set_size(3, 4);

	P(arma::span(0, 2), arma::span(0, 2)) = K * R;
	P(arma::span(0, 2), arma::span(3, 3)) = K * t;

	m_nFrameNumber = frame.GetFrameNumber();
	m_nKeyFrameID = m_nKeyFrameCount++;
}

CSLAMKeyFrame::~CSLAMKeyFrame()
{
}

void	CSLAMKeyFrame::AddMapPointConnection(int nPointFeatureIdx, CMapPoint* pMapPoint)
{
	boost::mutex::scoped_lock lock(m_mutexMapPointConnections);
	if (m_mMapPointConnections.find(nPointFeatureIdx) == m_mMapPointConnections.end())
		m_mMapPointConnections[nPointFeatureIdx] = pMapPoint;
}

void	CSLAMKeyFrame::AddMapLineConnection(int nLineFeatureIdx, CMapLine* pMapLine)
{
	boost::mutex::scoped_lock lock(m_mutexMapLineConnections);
	if (m_mMapLineConnections.find(nLineFeatureIdx) == m_mMapLineConnections.end())
		m_mMapLineConnections[nLineFeatureIdx] = pMapLine;
}

void	CSLAMKeyFrame::AddAdjacentKeyFrame(CSLAMKeyFrame* pAdjacentKeyFrame)
{
	boost::mutex::scoped_lock lock(m_mutexAdjacentKeyFrames);
	m_sAdjacentKeyFrames.insert(pAdjacentKeyFrame);
}

std::set<CSLAMKeyFrame *>	CSLAMKeyFrame::GetAdjacentKeyFrames(void)
{
	boost::mutex::scoped_lock lock(m_mutexAdjacentKeyFrames);
	return m_sAdjacentKeyFrames;
}

std::vector<int>	CSLAMKeyFrame::GetPointFeatureIdxInPlane(double dbMinX, double dbMaxX, double dbMinY, double dbMaxY)
{
	std::vector<int> vPointFeatureIdx;

	int nStartRowIdx, nEndRowIdx;
	int nStartColIdx, nEndColIdx;

	nStartColIdx = (int)(dbMinX * m_dbInvGridColSize);
	nEndColIdx   = (int)(dbMaxX * m_dbInvGridColSize);
	nStartRowIdx = (int)(dbMinY * m_dbInvGridRowSize);
	nEndRowIdx   = (int)(dbMaxY * m_dbInvGridRowSize);

	if (nStartColIdx < 0)	nStartColIdx = 0;
	if (nEndColIdx >= IMAGE_GRID_COLS) nEndColIdx = IMAGE_GRID_COLS - 1;
	if (nStartRowIdx < 0)	nStartRowIdx = 0;
	if (nEndRowIdx >= IMAGE_GRID_ROWS) nEndRowIdx = IMAGE_GRID_ROWS - 1;

	for (int i = nStartRowIdx; i <= nEndRowIdx; i++) {
		for (int j = nStartColIdx; j <= nEndColIdx; j++) {
			for (int k = 0; k < (int)m_vPointFeatureIdxInGrid[i][j].size(); k++) {
				vPointFeatureIdx.push_back(m_vPointFeatureIdxInGrid[i][j][k]);
			}
		}
	}

	return vPointFeatureIdx;
}
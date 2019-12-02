#include "stdafx.h"
#include "Frame.h"

arma::mat	CFrame::K = arma::mat(3, 3);
arma::vec	CFrame::dist = arma::vec(5);
double	CFrame::m_dbMinX, CFrame::m_dbMaxX, CFrame::m_dbMinY, CFrame::m_dbMaxY;
double	CFrame::m_dbInvGridRowSize, CFrame::m_dbInvGridColSize;

CFrame::CFrame()
{
	R.eye(3, 3);
	t.zeros(3);
}

CFrame::CFrame(CFrame& frame)
	: m_cvColorImage(frame.m_cvColorImage.clone()), m_cvDepthImage(frame.m_cvDepthImage.clone())
	, m_vSiftKeyPoints(frame.m_vSiftKeyPoints), m_vSiftDescriptors(frame.m_vSiftDescriptors)
	, m_vORBKeyPoints(frame.m_vORBKeyPoints), m_cvORBDescriptors(frame.m_cvORBDescriptors.clone())
	, R(frame.R), t(frame.t)
	, m_mMapPointConnections(frame.m_mMapPointConnections), m_vLocalMapPoints(frame.m_vLocalMapPoints)
	, m_mLocalMapPointConnections(frame.m_mLocalMapPointConnections)
	, m_vKeyLines(frame.m_vKeyLines), m_cvKeyLineDescriptors(frame.m_cvKeyLineDescriptors.clone())
	, m_nFrameNumber(frame.m_nFrameNumber), m_vObservedMapPlanes(frame.m_vObservedMapPlanes)
	, m_mMapLineConnections(frame.m_mMapLineConnections)
{

	for (int i = 0; i < IMAGE_GRID_ROWS; i++) {
		for (int j = 0; j < IMAGE_GRID_COLS; j++) {
			m_vPointFeatureIdxInGrid[i][j].resize(frame.m_vPointFeatureIdxInGrid[i][j].size());
			std::copy(frame.m_vPointFeatureIdxInGrid[i][j].begin(), frame.m_vPointFeatureIdxInGrid[i][j].end(), m_vPointFeatureIdxInGrid[i][j].begin());
		}
	}
}

CFrame::CFrame(const cv::Mat& cvColorImage, const cv::Mat& cvDepthImage)
{
	m_cvColorImage = cvColorImage.clone();
	m_cvDepthImage = cvDepthImage.clone();

	R.eye(3, 3);
	t.zeros(3);

	m_vLocalMapPoints.clear();
	m_mLocalMapPointConnections.clear();
	m_mMapPointConnections.clear();
}

CFrame::~CFrame()
{
}

void	CFrame::SetImage(int nFrameNumber, const cv::Mat& cvColorImage, const cv::Mat& cvDepthImage)
{
	m_nFrameNumber = nFrameNumber;
	m_cvColorImage = cvColorImage.clone();

	if (!m_cvDepthImage.empty())
		m_cvDepthImage = cvDepthImage.clone();
}

void	CFrame::ExtractSiftFeatures(void)
{
	cv::Mat cvGrayImage;
	
	cv::cvtColor(m_cvColorImage, cvGrayImage, cv::COLOR_BGR2GRAY);

	CSiftGPUWrapper::ExtractSiftFeature((char *)cvGrayImage.data, cvGrayImage.cols, cvGrayImage.rows, m_vSiftKeyPoints, m_vSiftDescriptors);

	///< we need to undistortion
	double cx, cy, fx, fy;
	double k1, k2, k3, p1, p2;
	cx = K.at(0, 2);
	cy = K.at(1, 2);
	fx = K.at(0, 0);
	fy = K.at(1, 1);
	k1 = dist.at(0);
	k2 = dist.at(1);
	p1 = dist.at(2);
	p2 = dist.at(3);
	k3 = dist.at(4);
	
	Concurrency::parallel_for(0, (int)m_vSiftKeyPoints.size(), [&](int i) {
		SiftKeyPoint& key = m_vSiftKeyPoints[i];
		double x1 = (key.x - cx) / fx;
		double y1 = (key.y - cy) / fy;

		double r = sqrt(x1*x1 + y1 * y1);
		double r2 = r * r;
		double r4 = r2 * r2;
		double r6 = r2 * r4;

		double x2 = x1 * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1*x1*y1 + p2 * (r2 + 2 * x1);
		double y2 = y1 * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y1*y1) + 2 * p2*x1*y1;

		key.x = (float)(fx*x2 + cx);
		key.y = (float)(fy*y2 + cy);
	});
	/*for (int i = 0; i < (int)m_vSiftKeyPoints.size(); i++) {
		SiftKeyPoint& key = m_vSiftKeyPoints[i];
		double x1 = (key.x - cx) / fx;
		double y1 = (key.y - cy) / fy;

		double r = sqrt(x1*x1 + y1 * y1);
		double r2 = r * r;
		double r4 = r2 * r2;
		double r6 = r2 * r4;

		double x2 = x1 * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1*x1*y1 + p2 * (r2 + 2 * x1);
		double y2 = y1 * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y1*y1) + 2 * p2*x1*y1;

		key.x = (float)(fx*x2 + cx);
		key.y = (float)(fy*y2 + cy);
	}*/

	for (int i = 0; i < (int)m_vSiftKeyPoints.size(); i++) {
		const SiftKeyPoint& key = m_vSiftKeyPoints[i];

		float x = key.x;
		float y = key.y;

		int nRowIdx = cvRound(y * m_dbInvGridRowSize);
		int nColIdx = cvRound(x * m_dbInvGridColSize);

		if (nRowIdx >= 0 && nRowIdx < IMAGE_GRID_ROWS && nColIdx >= 0 && nColIdx < IMAGE_GRID_COLS)
			m_vPointFeatureIdxInGrid[nRowIdx][nColIdx].push_back(i);
	}
}

void	CFrame::ExtractLineFeatures(void)
{
	m_lsdWrapper.ExtractLineSegment(m_cvColorImage, m_vKeyLines, m_cvKeyLineDescriptors);
}

void	CFrame::SetR(const arma::mat& R) {
	this->R = R;
}

void	CFrame::SetT(const arma::vec& t) {
	this->t = t;
}

CFrame& CFrame::operator=(CFrame& frame)
{
	m_cvColorImage = frame.m_cvColorImage.clone();
	m_cvDepthImage = frame.m_cvDepthImage.clone();

	if (!frame.m_vSiftKeyPoints.empty()) {
		m_vSiftKeyPoints.resize(frame.m_vSiftKeyPoints.size());
		std::copy(frame.m_vSiftKeyPoints.begin(), frame.m_vSiftKeyPoints.end(), m_vSiftKeyPoints.begin());
	}

	if (!frame.m_vSiftDescriptors.empty()) {
		m_vSiftDescriptors.resize(frame.m_vSiftDescriptors.size());
		std::copy(frame.m_vSiftDescriptors.begin(), frame.m_vSiftDescriptors.end(), m_vSiftDescriptors.begin());
	}

	if (!frame.m_vORBKeyPoints.empty()) {
		m_vORBKeyPoints.resize(frame.m_vORBKeyPoints.size());
		std::copy(frame.m_vORBKeyPoints.begin(), frame.m_vORBKeyPoints.end(), m_vORBKeyPoints.begin());
	}
	m_cvORBDescriptors = frame.m_cvORBDescriptors.clone();

	for (int i = 0; i < IMAGE_GRID_ROWS; i++) {
		for (int j = 0; j < IMAGE_GRID_COLS; j++) {
			m_vPointFeatureIdxInGrid[i][j].resize(frame.m_vPointFeatureIdxInGrid[i][j].size());
			std::copy(frame.m_vPointFeatureIdxInGrid[i][j].begin(), frame.m_vPointFeatureIdxInGrid[i][j].end(), m_vPointFeatureIdxInGrid[i][j].begin());
		}
	}
	
	if (!frame.m_vLocalMapPoints.empty()) {
		m_vLocalMapPoints.resize(frame.m_vLocalMapPoints.size());
		std::copy(frame.m_vLocalMapPoints.begin(), frame.m_vLocalMapPoints.end(), m_vLocalMapPoints.begin());
	}
	
	m_mMapPointConnections.clear();
	if (!frame.m_mMapPointConnections.empty()) {
		std::map<int, CMapPoint *>::iterator iter = frame.m_mMapPointConnections.begin();
		std::map<int, CMapPoint *>::iterator end = frame.m_mMapPointConnections.end();
		
		while (iter != end) {
			m_mMapPointConnections[iter->first] = iter->second;
			iter++;
		}
	}
	
	m_mLocalMapPointConnections.clear();
	if (!frame.m_mLocalMapPointConnections.empty()) {
		std::map<int, int>::iterator iter = frame.m_mLocalMapPointConnections.begin();
		std::map<int, int>::iterator end = frame.m_mLocalMapPointConnections.end();

		while (iter != end) {
			m_mLocalMapPointConnections[iter->first] = iter->second;
			iter++;
		}
	}

	R = frame.R;
	t = frame.t;

	if (!frame.m_vKeyLines.empty()) {
		m_vKeyLines.resize(frame.m_vKeyLines.size());
		std::copy(frame.m_vKeyLines.begin(), frame.m_vKeyLines.end(), m_vKeyLines.begin());
	}

	m_cvKeyLineDescriptors = frame.m_cvKeyLineDescriptors.clone();

	//std::cout << "assignment operator" << std::endl;

	return (*this);
}

void	CFrame::ComputeImageBound(const arma::mat& imgSize)
{
	///< undistort each initial image bound points
	double cx, cy, fx, fy;
	double k1, k2, k3, p1, p2;
	cx = K.at(0, 2);
	cy = K.at(1, 2);
	fx = K.at(0, 0);
	fy = K.at(1, 1);
	k1 = dist.at(0);
	k2 = dist.at(1);
	p1 = dist.at(2);
	p2 = dist.at(3);
	k3 = dist.at(4);

	m_dbMinX = INT_MAX;
	m_dbMaxX = 0;
	m_dbMinY = INT_MAX;
	m_dbMaxY = 0;
	for (int i = 0; i < imgSize.n_cols; i++) {
		double x1 = (imgSize.at(0, i) - cx) / fx;
		double y1 = (imgSize.at(1, i) - cy) / fy;

		double r = sqrt(x1*x1 + y1 * y1);
		double r2 = r * r;
		double r4 = r2 * r2;
		double r6 = r2 * r4;

		double x2 = x1 * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1*x1*y1 + p2 * (r2 + 2 * x1);
		double y2 = y1 * (1 + k1 * r2 + k2 * r4 + k3 * r6) + p1 * (r2 + 2 * y1*y1) + 2 * p2*x1*y1;

		x2 = (double)(fx*x2 + cx);
		y2 = (double)(fy*y2 + cy);

		m_dbMinX = m_dbMinX > x2 ? x2 : m_dbMinX;
		m_dbMaxX = m_dbMaxX < x2 ? x2 : m_dbMaxX;
		m_dbMinY = m_dbMinY > y2 ? y2 : m_dbMinY;
		m_dbMaxY = m_dbMaxY < y2 ? y2 : m_dbMaxY;
	}
	m_dbInvGridColSize = IMAGE_GRID_COLS / (m_dbMaxX - m_dbMinX);
	m_dbInvGridRowSize = IMAGE_GRID_ROWS / (m_dbMaxY - m_dbMinY);
}

cv::Mat	CFrame::GetCurrFrame(void)
{
	return m_cvColorImage.clone();
}

void	CFrame::EstimateLocalScene(void)
{
	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	double fx, fy, cx, cy;
	fx = K.at(0, 0); fy = K.at(1, 1); cx = K.at(0, 2); cy = K.at(1, 2);
	
	m_vLocalMapPoints.clear();
	m_mLocalMapPointConnections.clear();
	if (vSLAMConfigure.m_nPointFeatureType == PointFeatureType::SIFT_GPU) {
		for (int i = 0; i < (int)m_vSiftKeyPoints.size(); i++) {
			const SiftKeyPoint& key = m_vSiftKeyPoints[i];

			/*int nX = cvRound(key.x);
			int nY = cvRound(key.y);

			if (nX < 10 || nX >= m_cvColorImage.cols - 10 || nY < 10 || nY >= m_cvColorImage.rows - 10)
				continue;
			double dbDepth = m_cvDepthImage.at<float>(nY, nX) / 10;*/
			int nX1 = (int)key.x;	int nX2 = nX1+1;
			int nY1 = (int)key.y;	int nY2 = nY1+1;

			if (nX1 < 10 || nX2 >= m_cvColorImage.cols - 10 || nY1 < 10 || nY2 >= m_cvColorImage.rows - 10)
				continue;
			double dbDepth1 = m_cvDepthImage.at<float>(nY1, nX1) / 10;
			double dbDepth2 = m_cvDepthImage.at<float>(nY1, nX2) / 10;
			double dbDepth3 = m_cvDepthImage.at<float>(nY2, nX1) / 10;
			double dbDepth4 = m_cvDepthImage.at<float>(nY2, nX2) / 10;

			double dbXRatio = key.x - nX1;
			double dbYRatio = key.y - nY1;

			double dbDepth = (1 - dbYRatio)*((1 - dbXRatio)*dbDepth1 + dbXRatio * dbDepth2) + 
								  dbYRatio *((1 - dbXRatio)*dbDepth3 + dbXRatio * dbDepth4);

			if (dbDepth != 0) {
				WorldPoint pt3D;
				pt3D.z = dbDepth;
				pt3D.x = (key.x - cx) * dbDepth / fx;
				pt3D.y = (key.y - cy) * dbDepth / fy;

				m_vLocalMapPoints.push_back(pt3D);
				m_mLocalMapPointConnections[i] = (int)m_vLocalMapPoints.size()-1;
			}
		}
	}
}

void	CFrame::AddMapPointConnection(int nIdx, CMapPoint* pMapPoint)
{
	if (m_mMapPointConnections.find(nIdx) == m_mMapPointConnections.end())
		m_mMapPointConnections[nIdx] = pMapPoint;
	/*if (!m_mMapPointConnections[nIdx])
		m_mMapPointConnections[nIdx] = pMapPoint;*/
}

std::vector<int>	CFrame::GetProjectionMatchCandidates(const arma::vec& x)
{
	std::vector<int> vCandidates;

	double dbMinX = x.at(0) - 50;
	double dbMaxX = x.at(0) + 50;
	double dbMinY = x.at(1) - 50;
	double dbMaxY = x.at(1) + 50;

	int nStartRow, nEndRow, nStartCol, nEndCol;

	nStartCol = (int)(dbMinX * m_dbInvGridColSize);
	nEndCol   = (int)(dbMaxX * m_dbInvGridColSize);
	nStartRow = (int)(dbMinY * m_dbInvGridRowSize);
	nEndRow   = (int)(dbMaxY * m_dbInvGridRowSize);

	if (nStartCol < 0)	nStartCol = 0;
	if (nEndCol >= IMAGE_GRID_COLS) nEndCol = IMAGE_GRID_COLS - 1;
	if (nStartRow < 0)	nStartRow = 0;
	if (nEndRow >= IMAGE_GRID_ROWS) nEndRow = IMAGE_GRID_ROWS - 1;

	for (int i = nStartRow; i <= nEndRow; i++) {
		for (int j = nStartCol; j <= nEndCol; j++) {
			for (int k = 0; k < (int)m_vPointFeatureIdxInGrid[i][j].size(); k++)
				vCandidates.push_back(m_vPointFeatureIdxInGrid[i][j][k]);
		}
	}

	return vCandidates;
}

//const CFrame& coperator=(const CFrame frame)
//{
//	std::cout << "bbb" << std::endl;
//
//	return (*this);
//}
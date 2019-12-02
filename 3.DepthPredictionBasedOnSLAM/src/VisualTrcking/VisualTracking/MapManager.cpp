#include "MapManager.h"

CMapManager::CMapManager(CWorldMap* pWorldMap)
	: m_pWorldMap(pWorldMap)
{
}

CMapManager::~CMapManager()
{
}

void	CMapManager::Clear(void)
{

}

void	CMapManager::AddMapPoint(CMapPoint* pNewMapPoint)
{
	m_pWorldMap->AddMapPoint(pNewMapPoint);
}

void	CMapManager::AddKeyFrame(CKeyFrame* pNewKeyFrame)
{
	if (m_pWorldMap->IsEmptyKeyFrames())
		m_pWorldMap->AddKeyFrame(pNewKeyFrame);
	else
		m_qKeyFrames.push(pNewKeyFrame);
	//m_pWorldMap->AddKeyFrame(pNewKeyFrame);
}

void	CMapManager::ProcessNewKeyFrame(void)
{
	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();

	m_pCurrKeyFrame = m_qKeyFrames.front();
	m_qKeyFrames.pop();

	std::map<int, CMapPoint *> mMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
	std::map<int, CMapPoint *>::iterator iter1 = mMapPointConnections.begin();
	std::map<int, CMapPoint *>::iterator end1 = mMapPointConnections.end();

	std::map<CKeyFrame *, int> mCovisibilities;
	for (; iter1 != end1; iter1++) {
		CMapPoint *pMapPoint = iter1->second;
		std::map<CKeyFrame*, int> mObservedFrames = pMapPoint->GetObservedKeyFrames();
		
		std::map<CKeyFrame*, int>::iterator iter2 = mObservedFrames.begin();
		std::map<CKeyFrame*, int>::iterator end2 = mObservedFrames.end();
		for (; iter2 != end2; iter2++)
			mCovisibilities[iter2->first]++;
	}

	std::map<CKeyFrame *, int>::iterator iter2 = mCovisibilities.begin();
	std::map<CKeyFrame *, int>::iterator end2 = mCovisibilities.end();
	std::map<int, CKeyFrame *> mCovisibilityWeights;
	for (; iter2 != end2; iter2++)
		mCovisibilityWeights[iter2->second] = iter2->first;

	std::map<int, CKeyFrame *>::iterator iter3 = mCovisibilityWeights.begin();
	std::map<int, CKeyFrame *>::iterator end3 = mCovisibilityWeights.end();
	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();
	std::vector<float>& vSrcDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();
	std::map<int, CMapPoint *>& mSrcMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
	arma::mat P1 = m_pCurrKeyFrame->GetP();
	cv::Mat cvSrcColorImage = m_pCurrKeyFrame->GetColorImage();

	std::vector<std::pair<int, int>> vCoarseMatchPair;
	for (; iter3 != end3; iter3++) {
		if (iter3->first >= 50) { /// covisibility initial weight
			CKeyFrame* pPrevKeyFrame = iter3->second;

			if (pPrevKeyFrame == m_pCurrKeyFrame)
				continue;
			std::vector<SiftKeyPoint>& vDstKeyPoints = pPrevKeyFrame->GetSiftKeyPoints();
			std::vector<float>& vDstDescriptors = pPrevKeyFrame->GetSiftDescriptors();
			std::map<int, CMapPoint *>& mDstMapPointConnections = pPrevKeyFrame->GetMapPointConnections();
			arma::mat P2 = pPrevKeyFrame->GetP();
			cv::Mat cvDstColorImage = pPrevKeyFrame->GetColorImage();

			CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vCoarseMatchPair);

			if (vCoarseMatchPair.size() < 50)
				continue;

			std::vector<int> vFInlierIndices = FindEpipolarGeometricMatchFromTwoView(vSrcKeyPoints, vDstKeyPoints, vCoarseMatchPair);

			for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
				int nSrcKeyPointIdx = vCoarseMatchPair[vFInlierIndices[i]].first;
				int nDstKeyPointIdx = vCoarseMatchPair[vFInlierIndices[i]].second;

				std::map<int, CMapPoint *>::iterator iter5 = mSrcMapPointConnections.find(nSrcKeyPointIdx);
				std::map<int, CMapPoint *>::iterator iter6 = mDstMapPointConnections.find(nDstKeyPointIdx);

				if (iter5 == mSrcMapPointConnections.end() && iter6 == mDstMapPointConnections.end()) {
					float x1 = vSrcKeyPoints[nSrcKeyPointIdx].x;
					float y1 = vSrcKeyPoints[nSrcKeyPointIdx].y;
					float x2 = vDstKeyPoints[nDstKeyPointIdx].x;
					float y2 = vDstKeyPoints[nDstKeyPointIdx].y;

					// if a key point is in boder size, drop it
					if (x1 <= BORDER_SIZE || x1 >= cvSrcColorImage.cols - BORDER_SIZE || y1 < BORDER_SIZE || y1 >= cvSrcColorImage.rows - BORDER_SIZE)
						continue;
					if (x2 <= BORDER_SIZE || x2 >= cvDstColorImage.cols - BORDER_SIZE || y2 < BORDER_SIZE || y2 >= cvDstColorImage.rows - BORDER_SIZE)
						continue;
					/// need triangulate
					arma::mat A(4, 4), U, V;
					arma::vec x, s;

					A.row(0) = vSrcKeyPoints[nSrcKeyPointIdx].x * P1.row(2) - P1.row(0);
					A.row(1) = vSrcKeyPoints[nSrcKeyPointIdx].y * P1.row(2) - P1.row(1);
					A.row(2) = vDstKeyPoints[nDstKeyPointIdx].x * P2.row(2) - P2.row(0);
					A.row(3) = vDstKeyPoints[nDstKeyPointIdx].y * P2.row(2) - P2.row(1);

					arma::svd(U, s, V, A);

					x = V.col(3);

					x /= x(3);

					CMapPoint* pNewMapPoint = new CMapPoint(x);
					pNewMapPoint->AddObservedKeyFrame(pPrevKeyFrame, nSrcKeyPointIdx);
					pNewMapPoint->AddObservedKeyFrame(m_pCurrKeyFrame, nDstKeyPointIdx);
					mSrcMapPointConnections[nSrcKeyPointIdx] = pNewMapPoint;
					mDstMapPointConnections[nDstKeyPointIdx] = pNewMapPoint;

					m_pWorldMap->AddMapPoint(pNewMapPoint);
				}
				else if (iter5 == mSrcMapPointConnections.end() && iter6 != mDstMapPointConnections.end()) {
					mSrcMapPointConnections[nSrcKeyPointIdx] = iter6->second;
				}
				else if (iter5 == mSrcMapPointConnections.end() && iter6 == mDstMapPointConnections.end()) {
					mDstMapPointConnections[nDstKeyPointIdx] = iter5->second;
				}
			}
		}
	}

	m_pWorldMap->AddKeyFrame(m_pCurrKeyFrame);

	/*CKeyFrame* pLastKeyFrame = m_pWorldMap->GetLastKeyFrame();

	if (vSLAMConfigure.m_nPointFeatureType == PointFeatureType::SIFT_GPU) {
		std::vector<SiftKeyPoint>& vPrevKeyPoint = pLastKeyFrame->GetSiftKeyPoints();
		std::vector<SiftKeyPoint>& vCurrKeyPoint = pNewKeyFrame->GetSiftKeyPoints();
		std::vector<float>& vPrevDescriptors = pLastKeyFrame->GetSiftDescriptors();
		std::vector<float>& vCurrDescriptors = pNewKeyFrame->GetSiftDescriptors();
		std::vector<std::pair<int, int>> vCoarseMatchPair;

		CSiftGPUWrapper::FindCoarseMatch(vPrevDescriptors, vCurrDescriptors, vCoarseMatchPair);
	}*/

	if (vSLAMConfigure.m_nCameraType == CameraType::RGBD) {
		std::vector<WorldPoint>& vLocalMapPoints = m_pCurrKeyFrame->GetLocalMapPoints();
		std::map<int, int>& mLocalMapPointConnections = m_pCurrKeyFrame->GetLocalMapPointConnections();
		std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
		arma::mat R = m_pCurrKeyFrame->GetR();
		arma::vec t = m_pCurrKeyFrame->GetT();

		R = R.t();
		t = -R * t;

		std::map<int, int>::iterator iter7 = mLocalMapPointConnections.begin();
		std::map<int, int>::iterator end7 = mLocalMapPointConnections.end();
		for (; iter7 != end7; iter7++) {
			if (mCurrTrackedMapPoints.find(iter7->first) == mCurrTrackedMapPoints.end()) {
				WorldPoint pt = vLocalMapPoints[iter7->second];
				arma::vec x(3);
				x.at(0) = R.at(0, 0)*pt.x + R.at(0, 1)*pt.y + R.at(0, 2)*pt.z + t.at(0);
				x.at(1) = R.at(1, 0)*pt.x + R.at(1, 1)*pt.y + R.at(1, 2)*pt.z + t.at(1);
				x.at(2) = R.at(2, 0)*pt.x + R.at(2, 1)*pt.y + R.at(2, 2)*pt.z + t.at(2);
				CMapPoint* pNewMapPoint = new CMapPoint(x);
				m_pCurrKeyFrame->AddMapPointConnection(iter7->first, pNewMapPoint);
				AddMapPoint(pNewMapPoint);
			}
		}
	}
}

void	CMapManager::ProcessMapPlanes(void)
{

}

std::vector<int>	CMapManager::FindEpipolarGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1,
																	   const std::vector<SiftKeyPoint>& vKeyPoints2,
																	   const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair)
{
	arma::mat SrcInitData(3, (int)vCoarseMatchIdxPair.size()), DstInitData(3, (int)vCoarseMatchIdxPair.size());
	Concurrency::parallel_for(0, (int)vCoarseMatchIdxPair.size(), [&](int i) {
		SrcInitData.at(0, i) = vKeyPoints1[vCoarseMatchIdxPair[i].first].x;
		SrcInitData.at(1, i) = vKeyPoints1[vCoarseMatchIdxPair[i].first].y;
		SrcInitData.at(2, i) = 1.0;

		DstInitData.at(0, i) = vKeyPoints2[vCoarseMatchIdxPair[i].second].x;
		DstInitData.at(1, i) = vKeyPoints2[vCoarseMatchIdxPair[i].second].y;
		DstInitData.at(2, i) = 1.0;
	});

	CEightPoint eight;
	eight.SetInitData(SrcInitData, DstInitData);
	eight.SetThreshold(5.0);
	eight.EstimateFundamentalMatrixByRANSAC();

	return eight.GetInlierDataIndices();
}
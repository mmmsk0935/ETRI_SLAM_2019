#include "stdafx.h"
#include "MapManager.h"

CMapManager::CMapManager()
	: m_bStop(false)
	, m_pWorldMap(nullptr)
	, m_pWorldMapView(nullptr)
{

}

CMapManager::CMapManager(CWorldMap* pWorldMap/*, CWorldMapView* pWorldMapView*/)
	: m_pWorldMap(pWorldMap)
	, m_bStop(false)
{
}

CMapManager::~CMapManager()
{
}

void	CMapManager::SetWorldMapView(CWorldMapView* pWorldMapView)
{
	m_pWorldMapView = pWorldMapView;
}

void	CMapManager::Run(void)
{
	while (!IsStop()) {
		if (IsNewKeyFrame()) {
			ProcessMapPoints();
			ProcessMapPlanes();
			//ProcessMapLines();
			//ManageMapPlanes();

			DrawCurrentKeyFrame();
		}
	}
}

bool	CMapManager::IsNewKeyFrame(void)
{
	boost::mutex::scoped_lock lock(m_mutexKeyFrameQueue);
	return !m_qKeyFrames.empty();
}

void	CMapManager::SetStopFlag(bool bStop)
{
	boost::mutex::scoped_lock lock(m_mutexStop);
	m_bStop = bStop;
}

void	CMapManager::SetWorldMap(CWorldMap* pWorldMap)
{
	m_pWorldMap = pWorldMap;
}

void	CMapManager::Clear(void)
{

}

void	CMapManager::AddMapPoint(CMapPoint* pNewMapPoint)
{
	m_pWorldMap->AddMapPoint(pNewMapPoint);
}

void	CMapManager::AddKeyFrame(CSLAMKeyFrame* pNewKeyFrame)
{
	boost::mutex::scoped_lock lock(m_mutexKeyFrameQueue);
	m_qKeyFrames.push(pNewKeyFrame);
}

void	CMapManager::AddInitKeyFrame(CSLAMKeyFrame* pNewKeyFrame)
{
	m_pWorldMap->AddKeyFrame(pNewKeyFrame);
}

bool	CMapManager::IsStop(void)
{
	boost::mutex::scoped_lock lock(m_mutexStop);
	return m_bStop;
}

void	CMapManager::ProcessMapPoints(void)
{
	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();

	{
		boost::mutex::scoped_lock lock(m_mutexKeyFrameQueue);
		m_pCurrKeyFrame = m_qKeyFrames.front();
		m_qKeyFrames.pop();
	}

	std::map<int, CMapPoint *> mMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
	std::map<int, CMapPoint *>::iterator iter1 = mMapPointConnections.begin();
	std::map<int, CMapPoint *>::iterator end1 = mMapPointConnections.end();

	std::map<CSLAMKeyFrame *, int> mCovisibilities;
	for (; iter1 != end1; iter1++) {
		CMapPoint *pMapPoint = iter1->second;
		std::map<CSLAMKeyFrame*, int> mObservedFrames = pMapPoint->GetObservedKeyFrames();
		
		std::map<CSLAMKeyFrame*, int>::iterator iter2 = mObservedFrames.begin();
		std::map<CSLAMKeyFrame*, int>::iterator end2 = mObservedFrames.end();
		for (; iter2 != end2; iter2++) {
			if (iter2->first->IsBad())
				continue;
			mCovisibilities[iter2->first]++;
		}

		(iter1->second)->AddObservedKeyFrame(m_pCurrKeyFrame, iter1->first);
	}

	std::map<CSLAMKeyFrame *, int>::iterator iter2 = mCovisibilities.begin();
	std::map<CSLAMKeyFrame *, int>::iterator end2 = mCovisibilities.end();
	std::map<int, CSLAMKeyFrame *> mCovisibilityWeights;
	for (; iter2 != end2; iter2++)
		mCovisibilityWeights[iter2->second] = iter2->first;

	std::map<int, CSLAMKeyFrame *>::iterator iter3 = mCovisibilityWeights.begin();
	std::map<int, CSLAMKeyFrame *>::iterator end3 = mCovisibilityWeights.end();
	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();
	std::vector<float>& vSrcDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();
	
	arma::mat P1(3, 4), P2(3, 4);
	std::map<int, CMapPoint *> mSrcMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
	arma::mat R1 = m_pCurrKeyFrame->GetR();
	arma::vec t1 = m_pCurrKeyFrame->GetT();
	arma::vec c1 = -R1.t()*t1;

	P1(arma::span(0, 2), arma::span(0, 2)) = R1;
	P1(arma::span(0, 2), arma::span(3, 3)) = t1;

	cv::Mat cvSrcColorImage = m_pCurrKeyFrame->GetColorImage();

	std::vector<std::pair<int, int>> vCoarsePointMatchPair;

	int cnt = 0;

	std::vector<CMapPlane*> vObservedMapPlanes = m_pCurrKeyFrame->GetObservedMapPlanes();

	for (; iter3 != end3; iter3++) {
		if (iter3->first >= 50) { /// covisibility initial weight
			CSLAMKeyFrame* pPrevKeyFrame = iter3->second;
			
			if (pPrevKeyFrame == m_pCurrKeyFrame)
				continue;

			m_pCurrKeyFrame->AddAdjacentKeyFrame(pPrevKeyFrame);
			pPrevKeyFrame->AddAdjacentKeyFrame(m_pCurrKeyFrame);

			std::vector<SiftKeyPoint>& vDstKeyPoints = pPrevKeyFrame->GetSiftKeyPoints();
			std::vector<float>& vDstDescriptors = pPrevKeyFrame->GetSiftDescriptors();
			std::map<int, CMapPoint *> mDstMapPointConnections = pPrevKeyFrame->GetMapPointConnections();
			arma::mat R2 = pPrevKeyFrame->GetR();
			arma::vec t2 = pPrevKeyFrame->GetT();
			arma::vec c2 = -R2.t()*t2;
			cv::Mat cvDstColorImage = pPrevKeyFrame->GetColorImage();

			CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vCoarsePointMatchPair, 1);

			if (vCoarsePointMatchPair.size() < 50) {
				cv::Mat tmp;

				cv::hconcat(cvSrcColorImage, cvDstColorImage, tmp);
				cv::waitKey(5);

				continue;
			}

			std::vector<int> vFInlierIndices = FindEpipolarGeometricMatchFromTwoView(vSrcKeyPoints, vDstKeyPoints, vCoarsePointMatchPair);

			P2(arma::span(0, 2), arma::span(0, 2)) = R2;
			P2(arma::span(0, 2), arma::span(3, 3)) = t2;
			for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
				int nSrcKeyPointIdx = vCoarsePointMatchPair[vFInlierIndices[i]].first;
				int nDstKeyPointIdx = vCoarsePointMatchPair[vFInlierIndices[i]].second;

				std::map<int, CMapPoint *>::iterator iter5 = mSrcMapPointConnections.find(nSrcKeyPointIdx);
				std::map<int, CMapPoint *>::iterator iter6 = mDstMapPointConnections.find(nDstKeyPointIdx);

				if (iter5 == mSrcMapPointConnections.end() && iter6 == mDstMapPointConnections.end()) {
					double x1 = (double)vSrcKeyPoints[nSrcKeyPointIdx].x;
					double y1 = (double)vSrcKeyPoints[nSrcKeyPointIdx].y;
					double x2 = (double)vDstKeyPoints[nDstKeyPointIdx].x;
					double y2 = (double)vDstKeyPoints[nDstKeyPointIdx].y;

					// if a key point is in boder size, drop it
					if (x1 <= BORDER_SIZE || x1 >= cvSrcColorImage.cols - BORDER_SIZE || y1 < BORDER_SIZE || y1 >= cvSrcColorImage.rows - BORDER_SIZE)
						continue;
					if (x2 <= BORDER_SIZE || x2 >= cvDstColorImage.cols - BORDER_SIZE || y2 < BORDER_SIZE || y2 >= cvDstColorImage.rows - BORDER_SIZE)
						continue;
					/// need triangulate
					arma::mat A(4, 4), U, V;
					arma::vec x(3), X1(3), X2(3), s;

					x1 = (x1 - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
					y1 = (y1 - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
					x2 = (x2 - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
					y2 = (y2 - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);

					A.row(0) = x1 * P1.row(2) - P1.row(0);
					A.row(1) = y1 * P1.row(2) - P1.row(1);
					A.row(2) = x2 * P2.row(2) - P2.row(0);
					A.row(3) = y2 * P2.row(2) - P2.row(1);

					arma::svd(U, s, V, A);

					x.at(0) = V.at(0, 3);
					x.at(1) = V.at(1, 3);
					x.at(2) = V.at(2, 3);

					x /= V.at(3, 3);

					X1 = R1 * x + t1;
					X2 = R2 * x + t2;

					if (X1.at(2) < 0 || X2.at(2) < 0)
						continue;

					///< compute cos angle of two ray
					arma::vec n1 = x - c1;
					arma::vec n2 = x - c2;

					double dbCosAngle = arma::dot(n1, n2) / (arma::norm(n1)*arma::norm(n2));
					if (acos(abs(dbCosAngle)) < 0.1)
						continue;

					cnt++;
					CMapPoint* pNewMapPoint = new CMapPoint(x);
					pNewMapPoint->AddObservedKeyFrame(m_pCurrKeyFrame, nSrcKeyPointIdx);
					pNewMapPoint->AddObservedKeyFrame(pPrevKeyFrame, nDstKeyPointIdx);
					
					m_pCurrKeyFrame->AddMapPointConnection(nSrcKeyPointIdx, pNewMapPoint);
					pPrevKeyFrame->AddMapPointConnection(nDstKeyPointIdx, pNewMapPoint);

					m_pWorldMap->AddMapPoint(pNewMapPoint);

					/*///< if a new map points can be located on one of the observed map planes, it can be assigned the map plane
					for (int j = 0; j < (int)vObservedMapPlanes.size(); j++) {
						arma::vec abcd = vObservedMapPlanes[j]->GetParameters();

						double dbDistance = abs(abcd.at(0)*x.at(0) + abcd.at(1)*x.at(1) + abcd.at(2)*x.at(2) + abcd.at(3));

						if (dbDistance < 3.0)
							vObservedMapPlanes[j]->AddMapPoint(pNewMapPoint);
					}*/
				}
				else if (iter5 == mSrcMapPointConnections.end() && iter6 != mDstMapPointConnections.end()) {
					//mSrcMapPointConnections[nSrcKeyPointIdx] = iter6->second;
					m_pCurrKeyFrame->AddMapPointConnection(nSrcKeyPointIdx, iter6->second);
					(iter6->second)->AddObservedKeyFrame(m_pCurrKeyFrame, nSrcKeyPointIdx);
				}
				else if (iter5 != mSrcMapPointConnections.end() && iter6 == mDstMapPointConnections.end()) {
					//mDstMapPointConnections[nDstKeyPointIdx] = iter5->second;
					pPrevKeyFrame->AddMapPointConnection(nDstKeyPointIdx, iter5->second);
					(iter5->second)->AddObservedKeyFrame(pPrevKeyFrame, nDstKeyPointIdx);
				}
			}
		}
	}

	TRACE("the number of new map points:%d\n", cnt);

	for (int j = 0; j < (int)vObservedMapPlanes.size(); j++)
		vObservedMapPlanes[j]->UpdateParameter();

	std::cout << "new map point: " << cnt << std::endl;
	m_pWorldMap->AddKeyFrame(m_pCurrKeyFrame);

	if (vSLAMConfigure.m_nCameraType == CameraType::RGBD) {
		std::vector<WorldPoint>& vLocalMapPoints = m_pCurrKeyFrame->GetLocalMapPoints();
		std::map<int, int>& mLocalMapPointConnections = m_pCurrKeyFrame->GetLocalMapPointConnections();
		std::map<int, CMapPoint *> mCurrTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
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
				pNewMapPoint->AddObservedKeyFrame(m_pCurrKeyFrame, iter7->first);
				AddMapPoint(pNewMapPoint);
			}
		}
	}
}

void	CMapManager::ProcessMapLines(void)
{
	std::set<CSLAMKeyFrame *> sAdjacentKeyFrame = m_pCurrKeyFrame->GetAdjacentKeyFrames();
	std::set<CSLAMKeyFrame *>::iterator iter = sAdjacentKeyFrame.begin();
	std::set<CSLAMKeyFrame *>::iterator end = sAdjacentKeyFrame.end();

	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();
	std::vector<float>& vSrcDescriptors      = m_pCurrKeyFrame->GetSiftDescriptors();
	std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines = m_pCurrKeyFrame->GetKeyLines();
	cv::Mat cvSrcKeyLineDescriptors                         = m_pCurrKeyFrame->GetKeyLineDescriptors();
	arma::mat P1 = m_pCurrKeyFrame->GetP();
	arma::mat R1 = m_pCurrKeyFrame->GetR();
	arma::vec t1 = m_pCurrKeyFrame->GetT();

	R1 = R1.t();
	t1 = -R1*t1;
	
	arma::mat K = CFrame::K;
	arma::mat InvK = arma::inv(K);

	std::vector<std::pair<int, int>> vKeyPointCoarseMatchPair, vKeyLineCoarseMatchPair;
	
	for (; iter != end; iter++) {
		CSLAMKeyFrame* pPrevKeyFrame = (*iter);
		std::vector<SiftKeyPoint>& vDstKeyPoints = pPrevKeyFrame->GetSiftKeyPoints();
		std::vector<float>& vDstDescriptors      = pPrevKeyFrame->GetSiftDescriptors();
		std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines = pPrevKeyFrame->GetKeyLines();
		cv::Mat cvDstKeyLineDescriptors                         = pPrevKeyFrame->GetKeyLineDescriptors();
		arma::mat P2 = pPrevKeyFrame->GetP();
		arma::mat R2 = pPrevKeyFrame->GetR();
		arma::vec t2 = pPrevKeyFrame->GetT();
		
		R2 = R2.t();
		t2 = -R2*t2;
		CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vKeyPointCoarseMatchPair, 1);

		if (vKeyPointCoarseMatchPair.empty())
			continue;

		m_lsdWrapper.FindCoarseMatch(vSrcKeyLines, vDstKeyLines, cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vKeyLineCoarseMatchPair,
									 vSrcKeyPoints, vDstKeyPoints, vKeyPointCoarseMatchPair);

		if (vKeyLineCoarseMatchPair.empty())
			continue;

		/*cv::Mat img1, img2, img3;
		img1 = m_pCurrKeyFrame->GetColorImage();
		img2 = pPrevKeyFrame->GetColorImage();
		cv::hconcat(img1, img2, img3);
		cv::RNG rng(12345);*/

		for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
			const cv::line_descriptor::KeyLine& srcKeyLine = vSrcKeyLines[vKeyLineCoarseMatchPair[i].first];
			const cv::line_descriptor::KeyLine& dstKeyLine = vDstKeyLines[vKeyLineCoarseMatchPair[i].second];

			/*cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::Point pt1, pt2, pt3, pt4, pt5, pt6;

			pt1.x = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].startPointX);
			pt1.y = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].startPointY);

			pt2.x = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].endPointX);
			pt2.y = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].endPointY);

			pt3.x = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].pt.x);
			pt3.y = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].pt.y);

			pt4.x = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].startPointX) + img1.cols;
			pt4.y = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].startPointY);

			pt5.x = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].endPointX) + img1.cols;
			pt5.y = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].endPointY);

			pt6.x = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].pt.x) + img1.cols;
			pt6.y = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].pt.y);

			cv::line(img3, pt1, pt2, color, 2);
			cv::line(img3, pt4, pt5, color, 2);
			cv::line(img3, pt3, pt6, color, 2);*/

			arma::vec x1(3), x2(3), x3(3), x4(3);

			x1.at(0) = srcKeyLine.startPointX;
			x1.at(1) = srcKeyLine.startPointY;
			x1.at(2) = 1.0;

			x2.at(0) = srcKeyLine.endPointX;
			x2.at(1) = srcKeyLine.endPointY;
			x2.at(2) = 1.0;

			x3.at(0) = dstKeyLine.startPointX;
			x3.at(1) = dstKeyLine.startPointY;
			x3.at(2) = 1.0;

			x4.at(0) = dstKeyLine.endPointX;
			x4.at(1) = dstKeyLine.endPointY;
			x4.at(2) = 1.0;

			x1 = InvK*x1; x2 = InvK*x2; x3 = InvK*x3; x4 = InvK*x4;

			x1 = R1*x1; x2 = R1*x2;
			x3 = R2*x3; x4 = R2*x4;

			arma::vec n1 = arma::cross(x1, x2);
			arma::vec n2 = arma::cross(x3, x4);

			n1 /= arma::norm(n1);
			n2 /= arma::norm(n2);

			double d1 = -arma::dot(n1, t1);
			double d2 = -arma::dot(n2, t2);

			arma::vec dir = arma::cross(n1, n2);
			dir /= arma::norm(dir);

			double a1 = -(arma::dot(t1, n2) + d2) / arma::dot(x1, n2);
			double a2 = -(arma::dot(t1, n2) + d2) / arma::dot(x2, n2);
			double a3 = -(arma::dot(t2, n1) + d1) / arma::dot(x3, n1);
			double a4 = -(arma::dot(t2, n1) + d1) / arma::dot(x4, n1);

			arma::vec pt[4] = { t1 + a1 * x1, t1 + a2 * x2, t2 + a3 * x3, t2 + a4 * x4 };

			std::vector<double> vLineCorrTmp(18);
			vLineCorrTmp[0] =     t1.at(0); vLineCorrTmp[ 1] =    t1.at(1); vLineCorrTmp[ 2] =    t1.at(2);
			vLineCorrTmp[3] =  pt[0].at(0); vLineCorrTmp[ 4] = pt[0].at(1); vLineCorrTmp[ 5] = pt[0].at(2);
			vLineCorrTmp[6] =  pt[1].at(0); vLineCorrTmp[ 7] = pt[1].at(1); vLineCorrTmp[ 8] = pt[1].at(2);
			vLineCorrTmp[9] =     t2.at(0); vLineCorrTmp[10] =    t2.at(1); vLineCorrTmp[11] =    t2.at(2);
			vLineCorrTmp[12] = pt[2].at(0); vLineCorrTmp[13] = pt[2].at(1); vLineCorrTmp[14] = pt[2].at(2);
			vLineCorrTmp[15] = pt[3].at(0); vLineCorrTmp[16] = pt[3].at(1); vLineCorrTmp[17] = pt[3].at(2);

			m_pWorldMapView->SetLineCorrTmp(vLineCorrTmp);
			

			double dist, maxDist = 0;
			int idx1 = 0, idx2 = 0;
			for (int j = 0; j < 3; j++) {
				for (int k = j + 1; k < 4; k++) {
					dist = std::sqrt(arma::dot(pt[j] - pt[k], pt[j] - pt[k]));
					if (dist > maxDist) {
						idx1 = j;
						idx2 = k;
					}
				}
			}

			std::vector<double> v3DMapLines(6);

			v3DMapLines[0] = pt[0].at(0); v3DMapLines[1] = pt[0].at(1); v3DMapLines[2] = pt[0].at(2);
			v3DMapLines[3] = pt[1].at(0); v3DMapLines[4] = pt[1].at(1); v3DMapLines[5] = pt[1].at(2);
			m_pWorldMapView->AddMapLines(v3DMapLines);
			m_pWorldMapView->Invalidate(FALSE);
			arma::mat M(3, 3);
			M.col(0) = dir;
			M.col(1) = n1;
			M.col(2) = n2;
			arma::vec b(3);
			b.at(0) = 0;
			b.at(1) = -d1;
			b.at(2) = -d2;

			arma::vec pt1 = arma::inv(M) * b;
			arma::vec pt2 = pt1 + dir*10;
			pt1 = pt1 - dir * 10;

			//CMapLine* pNewMapLine = new CMapLine(pt[idx1], pt[idx2], dir);
			CMapLine* pNewMapLine = new CMapLine(pt1, pt2, dir);
			pNewMapLine->AddObservedKeyFrame(m_pCurrKeyFrame, vKeyLineCoarseMatchPair[i].first);
			pNewMapLine->AddObservedKeyFrame(pPrevKeyFrame, vKeyLineCoarseMatchPair[i].second);
			m_pCurrKeyFrame->AddMapLineConnection(vKeyLineCoarseMatchPair[i].first, pNewMapLine);
			pPrevKeyFrame->AddMapLineConnection(vKeyLineCoarseMatchPair[i].second, pNewMapLine);
		}

		/*cv::imshow("line match test", img3);

		cv::waitKey(0);*/
	}
}

void	CMapManager::ProcessMapPlanes(void)
{
	std::set<CSLAMKeyFrame *> sAdjacentKeyFrame = m_pCurrKeyFrame->GetAdjacentKeyFrames();
	std::set<CSLAMKeyFrame *>::iterator iter = sAdjacentKeyFrame.begin();
	std::set<CSLAMKeyFrame *>::iterator end = sAdjacentKeyFrame.end();

	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();
	std::vector<float>& vSrcDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();
	std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines = m_pCurrKeyFrame->GetKeyLines();
	cv::Mat cvSrcKeyLineDescriptors = m_pCurrKeyFrame->GetKeyLineDescriptors();
	cv::Mat cvSrcColorImage = m_pCurrKeyFrame->GetColorImage();

	std::map<int, CMapPoint *>& mSrcMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();

	std::vector<std::pair<int, int>> vCoarseMatchPair, vCoarseMatchPairTmp;
	std::vector<int> vHInliers, vHOutliers;
	std::cout << "the number of adjacent key frame" << sAdjacentKeyFrame.size() << std::endl;

	///< find the dominant line in the current key-frame
	float fSrcMaxLineLength = 0;
	int nSrcLineIdx = -1;
	double pSrcLineParams[6] = {0};
	for (int i = 0; i < (int)vSrcKeyLines.size(); i++) {
		if (fSrcMaxLineLength < vSrcKeyLines[i].lineLength) {
			fSrcMaxLineLength = vSrcKeyLines[i].lineLength;
			nSrcLineIdx = i;
		}
	}
	if (fSrcMaxLineLength != 0) {
		cv::line_descriptor::KeyLine srcKeyLine = vSrcKeyLines[nSrcLineIdx];
		pSrcLineParams[0] = srcKeyLine.startPointY - srcKeyLine.endPointY;
		pSrcLineParams[1] = srcKeyLine.endPointX - srcKeyLine.startPointX;
		pSrcLineParams[2] = srcKeyLine.startPointX*srcKeyLine.endPointY - srcKeyLine.endPointX*srcKeyLine.startPointY;
		pSrcLineParams[3] =  pSrcLineParams[1];
		pSrcLineParams[4] = -pSrcLineParams[0];
		pSrcLineParams[5] = -pSrcLineParams[1]*srcKeyLine.pt.x + pSrcLineParams[0]*srcKeyLine.pt.y;
		
		double dbNorm = sqrt(pSrcLineParams[0]*pSrcLineParams[0] + pSrcLineParams[1]*pSrcLineParams[1]);
		pSrcLineParams[0] /= dbNorm;
		pSrcLineParams[1] /= dbNorm;
		pSrcLineParams[2] /= dbNorm;

		dbNorm = sqrt(pSrcLineParams[3]*pSrcLineParams[3] + pSrcLineParams[4]*pSrcLineParams[4]);
		pSrcLineParams[3] /= dbNorm;
		pSrcLineParams[4] /= dbNorm;
		pSrcLineParams[5] /= dbNorm;

		/*std::cout << "curr dominant line segment: " << std::endl;
		std::cout << "[" << srcKeyLine.startPointX << ", " << srcKeyLine.startPointY << "]->[" 
				  << srcKeyLine.endPointX << ",  " << srcKeyLine.endPointY << "]->["
				  << srcKeyLine.pt.x << ", " << srcKeyLine.pt.y << "]" << std::endl;

		std::cout << "[";
		for (int idx = 0; idx < 6; idx++)
			std::cout << pSrcLineParams[idx] << ", ";
		std::cout << "]" << std::endl;*/
	}
	/*double x1 = vSrcKeyLines[nSrcLineIdx].pt.x + pSrcLineParams[0]*20;
	double y1 = vSrcKeyLines[nSrcLineIdx].pt.y + pSrcLineParams[1]*20;
	double x2 = vSrcKeyLines[nSrcLineIdx].pt.x - pSrcLineParams[0]*20;
	double y2 = vSrcKeyLines[nSrcLineIdx].pt.y - pSrcLineParams[1]*20;
	cv::line(cvSrcColorImage, cv::Point(vSrcKeyLines[nSrcLineIdx].startPointX, vSrcKeyLines[nSrcLineIdx].startPointY),
							  cv::Point(vSrcKeyLines[nSrcLineIdx].endPointX, vSrcKeyLines[nSrcLineIdx].endPointY), cv::Scalar(0, 255, 0), 2);*/

	/*double tmp1 = pSrcLineParams[3]*x1 + pSrcLineParams[4]*y1 + pSrcLineParams[5];
	double tmp2 = pSrcLineParams[3]*x2 + pSrcLineParams[4]*y2 + pSrcLineParams[5];
	std::cout << tmp1 << ", " << tmp2 << std::endl;
	cv::line(cvSrcColorImage, cv::Point(x1, y1),
							  cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);
	cv::imshow("src dominant line", cvSrcColorImage);*/
	for (; iter != end; iter++) {
		CSLAMKeyFrame* pPrevKeyFrame = (*iter);
		std::vector<SiftKeyPoint>& vDstKeyPoints = pPrevKeyFrame->GetSiftKeyPoints();
		std::vector<float>& vDstDescriptors = pPrevKeyFrame->GetSiftDescriptors();
		std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines = pPrevKeyFrame->GetKeyLines();
		cv::Mat cvDstKeyLineDescriptors = pPrevKeyFrame->GetKeyLineDescriptors();
		std::map<int, CMapPoint *>& mDstMapPointConnections = pPrevKeyFrame->GetMapPointConnections();
		cv::Mat cvDstColorImage = pPrevKeyFrame->GetColorImage();

		if (vSrcKeyLines.empty() || vDstKeyLines.empty())
			continue;

		CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vCoarseMatchPair, 1);

		if (vCoarseMatchPair.size() < 50)
			continue;

		///< create a new map plane if we find a dominant line in the image plane
		///< find the dominant line in the current key-frame
		float fDstMaxLineLength = 0;
		int nDstLineIdx = -1;
		double pDstLineParams[6] = { 0 };
		for (int i = 0; i < (int)vDstKeyLines.size(); i++) {
			if (fDstMaxLineLength < vDstKeyLines[i].lineLength) {
				fDstMaxLineLength = vDstKeyLines[i].lineLength;
				nDstLineIdx = i;
			}
		}
		if (fDstMaxLineLength != 0) {
			cv::line_descriptor::KeyLine dstKeyLine = vDstKeyLines[nDstLineIdx];
			pDstLineParams[0] = dstKeyLine.startPointY - dstKeyLine.endPointY;
			pDstLineParams[1] = dstKeyLine.endPointX - dstKeyLine.startPointX;
			pDstLineParams[2] = dstKeyLine.startPointX*dstKeyLine.endPointY - dstKeyLine.endPointX*dstKeyLine.startPointY;
			pDstLineParams[3] =  pDstLineParams[1];
			pDstLineParams[4] = -pDstLineParams[0];
			pDstLineParams[5] = -pDstLineParams[1]*dstKeyLine.pt.x + pDstLineParams[0]*dstKeyLine.pt.y;

			double dbNorm = sqrt(pDstLineParams[0]* pDstLineParams[0] + pDstLineParams[1]*pDstLineParams[1]);
			pDstLineParams[0] /= dbNorm;
			pDstLineParams[1] /= dbNorm;
			pDstLineParams[2] /= dbNorm;

			dbNorm = sqrt(pDstLineParams[3]*pDstLineParams[3] + pDstLineParams[4]*pDstLineParams[4]);
			pDstLineParams[3] /= dbNorm;
			pDstLineParams[4] /= dbNorm;
			pDstLineParams[5] /= dbNorm;

			/*std::cout << "previous dominant line segment: " << std::endl;
			std::cout << "[" << dstKeyLine.startPointX << ", " << dstKeyLine.startPointY << "]->["
					  << dstKeyLine.endPointX << ",  " << dstKeyLine.endPointY << "]->["
					  << dstKeyLine.pt.x << ", " << dstKeyLine.pt.y << "]" << std::endl;

			std::cout << "[";
			for (int idx = 0; idx < 6; idx++)
				std::cout << pDstLineParams[idx] << ", ";
			std::cout << "]" << std::endl;*/
		}
		/*cv::line(cvDstColorImage, cv::Point(vDstKeyLines[nDstLineIdx].startPointX, vDstKeyLines[nDstLineIdx].startPointY),
								  cv::Point(vDstKeyLines[nDstLineIdx].endPointX, vDstKeyLines[nDstLineIdx].endPointY), cv::Scalar(0, 255, 0), 2);
		cv::imshow("dst dominant line", cvDstColorImage);*/

		std::vector<CMapPoint*> vLeftSideMapPoints, vRightSideMapPoints;
		if (fSrcMaxLineLength > fDstMaxLineLength) {
			//std::cout << "current frame" << std::endl;
			for (int i = 0; i < (int)vSrcKeyPoints.size(); i++) {
				if (mSrcMapPointConnections.find(i) != mSrcMapPointConnections.end()) {
					double x = vSrcKeyPoints[i].x;
					double y = vSrcKeyPoints[i].y;

					double dbDistance1 = pSrcLineParams[0]*x + pSrcLineParams[1]*y + pSrcLineParams[2];
					double dbDistance2 = pSrcLineParams[3]*x + pSrcLineParams[4]*y + pSrcLineParams[5];
					//std::cout << dbDistance1 << ",  " << dbDistance2 << std::endl;
					if (dbDistance1 < 0 && abs(dbDistance1) < fSrcMaxLineLength && abs(dbDistance2) < fSrcMaxLineLength*0.5)
						vLeftSideMapPoints.push_back(mSrcMapPointConnections[i]);
					else if (dbDistance1 > 0 && abs(dbDistance1) < fSrcMaxLineLength && abs(dbDistance2) < fSrcMaxLineLength*0.5)
						vRightSideMapPoints.push_back(mSrcMapPointConnections[i]);
				}
			}
		}
		else {
			//std::cout << "previous frame" << std::endl;
			for (int i = 0; i < (int)vDstKeyPoints.size(); i++) {
				if (mDstMapPointConnections.find(i) != mDstMapPointConnections.end()) {
					double x = vDstKeyPoints[i].x;
					double y = vDstKeyPoints[i].y;

					double dbDistance1 = pDstLineParams[0]*x + pDstLineParams[1]*y + pDstLineParams[2];
					double dbDistance2 = pDstLineParams[3]*x + pDstLineParams[4]*y + pDstLineParams[5];
					//std::cout << dbDistance1 << ",  " << dbDistance2 << std::endl;
					if (dbDistance1 < 0 && abs(dbDistance1) < fDstMaxLineLength && abs(dbDistance2) < fDstMaxLineLength*0.5)
						vLeftSideMapPoints.push_back(mDstMapPointConnections[i]);
					else if (dbDistance1 > 0 && abs(dbDistance1) < fDstMaxLineLength && abs(dbDistance2) < fDstMaxLineLength*0.5)
						vRightSideMapPoints.push_back(mDstMapPointConnections[i]);
				}
			}
		}

		/*std::cout << "the dominant line length: " << fSrcMaxLineLength << ", " << fDstMaxLineLength << std::endl;
		std::cout << "the number of local plane map points" << vLeftSideMapPoints.size() << ", " << vRightSideMapPoints.size() << std::endl;*/

		///< create a new map plane or extension
		if (vLeftSideMapPoints.size() > 30) {
			std::vector<CMapPoint*> vExistedMapPlanes, vMapPlaneCandidates;
			std::map<CMapPlane*, int> mMapPlaneWithMapPoints;
			std::map<int, CMapPlane*> mSortedMapPlanes;

			for (int i = 0; i < (int)vLeftSideMapPoints.size(); i++) {
				if (vLeftSideMapPoints[i]->GetMapPlane()) {
					vExistedMapPlanes.push_back(vLeftSideMapPoints[i]);
					mMapPlaneWithMapPoints[vLeftSideMapPoints[i]->GetMapPlane()]++;
				}
				else
					vMapPlaneCandidates.push_back(vLeftSideMapPoints[i]);
			}
			if (vExistedMapPlanes.size() > vLeftSideMapPoints.size()*0.5) {
				///< sort map planes with the number of map points
				std::map<CMapPlane*, int>::iterator iter1 = mMapPlaneWithMapPoints .begin();
				std::map<CMapPlane*, int>::iterator end1 = mMapPlaneWithMapPoints.end();
				for (;iter1 != end1; iter1++)
					mSortedMapPlanes[iter1->second] = iter1->first;
				///< extension
				for (int i = 0; i < (int)vMapPlaneCandidates.size(); i++) {
					std::map<int, CMapPlane*>::iterator iter2 = mSortedMapPlanes.begin();
					std::map<int, CMapPlane*>::iterator end2 = mSortedMapPlanes.end();
					CMapPoint* pMapPoint = vMapPlaneCandidates[i];
					arma::vec x = pMapPoint->GetPosition();

					double dbMinDistance = INT_MAX;
					CMapPlane* pClosestMapPlane = nullptr;
					for (; iter2 != end2; iter2++) {
						CMapPlane* pMapPlane = iter2->second;
						arma::vec abcd = pMapPlane->GetParameters();
						double dbNorm = sqrt(abcd.at(0)*abcd.at(0) + abcd.at(1)*abcd.at(1) + abcd.at(2)*abcd.at(2));
						double dbDistance = abs(abcd.at(0)*x.at(0) + abcd.at(1)*x.at(1) + abcd.at(2)*x.at(2) + abcd.at(3))/dbNorm;

						if (dbDistance < dbMinDistance) {
							dbMinDistance = dbDistance;
							pClosestMapPlane = pMapPlane;
						}
					}

					if (dbMinDistance < 0.1) {
						pClosestMapPlane->AddMapPoint(pMapPoint);
						pMapPoint->SetMapPlane(pClosestMapPlane);

						std::map<CSLAMKeyFrame*, int> mObservedKeyFrames = pMapPoint->GetObservedKeyFrames();
						std::map<CSLAMKeyFrame*, int>::iterator iter3 = mObservedKeyFrames.begin();
						std::map<CSLAMKeyFrame*, int>::iterator end3 = mObservedKeyFrames.end();
						for (; iter3 != end3; iter3++)
							iter3->first->AddObservedMapPlane(pClosestMapPlane);
					}
				}
			}
			else {
				///< create a new map plane
				arma::mat InitWorldPoints;
				for (int i = 0; i < (int)vMapPlaneCandidates.size(); i++)
					InitWorldPoints.insert_cols(InitWorldPoints.n_cols, vMapPlaneCandidates[i]->GetPosition());

				CPlaneEstimator estimator;
				estimator.SetInitData(InitWorldPoints);
				estimator.SetThreshold(0.1);
				estimator.EstimatePlaneByRANSAC();

				const std::vector<int>& vPlaneInliers = estimator.GetInlierDataIndices();
				if (vPlaneInliers.size() > InitWorldPoints.n_cols*0.5) {
					CMapPlane* pNewMapPlane = new CMapPlane();
					const arma::vec& n = estimator.GetNormalVector();
					double dbDistance = estimator.GetDistance();
					arma::vec abcd(4);

					abcd.at(0) = n.at(0); abcd.at(1) = n.at(1); abcd.at(2) = n.at(2); abcd.at(3) = dbDistance;
					pNewMapPlane->SetPlaneEquation(abcd);
					
					for (int i = 0; i < (int)vPlaneInliers.size(); i++) {
						pNewMapPlane->AddMapPoint(vMapPlaneCandidates[vPlaneInliers[i]]);
						vMapPlaneCandidates[vPlaneInliers[i]]->SetMapPlane(pNewMapPlane);
					}

					m_pWorldMap->AddMapPlane(pNewMapPlane);
				}
			}
		}
		if (vRightSideMapPoints.size() > 30) {
			std::vector<CMapPoint*> vExistedMapPlanes, vMapPlaneCandidates;
			std::map<CMapPlane*, int> mMapPlaneWithMapPoints;
			std::map<int, CMapPlane*> mSortedMapPlanes;

			for (int i = 0; i < (int)vRightSideMapPoints.size(); i++) {
				if (vRightSideMapPoints[i]->GetMapPlane()) {
					vExistedMapPlanes.push_back(vRightSideMapPoints[i]);
					mMapPlaneWithMapPoints[vRightSideMapPoints[i]->GetMapPlane()]++;
				}
				else
					vMapPlaneCandidates.push_back(vRightSideMapPoints[i]);
			}
			if (vExistedMapPlanes.size() > vRightSideMapPoints.size()*0.5) {
				///< sort map planes with the number of map points
				std::map<CMapPlane*, int>::iterator iter1 = mMapPlaneWithMapPoints.begin();
				std::map<CMapPlane*, int>::iterator end1 = mMapPlaneWithMapPoints.end();
				for (; iter1 != end1; iter1++)
					mSortedMapPlanes[iter1->second] = iter1->first;
				///< extension
				for (int i = 0; i < (int)vMapPlaneCandidates.size(); i++) {
					std::map<int, CMapPlane*>::iterator iter2 = mSortedMapPlanes.begin();
					std::map<int, CMapPlane*>::iterator end2 = mSortedMapPlanes.end();
					CMapPoint* pMapPoint = vMapPlaneCandidates[i];
					arma::vec x = pMapPoint->GetPosition();

					double dbMinDistance = INT_MAX;
					CMapPlane* pClosestMapPlane = nullptr;
					for (; iter2 != end2; iter2++) {
						CMapPlane* pMapPlane = iter2->second;
						arma::vec abcd = pMapPlane->GetParameters();
						double dbNorm = sqrt(abcd.at(0)*abcd.at(0) + abcd.at(1)*abcd.at(1) + abcd.at(2)*abcd.at(2));
						double dbDistance = abs(abcd.at(0)*x.at(0) + abcd.at(1)*x.at(1) + abcd.at(2)*x.at(2) + abcd.at(3)) / dbNorm;

						if (dbDistance < dbMinDistance) {
							dbMinDistance = dbDistance;
							pClosestMapPlane = pMapPlane;
						}
					}

					if (dbMinDistance < 0.1) {
						pClosestMapPlane->AddMapPoint(pMapPoint);
						pMapPoint->SetMapPlane(pClosestMapPlane);

						std::map<CSLAMKeyFrame*, int> mObservedKeyFrames = pMapPoint->GetObservedKeyFrames();
						std::map<CSLAMKeyFrame*, int>::iterator iter3 = mObservedKeyFrames.begin();
						std::map<CSLAMKeyFrame*, int>::iterator end3 = mObservedKeyFrames.end();
						for (; iter3 != end3; iter3++)
							iter3->first->AddObservedMapPlane(pClosestMapPlane);
					}
				}
			}
			else {
				///< create a new map plane
				arma::mat InitWorldPoints;
				for (int i = 0; i < (int)vMapPlaneCandidates.size(); i++)
					InitWorldPoints.insert_cols(InitWorldPoints.n_cols, vMapPlaneCandidates[i]->GetPosition());

				CPlaneEstimator estimator;
				estimator.SetInitData(InitWorldPoints);
				estimator.SetThreshold(0.1);
				estimator.EstimatePlaneByRANSAC();

				const std::vector<int>& vPlaneInliers = estimator.GetInlierDataIndices();
				if (vPlaneInliers.size() > InitWorldPoints.n_cols*0.5) {
					CMapPlane* pNewMapPlane = new CMapPlane();
					const arma::vec& n = estimator.GetNormalVector();
					double dbDistance = estimator.GetDistance();
					arma::vec abcd(4);

					abcd.at(0) = n.at(0); abcd.at(1) = n.at(1); abcd.at(2) = n.at(2); abcd.at(3) = dbDistance;
					pNewMapPlane->SetPlaneEquation(abcd);

					for (int i = 0; i < (int)vPlaneInliers.size(); i++) {
						pNewMapPlane->AddMapPoint(vMapPlaneCandidates[vPlaneInliers[i]]);
						vMapPlaneCandidates[vPlaneInliers[i]]->SetMapPlane(pNewMapPlane);
					}

					m_pWorldMap->AddMapPlane(pNewMapPlane);
				}
			}
		}

		///< create a new map plane if we find a dominant line in the image plane
		//std::vector<std::pair<int, int>> vKeyLineCoarseMatchPair;
		/*m_lsdWrapper.FindCoarseMatch(cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vKeyLineCoarseMatchPair);

		{
			cv::Mat tmp;
			cv::hconcat(cvSrcColorImage, cvDstColorImage, tmp);
			for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
				cv::line_descriptor::KeyLine keyLine1 = vSrcKeyLines[vKeyLineCoarseMatchPair[i].first];
				cv::line_descriptor::KeyLine keyLine2 = vDstKeyLines[vKeyLineCoarseMatchPair[i].second];

				cv::line(tmp, cv::Point(keyLine1.startPointX, keyLine1.startPointY), cv::Point(keyLine1.endPointX, keyLine1.endPointY), cv::Scalar(0, 255, 0), 2);
				cv::line(tmp, cv::Point(keyLine2.startPointX+cvSrcColorImage.cols, keyLine2.startPointY), cv::Point(keyLine2.endPointX + cvSrcColorImage.cols, keyLine2.endPointY), cv::Scalar(0, 255, 0), 2);
				cv::line(tmp, cv::Point(keyLine1.pt.x, keyLine1.pt.y), cv::Point(keyLine2.pt.x + cvSrcColorImage.cols, keyLine2.pt.y), cv::Scalar(0, 255, 0), 2);
				
			}
			cv::imshow("dominant line segment", tmp);
		}*/

		/*m_lsdWrapper.FindCoarseMatch(vSrcKeyLines, vDstKeyLines, cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vKeyLineCoarseMatchPair, 
									 vSrcKeyPoints, vDstKeyPoints, vCoarseMatchPair);

		{
			cv::Mat tmp;
			cv::hconcat(cvSrcColorImage, cvDstColorImage, tmp);
			for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
				cv::line_descriptor::KeyLine keyLine1 = vSrcKeyLines[vKeyLineCoarseMatchPair[i].first];
				cv::line_descriptor::KeyLine keyLine2 = vDstKeyLines[vKeyLineCoarseMatchPair[i].second];

				cv::line(tmp, cv::Point(keyLine1.startPointX, keyLine1.startPointY), cv::Point(keyLine1.endPointX, keyLine1.endPointY), cv::Scalar(0, 255, 0), 2);
				cv::line(tmp, cv::Point(keyLine2.startPointX + cvSrcColorImage.cols, keyLine2.startPointY), cv::Point(keyLine2.endPointX + cvSrcColorImage.cols, keyLine2.endPointY), cv::Scalar(0, 255, 0), 2);
				cv::line(tmp, cv::Point(keyLine1.pt.x, keyLine1.pt.y), cv::Point(keyLine2.pt.x + cvSrcColorImage.cols, keyLine2.pt.y), cv::Scalar(0, 255, 0), 2);

			}
			cv::imshow("dominant line segment", tmp);
		}

		///< we find the longest line segments
		float fMaxLineLength = 0;
		int nMaxLineIdx = -1;
		for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
			cv::line_descriptor::KeyLine keyLine1 = vSrcKeyLines[vKeyLineCoarseMatchPair[i].first];
			cv::line_descriptor::KeyLine keyLine2 = vDstKeyLines[vKeyLineCoarseMatchPair[i].second];

			if (keyLine1.lineLength > keyLine2.lineLength) {
				if (fMaxLineLength < keyLine1.lineLength) {
					fMaxLineLength = keyLine1.lineLength;
					nMaxLineIdx = i;
				}
			}
			else {
				if (fMaxLineLength < keyLine2.lineLength) {
					fMaxLineLength = keyLine2.lineLength;
					nMaxLineIdx = i;
				}
			}
		}

		if (fMaxLineLength < 50)
			continue;

		///< separate point correspondences based on the dominant line on each image plane
		cv::line_descriptor::KeyLine keyLine1 = vSrcKeyLines[vKeyLineCoarseMatchPair[nMaxLineIdx].first];
		cv::line_descriptor::KeyLine keyLine2 = vDstKeyLines[vKeyLineCoarseMatchPair[nMaxLineIdx].second];
		double a1, b1, c1, a2, b2, c2;

		cv::line(cvSrcColorImage, cv::Point(keyLine1.startPointX, keyLine1.startPointY), cv::Point(keyLine1.endPointX, keyLine1.endPointY), cv::Scalar(0, 255, 0), 2);
		cv::imshow("dominant line segment", cvSrcColorImage);

		a1 = keyLine1.startPointY - keyLine1.endPointY;
		b1 = keyLine1.endPointX - keyLine1.startPointX;
		c1 = keyLine1.startPointX*keyLine1.endPointY - keyLine1.endPointX*keyLine1.startPointY;

		a2 = keyLine2.startPointY - keyLine2.endPointY;
		b2 = keyLine2.endPointX - keyLine2.startPointX;
		c2 = keyLine2.startPointX*keyLine2.endPointY - keyLine2.endPointX*keyLine2.startPointY;

		std::vector<std::pair<int, int>> vLeftSideMatchPair, vRightSideMatchPair;
		for (int i = 0; i < (int)vCoarseMatchPair.size(); i++) {
			if (mSrcMapPointConnections.find(vCoarseMatchPair[i].first) == mSrcMapPointConnections.end() || 
				mDstMapPointConnections.find(vCoarseMatchPair[i].second) == mDstMapPointConnections.end())
				continue;

			CMapPoint* pMapPoint1 = mSrcMapPointConnections[vCoarseMatchPair[i].first];
			CMapPoint* pMapPoint2 = mSrcMapPointConnections[vCoarseMatchPair[i].second];

			if (pMapPoint1 != pMapPoint2)
				continue;

			SiftKeyPoint key1 = vSrcKeyPoints[vCoarseMatchPair[i].first];
			SiftKeyPoint key2 = vDstKeyPoints[vCoarseMatchPair[i].second];

			double dbDistance1 = (a1*key1.x + b1*key1.y + c1);
			double dbDistance2 = (a2*key2.x + b2*key2.y + c2);

			if (dbDistance1*dbDistance2 > 0) {
				if (abs(dbDistance1)/sqrt(a1*a1 + b1*b1) < fMaxLineLength && 
					abs(dbDistance2)/sqrt(a2*a2 + b2*b2) < fMaxLineLength) {
					if (dbDistance1 < 0)
						vLeftSideMatchPair.push_back(vCoarseMatchPair[i]);
					else
						vRightSideMatchPair.push_back(vCoarseMatchPair[i]);
				}
			}
		}

		///< we assume that the left side match pair and right side match pair are correspondences on a local planar transformation, that is local homography
		std::cout << "the number of map points on local homography" << vLeftSideMatchPair.size() << ", " << vRightSideMatchPair.size() << std::endl;*/
		/*FindPlanarTransformMatches(vSrcKeyPoints, vDstKeyPoints, vCoarseMatchPair, vHInliers, vHOutliers);

		std::vector<CMapPoint*> vMapPointsOnPlane;
		std::map<CMapPlane*, int> mMapPlaneWithPointCount;

		for (int i = 0; i < (int)vHInliers.size(); i++) {
			if (mSrcMapPointConnections.find(vCoarseMatchPair[vHInliers[i]].first) == mSrcMapPointConnections.end())
				continue;
			if (mDstMapPointConnections.find(vCoarseMatchPair[vHInliers[i]].second) == mDstMapPointConnections.end())
				continue;
			CMapPoint* pMapPoint1 = mSrcMapPointConnections[vCoarseMatchPair[vHInliers[i]].first];
			CMapPoint* pMapPoint2 = mDstMapPointConnections[vCoarseMatchPair[vHInliers[i]].second];

			if (pMapPoint1 == pMapPoint2) {
				vMapPointsOnPlane.push_back(pMapPoint1);
				if (pMapPoint1->GetMapPlane() != nullptr)
					mMapPlaneWithPointCount[pMapPoint1->GetMapPlane()]++;
			}
		}*/
		
		//if (mMapPlaneWithPointCount.empty()) {	///< create a new map plane
		//	arma::vec abcd(4);	///< plane coefficient for ax+by+cz+d=0
		//	std::vector<int> vPlaneInliers;
		//	EstimateNewMapPlanes(vMapPointsOnPlane, abcd, vPlaneInliers);

		//	if (vPlaneInliers.size() > (int)vMapPointsOnPlane.size() * 0.5) {
		//		///< verify the estimated map plane to compute distance nearby projected points
		//		double dbMinX = 9999, dbMaxX = 0, dbMinY = 9999, dbMaxY = 0;
		//		for (int i = 0; i < (int)vPlaneInliers.size(); i++) {
		//			CMapPoint* pMapPoint = vMapPointsOnPlane[i];
		//			std::map<CSLAMKeyFrame*, int>& mObservedKeyFrame = pMapPoint->GetObservedKeyFrames();
		//			int nPointFeatureIdx = mObservedKeyFrame[m_pCurrKeyFrame];

		//			dbMinX = vSrcKeyPoints[nPointFeatureIdx].x < dbMinX ? vSrcKeyPoints[nPointFeatureIdx].x : dbMinX;
		//			dbMaxX = vSrcKeyPoints[nPointFeatureIdx].x > dbMaxX ? vSrcKeyPoints[nPointFeatureIdx].x : dbMaxX;
		//			dbMinY = vSrcKeyPoints[nPointFeatureIdx].y < dbMinY ? vSrcKeyPoints[nPointFeatureIdx].y : dbMinY;
		//			dbMaxY = vSrcKeyPoints[nPointFeatureIdx].y > dbMaxY ? vSrcKeyPoints[nPointFeatureIdx].y : dbMaxY;
		//		}

		//		std::cout << "area: " << (dbMaxX - dbMinX) * (dbMaxY - dbMinY) << std::endl;

		//		std::vector<int> vPointFeatureIdxInPlane = m_pCurrKeyFrame->GetPointFeatureIdxInPlane(dbMinX, dbMaxX, dbMinY, dbMaxY);
		//		int nClosePointCount = 0, nFarPointCount = 0;
		//		std::vector<CMapPoint*> vCloseMapPoints;
		//		for (int i = 0; i < (int)vPointFeatureIdxInPlane.size(); i++) {
		//			if (mSrcMapPointConnections.find(vPointFeatureIdxInPlane[i]) != mSrcMapPointConnections.end()) {
		//				CMapPoint* pMapPoint = mSrcMapPointConnections[vPointFeatureIdxInPlane[i]];
		//				if (std::find(vMapPointsOnPlane.begin(), vMapPointsOnPlane.end(), pMapPoint) == vMapPointsOnPlane.end()) {
		//					// calculate distance from the point to plane
		//					arma::vec x = pMapPoint->GetPosition();
		//					double dbDistance = abs(arma::dot(x, abcd(arma::span(0, 2))) + abcd.at(3));

		//					if (abs(dbDistance) < 5) {
		//						nClosePointCount++;
		//						vCloseMapPoints.push_back(pMapPoint);
		//					}
		//					else
		//						nFarPointCount++;
		//				}
		//			}
		//		}
		//		std::cout << "the number of closed points: " << nClosePointCount << ", the number of far points: " << nFarPointCount << std::endl;
		//		if (nClosePointCount > nFarPointCount) {
		//			// create a new map plane
		//			CMapPlane* pNewMapPlane = new CMapPlane(abcd);
		//			for (int i = 0; i < (int)vPlaneInliers.size(); i++) {
		//				pNewMapPlane->AddMapPoint(vMapPointsOnPlane[vPlaneInliers[i]]);
		//				vMapPointsOnPlane[vPlaneInliers[i]]->SetMapPlane(pNewMapPlane);
		//			}
		//			for (int i = 0; i < (int)vCloseMapPoints.size(); i++) {
		//				pNewMapPlane->AddMapPoint(vCloseMapPoints[i]);
		//				vCloseMapPoints[i]->SetMapPlane(pNewMapPlane);
		//			}
		//			pNewMapPlane->UpdateParameter();

		//			m_pCurrKeyFrame->AddObservedMapPlane(pNewMapPlane);
		//			pNewMapPlane->AddObservedKeyFrame(pPrevKeyFrame);
		//			pNewMapPlane->AddObservedKeyFrame(m_pCurrKeyFrame);
		//			m_pWorldMap->AddMapPlane(pNewMapPlane);
		//		}
		//	}
		//}
		//else {
		//	///< extend existed map plane or split

		//	///< firstly, sort map planes with the number of map points on the plane
		//	std::map<int, CMapPlane*> mSortedMapPlaneWithMapPointCount;
		//	std::map<CMapPlane*, int>::iterator iter1 = mMapPlaneWithPointCount.begin();
		//	std::map<CMapPlane*, int>::iterator end1 = mMapPlaneWithPointCount.end();
		//	for (; iter1 != end1; iter1++)
		//		mSortedMapPlaneWithMapPointCount[iter1->second] = iter1->first;

		//	std::map<CMapPoint*, CMapPlane*> mMapPointToMapPlaneCandidate;
		//	for (int j = 0; j < (int)vMapPointsOnPlane.size(); j++) {
		//		CMapPoint* pMapPoint = vMapPointsOnPlane[j];
		//		arma::vec x = pMapPoint->GetPosition();

		//		if (!pMapPoint->GetMapPlane()) {
		//			double dbDistance, dbMinDistance = INT_MAX;
		//			CMapPlane* pClosestMapPlane = nullptr;

		//			std::map<int, CMapPlane*>::iterator iter2 = mSortedMapPlaneWithMapPointCount.begin();
		//			std::map<int, CMapPlane*>::iterator end2 = mSortedMapPlaneWithMapPointCount.end();
		//			for (; iter2 != end2; iter2++) {
		//				arma::vec params = (iter2->second)->GetParameters();
		//				dbDistance = abs(params.at(0)*x.at(0) + 
		//								 params.at(1)*x.at(1) +
		//								 params.at(2)*x.at(2) +
		//								 params.at(3));

		//				if (dbDistance < dbMinDistance) {
		//					dbMinDistance = dbDistance;
		//					pClosestMapPlane = iter2->second;
		//				}
		//			}

		//			if (dbMinDistance < 5) {
		//				/*pMapPoint->SetMapPlane(pClosestMapPlane);
		//				pClosestMapPlane->AddMapPoint(pMapPoint);*/

		//				if (pClosestMapPlane)
		//					mMapPointToMapPlaneCandidate[pMapPoint] = pClosestMapPlane;
		//			}
		//		}
		//	}

		//	int nInitMapPointCountOnPlane = (int)vMapPointsOnPlane.size();
		//	std::map<CMapPoint*, CMapPlane*>::iterator iterMapPlaneCandidates = mMapPointToMapPlaneCandidate.begin();
		//	std::map<CMapPoint*, CMapPlane*>::iterator endMapPlaneCandidates = mMapPointToMapPlaneCandidate.end();
		//	for (; iterMapPlaneCandidates != endMapPlaneCandidates; iterMapPlaneCandidates++)
		//		vMapPointsOnPlane.erase(std::find(vMapPointsOnPlane.begin(), vMapPointsOnPlane.end(), iterMapPlaneCandidates->first));

		//	bool bCreatedPlane = false;
		//	CMapPlane* pNewMapPlane = nullptr;
		//	if (vMapPointsOnPlane.size() > nInitMapPointCountOnPlane * 0.5 && vMapPointsOnPlane.size() > 50) {
		//		///< create a new map plane
		//		arma::mat InitWorldPoints;

		//		for (size_t idx = 0; idx < vMapPointsOnPlane.size(); idx++)
		//			InitWorldPoints.insert_cols(InitWorldPoints.n_cols, vMapPointsOnPlane[idx]->GetPosition());

		//		CPlaneEstimator plane;
		//		plane.SetInitData(InitWorldPoints);
		//		plane.SetThreshold(1.0);
		//		plane.EstimatePlaneByRANSAC();

		//		const std::vector<int>& vPlaneInliers = plane.GetInlierDataIndices();

		//		if (vPlaneInliers.size() > InitWorldPoints.n_cols * 0.5) {
		//			arma::vec abcd(4);
		//			const arma::vec& n = plane.GetNormalVector();
		//			abcd.at(0) = n.at(0);
		//			abcd.at(1) = n.at(1);
		//			abcd.at(2) = n.at(2);
		//			abcd.at(3) = plane.GetDistance();

		//			bool bDuplicate = false;

		//			std::map<int, CMapPlane*>::iterator iter2 = mSortedMapPlaneWithMapPointCount.begin();
		//			std::map<int, CMapPlane*>::iterator end2 = mSortedMapPlaneWithMapPointCount.end();
		//			for (; iter2 != end2; iter2++) {
		//				CMapPlane* pMapPlane = iter2->second;
		//				arma::vec params = pMapPlane->GetParameters();

		//				///< compare two plane similarity
		//				double dbCosAngle = abcd.at(0)*params.at(0)+ 
		//									abcd.at(1)*params.at(1)+
		//									abcd.at(2)*params.at(2);
		//				if (acos(abs(dbCosAngle)) < 0.1 && abs(abcd.at(3)) - abs(params.at(3)) < 1) {
		//					bDuplicate = true;
		//					break;
		//				}
		//			}

		//			if (!bDuplicate) {
		//				pNewMapPlane = new CMapPlane();
		//				pNewMapPlane->SetPlaneEquation(abcd);

		//				for (size_t idx = 0; idx < vPlaneInliers.size(); idx++) {
		//					CMapPoint* pMapPoint = vMapPointsOnPlane[vPlaneInliers[idx]];
		//					pMapPoint->SetMapPlane(pNewMapPlane);
		//					pNewMapPlane->AddMapPoint(pMapPoint);
		//				}
		//			}
		//		}
		//	}

		//	if (pNewMapPlane) {
		//		pPrevKeyFrame->AddObservedMapPlane(pNewMapPlane);
		//		m_pCurrKeyFrame->AddObservedMapPlane(pNewMapPlane);

		//		std::map<CMapPoint*, CMapPlane*>::iterator iterMapPlaneCandidates = mMapPointToMapPlaneCandidate.begin();
		//		std::map<CMapPoint*, CMapPlane*>::iterator endMapPlaneCandidates = mMapPointToMapPlaneCandidate.end();

		//		arma::vec abcd1 = pNewMapPlane->GetParameters();
		//		for (; iterMapPlaneCandidates != endMapPlaneCandidates; iterMapPlaneCandidates++) {
		//			CMapPoint* pMapPoint = iterMapPlaneCandidates->first;
		//			CMapPlane* pMapPlane = iterMapPlaneCandidates->second;
		//			arma::vec x = pMapPoint->GetPosition();
		//			arma::vec abcd2 = pMapPlane->GetParameters();

		//			double dbDistance1 = abs(abcd1.at(0)*x.at(0) +
		//									 abcd1.at(1)*x.at(1) +
		//									 abcd1.at(2)*x.at(2) +
		//									 abcd1.at(3));
		//			double dbDistance2 = abs(abcd2.at(0)*x.at(0) +
		//									 abcd2.at(1)*x.at(1) +
		//									 abcd2.at(2)*x.at(2) +
		//									 abcd2.at(3));

		//			if (dbDistance1 < dbDistance2) {
		//				pNewMapPlane->AddMapPoint(pMapPoint);
		//				pMapPoint->SetMapPlane(pNewMapPlane);
		//			}
		//			else {
		//				pMapPlane->AddMapPoint(pMapPoint);
		//				pMapPoint->SetMapPlane(pMapPlane);
		//			}
		//		}

		//		pNewMapPlane->UpdateParameter();
		//		m_pWorldMap->AddMapPlane(pNewMapPlane);
		//	}
		//	else {
		//		std::map<CMapPoint*, CMapPlane*>::iterator iterMapPlaneCandidates = mMapPointToMapPlaneCandidate.begin();
		//		std::map<CMapPoint*, CMapPlane*>::iterator endMapPlaneCandidates = mMapPointToMapPlaneCandidate.end();

		//		for (; iterMapPlaneCandidates != endMapPlaneCandidates; iterMapPlaneCandidates++) {
		//			CMapPoint* pMapPoint = iterMapPlaneCandidates->first;
		//			CMapPlane* pMapPlane = iterMapPlaneCandidates->second;

		//			pMapPoint->SetMapPlane(pMapPlane);
		//			pMapPlane->AddMapPoint(pMapPoint);
		//		}
		//	}

		//	std::map<int, CMapPlane*>::iterator iter2 = mSortedMapPlaneWithMapPointCount.begin();
		//	std::map<int, CMapPlane*>::iterator end2 = mSortedMapPlaneWithMapPointCount.end();
		//	for (; iter2 != end2; iter2++) {
		//		iter2->second->UpdateParameter();
		//	}
		//}
	}

	std::set<CMapPlane*> sMapPlanes = m_pWorldMap->GetMapPlanes();

	std::set<CMapPlane*>::iterator tmp1 = sMapPlanes.begin();
	std::set<CMapPlane*>::iterator tmp2 = sMapPlanes.end();
	for (; tmp1 != tmp2; tmp1++)
		(*tmp1)->UpdateParameter();
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

void	CMapManager::FindPlanarTransformMatches(const std::vector<SiftKeyPoint>& vKeyPoints1,
												const std::vector<SiftKeyPoint>& vKeyPoints2,
												const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
												std::vector<int>& vHInliers, std::vector<int>& vHOutliers)
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

	CHomography homography;
	homography.SetInitData(SrcInitData, DstInitData);
	homography.SetThreshold(1.0);
	homography.EstimateHomographyByRANSAC();

	Concurrency::concurrent_vector<int>& inliers = homography.GetInlierDataIndices();
	Concurrency::concurrent_vector<int>& outliers = homography.GetOutlierDataIndices();

	std::sort(inliers.begin(), inliers.end());
	std::sort(outliers.begin(), outliers.end());

	vHInliers.clear();
	for (int i = 0; i < (int)inliers.size(); i++)
		vHInliers.push_back(inliers[i]);

	vHOutliers.clear();
	for (int i = 0; i < (int)outliers.size(); i++)
		vHOutliers.push_back(outliers[i]);
}

void	CMapManager::EstimateNewMapPlanes(std::vector<CMapPoint*>& vMapPointsOnPlane, arma::vec& abcd, std::vector<int>& vInliers)
{
	arma::mat InitWorldPoints;

	/*std::set<CMapPoint*>::iterator iter = sMapPointsOnPlane.begin();
	std::set<CMapPoint*>::iterator end = sMapPointsOnPlane.end();
	
	for (; iter != end; iter++)
		InitWorldPoints.insert_cols(InitWorldPoints.n_cols, (*iter)->GetPosition());*/
	for (int i = 0; i < (int)vMapPointsOnPlane.size(); i++)
		InitWorldPoints.insert_cols(InitWorldPoints.n_cols, vMapPointsOnPlane[i]->GetPosition()); 

	CPlaneEstimator estimator;
	estimator.SetInitData(InitWorldPoints);
	estimator.SetThreshold(1.0);
	estimator.EstimatePlaneByRANSAC();

	arma::vec normal = estimator.GetNormalVector();
	double dbDistance = estimator.GetDistance();

	abcd.at(0) = normal.at(0);
	abcd.at(1) = normal.at(1);
	abcd.at(2) = normal.at(2);
	abcd.at(3) = dbDistance;
	vInliers = estimator.GetInlierDataIndices();
}

void	CMapManager::ProcessTrackedMapPlanes(std::set<CMapPoint*>& sMapPointsOnPlane, std::map<CMapPlane*, int>& mMapPointCountOnPlanes)
{

}

void	CMapManager::ManageMapPlanes(void)
{

}

void	CMapManager::DrawCurrentKeyFrame(void)
{
	///< draw map points
	std::vector<double> vMapPoints;
	std::map<int, CMapPoint *> mCurrKeyFrameMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
	std::map<int, CMapPoint *>::iterator iterMapPoint = mCurrKeyFrameMapPointConnections.begin();
	std::map<int, CMapPoint *>::iterator endMapPoint = mCurrKeyFrameMapPointConnections.end();
	for (; iterMapPoint != endMapPoint; iterMapPoint++) {
		arma::vec pos = (iterMapPoint->second)->GetPosition();
		vMapPoints.push_back(pos.at(0));
		vMapPoints.push_back(pos.at(1));
		vMapPoints.push_back(pos.at(2));
	}

	if (!vMapPoints.empty()) {
		m_pWorldMapView->AddMapPoints(vMapPoints);
		m_pWorldMapView->SetCurrentKeyFrameMapPoints(vMapPoints);
	}

	///< draw keyframe pose
	arma::mat R = m_pCurrKeyFrame->GetR();
	arma::vec t = m_pCurrKeyFrame->GetT();
	arma::mat E(4, 4);
	
	R = R.t();
	t = -R * t;
	E.at(0, 0) = R.at(0, 0); E.at(0, 1) = R.at(0, 1); E.at(0, 2) = R.at(0, 2); E.at(0, 3) = t.at(0);
	E.at(1, 0) = R.at(1, 0); E.at(1, 1) = R.at(1, 1); E.at(1, 2) = R.at(1, 2); E.at(1, 3) = t.at(1);
	E.at(2, 0) = R.at(2, 0); E.at(2, 1) = R.at(2, 1); E.at(2, 2) = R.at(2, 2); E.at(2, 3) = t.at(2);
	E.at(3, 0) = E.at(3, 1) = E.at(3, 2) = 0.0; E.at(3, 3) = 1.0;

	m_pWorldMapView->AddCameraPose(E.memptr());

	///< draw ajdacent keyframe pose
	std::set<CSLAMKeyFrame*> sAdjacentKeyFrames = m_pCurrKeyFrame->GetAdjacentKeyFrames();
	std::vector<double> vAdjacentKeyFramePositions;
	if (!sAdjacentKeyFrames.empty()) {
		std::set<CSLAMKeyFrame*>::iterator iter = sAdjacentKeyFrames.begin();
		std::set<CSLAMKeyFrame*>::iterator end = sAdjacentKeyFrames.end();
		for (; iter != end; iter++) {
			arma::mat R1 = (*iter)->GetR();
			arma::vec t1 = (*iter)->GetT();
			t1 = -R1.t()*t1;

			vAdjacentKeyFramePositions.push_back(t.at(0));
			vAdjacentKeyFramePositions.push_back(t.at(1));
			vAdjacentKeyFramePositions.push_back(t.at(2));
			vAdjacentKeyFramePositions.push_back(t1.at(0));
			vAdjacentKeyFramePositions.push_back(t1.at(1));
			vAdjacentKeyFramePositions.push_back(t1.at(2));
		}
	}

	if (!vAdjacentKeyFramePositions.empty())
		m_pWorldMapView->AddKeyFramesConnectivity(vAdjacentKeyFramePositions);

	m_pWorldMapView->ClearMapPlaneContours();

	std::set<CMapPlane*> sMapPlanes = m_pWorldMap->GetMapPlanes();
	std::set<CMapPlane*>::iterator iter = sMapPlanes.begin();
	std::set<CMapPlane*>::iterator end = sMapPlanes.end();

	for (; iter != end; iter++) {
		std::vector<double> vMapPlaneContours;

		std::vector<arma::vec> vMapPlane = (*iter)->GetConvexHull();

		for (int i = 0; i < (int)vMapPlane.size(); i++) {
			arma::vec x = vMapPlane[i];

			vMapPlaneContours.push_back(x.at(0));
			vMapPlaneContours.push_back(x.at(1));
			vMapPlaneContours.push_back(x.at(2));
		}

		m_pWorldMapView->AddMapPlaneContour(vMapPlaneContours);
	}
	m_pWorldMapView->Invalidate(FALSE);
}
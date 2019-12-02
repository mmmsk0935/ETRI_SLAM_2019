#include "Tracker.h"

CTracker::CTracker(CImageGrabber* pImageGrabber, CMapManager* pMapMaker, const arma::mat& Intrinsic, const arma::vec& DistCoeff)
	: m_pImageGrabber(pImageGrabber)
	, m_pMapManager(pMapMaker)
	, K(Intrinsic)
	, D(DistCoeff)
	, m_pCurrKeyFrame(nullptr)
	, m_nCurrentFrameNumber(0)
{
	CFrame::K = K;
	CFrame::dist = D;
	cv::namedWindow("tracking state");
}

CTracker::CTracker()
{
}

CTracker::~CTracker()
{
}

void	CTracker::Track(void)
{
	CVisualSLAMConfigure& vTrackingConfigure = CVisualSLAMConfigure::GetInstance();

	while (true) {

		cv::Mat matCurrColorImage, matCurrDepthImage;

		///< read current frame from image grabber
		if (vTrackingConfigure.m_nCaptureType == VideoCaptureType::ImageSet)
			matCurrColorImage = m_pImageGrabber->GetNextColorFrame(m_nCurrentFrameNumber);

		///< if RGBD camera type, read current depth frame from imgae grabber
		if (vTrackingConfigure.m_nCameraType == CameraType::RGBD)
			matCurrDepthImage = m_pImageGrabber->GetNextDepthFrame(m_nCurrentFrameNumber);

		m_currFrame = CFrame(matCurrColorImage, matCurrDepthImage);

		///< extract blob features
		if (vTrackingConfigure.m_nPointFeatureType == PointFeatureType::SIFT_GPU)
			m_currFrame.ExtractSiftFeatures();
		
		if (vTrackingConfigure.m_nCameraType == CameraType::RGBD)
			m_currFrame.EstimateLocalScene();

		/*if (m_nCurrentState == TrackingState::FirstFrame && vTrackingConfigure.m_nCaptureType == VideoCaptureType::ImageSet)
			m_nCurrentState = TrackingState::Initializing;
		else*/ if (m_nCurrentState == TrackingState::Initializing) {
			InitializeVisualTracking();
			//if (m_nCurrentState == TrackingState::Initializing)
		}
		else if (m_nCurrentState == TrackingState::Tracking) {
			bool bTrackSuccess = false;
			if (bTrackSuccess = TrackFromPrevisouFrame()) {

			}
			else {

			}

			if (bTrackSuccess)
				CheckNewKeyFrame();
			else
				m_nCurrentState = TrackingState::TrackingLoss;
		}
		
		m_lastFrame = m_currFrame;
		m_nCurrentFrameNumber++;
		VisualizeCurrentFrame();

		char key = cv::waitKey(5);

		switch (key) {
		case ' ':
			m_nCurrentState = TrackingState::Initializing;
			break;
		case 'r':
			m_pMapManager->Clear();
			m_nCurrentState = TrackingState::FirstFrame;
			break;
		case 27:
			m_nCurrentState = TrackingState::Done;
			break;
		}

		if (m_nCurrentState == TrackingState::Done)
			break;
	}
}

bool	CTracker::TrackFromPrevisouFrame(void)
{
	///< find coarse match
	const std::vector<SiftKeyPoint>& vPrevKeyPoints = m_lastFrame.GetSiftKeyPoints();
	const std::vector<SiftKeyPoint>& vCurrKeyPoints = m_currFrame.GetSiftKeyPoints();
	const std::vector<SiftKeyPoint>& vCurrKeyFrameKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();

	const std::vector<float>& vPrevDescriptors = m_lastFrame.GetSiftDescriptors();
	const std::vector<float>& vCurrDescriptors = m_currFrame.GetSiftDescriptors();
	const std::vector<float>& vCurrKeyFrameDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();
	
	std::vector<std::pair<int, int>> vCoarseMatchPair;

	CSiftGPUWrapper::FindCoarseMatch(vPrevDescriptors, vCurrDescriptors, vCoarseMatchPair);

	std::vector<int> vFInlierIndices = FindGeometricMatchFromTwoView(vPrevKeyPoints, vCurrKeyPoints, vCoarseMatchPair);

	if (vFInlierIndices.size() < 50)
		return false;

	std::map<int, CMapPoint *>& mPrevTrackedMapPoints = m_lastFrame.GetTrackedMapPoints();
	std::vector<WorldPoint>& vCurrLocalMapPoints = m_currFrame.GetLocalMapPoints();
	std::map<int, int>& mCurrLocalMapPointConnections = m_currFrame.GetLocalMapPointConnections();
	
	/******************** pose tracking from the previsou frame ********************/
	arma::mat SrcInitImagePoints, DstInitImagePoints;
	arma::mat SrcInitWorldPoints, DstInitWorldPoints;
	std::vector<std::pair<int, int>> vGemetricMatchPair;
	for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
		if (mCurrLocalMapPointConnections.find(vCoarseMatchPair[vFInlierIndices[i]].second) == mCurrLocalMapPointConnections.end())
			continue;
		
		CMapPoint* pMapPoint = mPrevTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first];
		WorldPoint pt = vCurrLocalMapPoints[mCurrLocalMapPointConnections[vCoarseMatchPair[vFInlierIndices[i]].second]];

		if (pMapPoint) {
			const SiftKeyPoint& key1 = vPrevKeyPoints[vCoarseMatchPair[vFInlierIndices[i]].first];
			const SiftKeyPoint& key2 = vCurrKeyPoints[vCoarseMatchPair[vFInlierIndices[i]].second];
			arma::vec x1 = pMapPoint->GetPosition();
			
			arma::vec v1(3), v2(3), m1(3), m2(3);
			v1.at(0) = key1.x; v1.at(1) = key1.y; v1.at(2) = 1.0; m1.at(0) = x1.at(0); m1.at(1) = x1.at(1); m1.at(2) = x1.at(2);
			v2.at(0) = key2.x; v2.at(1) = key2.y; v2.at(2) = 1.0; m2.at(0) = pt.x; m2.at(1) = pt.y; m2.at(2) = pt.z;

			SrcInitImagePoints.insert_cols(SrcInitImagePoints.n_cols, v1);
			DstInitImagePoints.insert_cols(DstInitImagePoints.n_cols, v2);

			SrcInitWorldPoints.insert_cols(SrcInitWorldPoints.n_cols, m1);
			DstInitWorldPoints.insert_cols(DstInitWorldPoints.n_cols, m2);

			vGemetricMatchPair.push_back(vCoarseMatchPair[vFInlierIndices[i]]);
		}
	}

	///< esitmate 3D ego-motion
	CRigidBody rigid;
	rigid.SetInitData(SrcInitWorldPoints, SrcInitImagePoints, DstInitWorldPoints, DstInitImagePoints);
	rigid.SetIntrinsic(K, K);
	rigid.SetThreshold(3.0);
	rigid.EstimateRigidBodyTransformationByRANSAC();

	const std::vector<int>& vRigidInlierIndices = rigid.GetInlierDataIndices();

	if (vRigidInlierIndices.size() < 30)
		return false;

	arma::mat R = rigid.GetR();
	arma::vec t = rigid.GetT();

	m_currFrame.SetR(R);
	m_currFrame.SetT(t);

	for (int i = 0; i < (int)vRigidInlierIndices.size(); i++) {
		CMapPoint* pMapPoint = mPrevTrackedMapPoints[vGemetricMatchPair[vRigidInlierIndices[i]].first];
		int nPointFeatureIdx = vGemetricMatchPair[vRigidInlierIndices[i]].second;
		m_currFrame.AddMapPointConnection(nPointFeatureIdx, pMapPoint);
	}

	/****************** add map points from the current key frame ******************/
	int cnt = 0;
	CSiftGPUWrapper::FindCoarseMatch(vCurrKeyFrameDescriptors, vCurrDescriptors, vCoarseMatchPair);
	vFInlierIndices = FindGeometricMatchFromTwoView(vCurrKeyFrameKeyPoints, vCurrKeyPoints, vCoarseMatchPair);

	std::map<int, CMapPoint *>& mCurrKeyFrameTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
	std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
	for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
		if (mCurrTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].second) != mCurrTrackedMapPoints.end())
			continue;
		if (mCurrKeyFrameTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mCurrKeyFrameTrackedMapPoints.end())
			continue;
		
		m_currFrame.AddMapPointConnection(vCoarseMatchPair[vFInlierIndices[i]].second, mCurrKeyFrameTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first]);
		cnt++;
	}

	std::cout << "# of current map points: " << (int)mCurrKeyFrameTrackedMapPoints.size() << std::endl;
	std::cout << "# of initial correspondences: " << SrcInitImagePoints.n_cols << ", # of inlier correspondences" << (int)vRigidInlierIndices.size();
	std::cout << ", # of tracked from key-frame: " << ", " << cnt << std::endl;

	return true;
}

std::vector<int>	CTracker::FindGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1, 
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
	eight.SetK(K, K);
	eight.SetThreshold(5.0);
	eight.EstimateFundamentalMatrixByRANSAC();

	return eight.GetInlierDataIndices();	
}

void	CTracker::CheckNewKeyFrame(void)
{
	///< the number of tracking map points
	std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
	int nTrackedMapPointCount = (int)mCurrTrackedMapPoints.size();
	int nLocalMapPointCountInCurrKeyFrame = (int)m_pCurrKeyFrame->GetMapPointConnections().size();

	double dbTrackedRatio = (double)nTrackedMapPointCount / nLocalMapPointCountInCurrKeyFrame;

	std::cout << "tracking ratio: " << dbTrackedRatio << std::endl;
	if (dbTrackedRatio < 0.5) {
		///< create new key frame
		CKeyFrame* pPrevKeyFrame = m_pCurrKeyFrame;

		m_pCurrKeyFrame = new CKeyFrame(m_currFrame);
		
		/*std::vector<WorldPoint>& vLocalMapPoints = m_currFrame.GetLocalMapPoints();
		std::map<int, int>& mLocalMapPointConnections = m_currFrame.GetLocalMapPointConnections();

		mCurrTrackedMapPoints.clear();
		std::map<int, int>::iterator iter = mLocalMapPointConnections.begin();
		std::map<int, int>::iterator end = mLocalMapPointConnections.end();
		for (; iter != end; iter++) {
			CMapPoint* pNewMapPoint = new CMapPoint(vLocalMapPoints[iter->second]);
			m_pCurrKeyFrame->AddMapPointConnection(iter->first, pNewMapPoint);
			m_currFrame.AddMapPointConnection(iter->first, pNewMapPoint);
			m_pMapManager->AddMapPoint(pNewMapPoint);
		}*/
		m_pMapManager->AddKeyFrame(m_pCurrKeyFrame);
		m_pMapManager->ProcessNewKeyFrame();
		///< estimate tracked plane
		/*while (true) {
			std::vector<SiftKeyPoint>& vPrevKeyPoints = pPrevKeyFrame->GetSiftKeyPoints();
			std::vector<SiftKeyPoint>& vCurrKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();
			std::vector<float>& vPrevDescriptors = pPrevKeyFrame->GetSiftDescriptors();
			std::vector<float>& vCurrDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();
			std::map<int, CMapPoint*>& mPrevMapPointConnections = pPrevKeyFrame->GetLocalMapPointConnections();
			std::vector<std::pair<int, int>> vCoarseMatchPair;

			CSiftGPUWrapper::FindCoarseMatch(vPrevDescriptors, vCurrDescriptors, vCoarseMatchPair);

			if ((int)vCoarseMatchPair.size() < 50)
				break;
			arma::mat SrcInitDataH(3, (int)vCoarseMatchPair.size()), DstInitDataH(3, (int)vCoarseMatchPair.size());

			Concurrency::parallel_for(0, (int)vCoarseMatchPair.size(), [&](int i) {
				SrcInitDataH.at(0, i) = vPrevKeyPoints[vCoarseMatchPair[i].first].x;
				SrcInitDataH.at(1, i) = vPrevKeyPoints[vCoarseMatchPair[i].first].y;
				SrcInitDataH.at(2, i) = i;

				DstInitDataH.at(0, i) = vCurrKeyPoints[vCoarseMatchPair[i].second].x;
				DstInitDataH.at(1, i) = vCurrKeyPoints[vCoarseMatchPair[i].second].y;
				DstInitDataH.at(2, i) = i;
			});

			CHomography homography;
			homography.SetInitData(SrcInitDataH, DstInitDataH);
			homography.SetIntrinsic(K, K);
			homography.SetThreshold(3.0);
			homography.EstimateHomographyByRANSAC();

			Concurrency::concurrent_vector<int>& vHInlierIndices = homography.GetInlierDataIndices();
			Concurrency::concurrent_vector<int>& vHOutlierIndices = homography.GetOutlierDataIndices();

			if (vHInlierIndices.size() < 50)
				break;

			arma::mat InitWorldPoint(3, (int)vHInlierIndices.size());
			for (int i = 0; i < (int)vHInlierIndices.size(); i++) {
				;
			}
			if ((int)vHOutlierIndices.size() < 50)
				break;
		}*/

		std::cout << "New key-frame number: " << m_nCurrentFrameNumber << ", new map point numbers: " << (int)mCurrTrackedMapPoints.size() << std::endl;
	}
	return;
}

void	CTracker::VisualizeCurrentFrame(void)
{
	switch (m_nCurrentState) {
	case TrackingState::FirstFrame:
		cv::imshow("tracking state", m_currFrame.GetCurrFrame());
		break;
	case TrackingState::Initializing:
		cv::imshow("tracking state", m_currFrame.GetCurrFrame());
		break;
	case TrackingState::Tracking:
		{
			cv::Mat cvCurrFrame = m_currFrame.GetCurrFrame();
			const std::vector<SiftKeyPoint>& vKeyPoints = m_currFrame.GetSiftKeyPoints();
			std::map<int, CMapPoint*>& mTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
			std::map<int, CMapPoint*>::iterator iter = mTrackedMapPoints.begin();
			std::map<int, CMapPoint*>::iterator end = mTrackedMapPoints.end();

			for (; iter != end; iter++) {
				const SiftKeyPoint& key = vKeyPoints[iter->first];
				cv::circle(cvCurrFrame, cv::Point(cvRound(key.x), cvRound(key.y)), 3, cv::Scalar(0, 0, 255), 2);
			}
			cv::imshow("tracking state", cvCurrFrame);
		}
		break;
	case TrackingState::TrackingLoss:
		{
			/*cv::Mat cvCurrFrame = m_currFrame.GetCurrFrame();
			cv::Mat cvCurrKeyFrameColorImage = m_pCurrKeyFrame->GetColorImage();
			cv::Mat cvCurrKeyFrameGrayImage, cvEdgeResult;

			cv::cvtColor(cvCurrKeyFrameColorImage, cvCurrKeyFrameGrayImage, cv::COLOR_BGR2GRAY);

			cv::Canny(cvCurrKeyFrameGrayImage, cvEdgeResult, 125, 350);*/
		}
		break;
	}
}

void	CTracker::InitializeVisualTracking(void)
{
	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	if (vSLAMConfigure.m_nCameraType == CameraType::RGBD || vSLAMConfigure.m_nCameraType == CameraType::Stereo) {
		
		m_pCurrKeyFrame = new CKeyFrame(m_currFrame);

		std::vector<WorldPoint>& vLocalMapPoints = m_currFrame.GetLocalMapPoints();
		std::map<int, int>& mLocalMapPointConnections = m_currFrame.GetLocalMapPointConnections();

		std::map<int, int>::iterator iter = mLocalMapPointConnections.begin();
		std::map<int, int>::iterator end = mLocalMapPointConnections.end();
		for (; iter != end; iter++) {
			CMapPoint* pNewMapPoint = new CMapPoint(vLocalMapPoints[iter->second]);
			m_pCurrKeyFrame->AddMapPointConnection(iter->first, pNewMapPoint);
			m_currFrame.AddMapPointConnection(iter->first, pNewMapPoint);

			pNewMapPoint->AddObservedKeyFrame(m_pCurrKeyFrame, iter->first);

			m_pMapManager->AddMapPoint(pNewMapPoint);
		}

		m_pMapManager->AddKeyFrame(m_pCurrKeyFrame);
		
		m_nCurrentState = TrackingState::Tracking;
	}
	else if (vSLAMConfigure.m_nCameraType == CameraType::Monocular) {

	}
}
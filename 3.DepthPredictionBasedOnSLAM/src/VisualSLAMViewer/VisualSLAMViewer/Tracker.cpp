#include "stdafx.h"
#include "Tracker.h"

CTracker::CTracker(CImageGrabber* pImageGrabber, CMapManager* pMapMaker, CWorldMapView* pWorldMapView, const arma::mat& Intrinsic, const arma::vec& DistCoeff)
	: m_pImageGrabber(pImageGrabber)
	, m_pMapManager(pMapMaker)
	, m_pWorldMapView(pWorldMapView)
	, K(Intrinsic)
	, D(DistCoeff)
	, m_pPrevKeyFrame(nullptr)
	, m_pCurrKeyFrame(nullptr)
	, m_nCurrentFrameNumber(0)
	, m_nCurrentState(TrackingState::Idle)
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

void	CTracker::PrepaerTracking(CImageGrabber* pImageGrabber, CMapManager* pMapMaker, CWorldMapView* pWorldMapView, const arma::mat& Intrinsic, const arma::vec& DistCoeff)
{
	m_pImageGrabber = pImageGrabber;
	m_pMapManager = pMapMaker;
	m_pWorldMapView = pWorldMapView;
	K = Intrinsic;
	D = DistCoeff;
	m_pCurrKeyFrame = nullptr;
	m_nCurrentFrameNumber = 0;
	CFrame::K = K;
	CFrame::dist = D;

	cv::namedWindow("tracking state");
}

void	CTracker::Track(void)
{
	CVisualSLAMConfigure& vTrackingConfigure = CVisualSLAMConfigure::GetInstance();

	while (true) {
		int nCurrentState = GetCurrentState();

		if (nCurrentState == TrackingState::Pause)
			continue;

		cv::Mat matCurrColorImage, matCurrDepthImage;

		///< read current frame from image grabber
		if (vTrackingConfigure.m_nCaptureType == VideoCaptureType::ImageSet)
			matCurrColorImage = m_pImageGrabber->GetNextColorFrame(m_nCurrentFrameNumber);
		else if (vTrackingConfigure.m_nCaptureType == VideoCaptureType::OnTheFly)
			matCurrColorImage = m_pImageGrabber->GetNextColorFrame();

		///< if RGBD camera type, read current depth frame from imgae grabber
		if (vTrackingConfigure.m_nCameraType == CameraType::RGBD)
			matCurrDepthImage = m_pImageGrabber->GetNextDepthFrame(m_nCurrentFrameNumber);

		m_currFrame = CFrame(matCurrColorImage, matCurrDepthImage);

		///< extract blob features
		if (vTrackingConfigure.m_nPointFeatureType == PointFeatureType::SIFT_GPU)
			m_currFrame.ExtractSiftFeatures();
		
		if (vTrackingConfigure.m_nCameraType == CameraType::RGBD)
			m_currFrame.EstimateLocalScene();

		m_currFrame.ExtractLineFeatures();

		
		if (nCurrentState == TrackingState::Idle)
			m_initFrame = m_currFrame;
		else if (nCurrentState == TrackingState::Initializing) {
			if (vTrackingConfigure.m_nCameraType == CameraType::RGBD || vTrackingConfigure.m_nCameraType == CameraType::Stereo)
				InitializeForRGBD();
			else if (vTrackingConfigure.m_nCameraType == CameraType::Monocular)
				InitializingForMonocular();
		}
		else if (nCurrentState == TrackingState::Tracking) {
			bool bTrackSuccess = TrackFromPrevisouFrame();
			
			/*if (!bTrackSuccess)
				bTrackSuccess = TrackFromPreviousKeyFrame();*/
			
			if (bTrackSuccess)
				CheckNewKeyFrame();
			else
				SetCurrentState(TrackingState::TrackingLoss);
		}
		else if (nCurrentState == TrackingState::TrackingLoss) {
			bool bTrackSuccess = TrackFromCurrKeyFrame();
			if (bTrackSuccess)
				SetCurrentState(TrackingState::Tracking);
		}
		
		m_lastFrame = m_currFrame;
		m_nCurrentFrameNumber++;
		VisualizeCurrentFrame();

		char key = cv::waitKey(5);

		/*switch (key) {
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
		}*/

		if (nCurrentState == TrackingState::Done)
			break;
	}
}

bool	CTracker::TrackFromPrevisouFrame(void)
{
	arma::mat F;
	///< find coarse match
	const std::vector<SiftKeyPoint>& vPrevKeyPoints = m_lastFrame.GetSiftKeyPoints();
	const std::vector<SiftKeyPoint>& vCurrKeyPoints = m_currFrame.GetSiftKeyPoints();
	const std::vector<SiftKeyPoint>& vCurrKeyFrameKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();

	const std::vector<float>& vPrevDescriptors = m_lastFrame.GetSiftDescriptors();
	const std::vector<float>& vCurrDescriptors = m_currFrame.GetSiftDescriptors();
	const std::vector<float>& vCurrKeyFrameDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();
	
	std::vector<std::pair<int, int>> vCoarseMatchPair;

	CSiftGPUWrapper::FindCoarseMatch(vPrevDescriptors, vCurrDescriptors, vCoarseMatchPair);

	if (vCoarseMatchPair.size() < 50)
		return false;

	std::vector<int> vFInlierIndices;
	FindGeometricMatchFromTwoView(vPrevKeyPoints, vCurrKeyPoints, vCoarseMatchPair, vFInlierIndices, F);

	if (vFInlierIndices.size() < 50)
		return false;

	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	if (vSLAMConfigure.m_nCameraType == CameraType::RGBD || vSLAMConfigure.m_nCameraType == CameraType::Stereo) {
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

		for (int i = 0; i < (int)vRigidInlierIndices.size(); i++) {
			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vGemetricMatchPair[vRigidInlierIndices[i]].first];
			int nPointFeatureIdx = vGemetricMatchPair[vRigidInlierIndices[i]].second;
			m_currFrame.AddMapPointConnection(nPointFeatureIdx, pMapPoint);
		}

		/****************** add map points from the current key frame ******************/
		int cnt = 0;
		CSiftGPUWrapper::FindCoarseMatch(vCurrKeyFrameDescriptors, vCurrDescriptors, vCoarseMatchPair);

		FindGeometricMatchFromTwoView(vCurrKeyFrameKeyPoints, vCurrKeyPoints, vCoarseMatchPair, vFInlierIndices, F);

		std::map<int, CMapPoint *> mCurrKeyFrameTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
		std::map<int, CMapPoint *> mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
		for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
			if (mCurrTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].second) != mCurrTrackedMapPoints.end())
				continue;
			if (mCurrKeyFrameTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mCurrKeyFrameTrackedMapPoints.end())
				continue;

			m_currFrame.AddMapPointConnection(vCoarseMatchPair[vFInlierIndices[i]].second, mCurrKeyFrameTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first]);
			cnt++;
		}

		std::vector<double> vWorldMapPoints;
		std::map<int, CMapPoint *>::iterator iterTmp = mCurrTrackedMapPoints.begin();
		std::map<int, CMapPoint *>::iterator endTmp = mCurrTrackedMapPoints.end();
		for (; iterTmp != endTmp; iterTmp++) {
			arma::vec x = (iterTmp->second)->GetPosition();
			vWorldMapPoints.push_back(x.at(0));
			vWorldMapPoints.push_back(x.at(1));
			vWorldMapPoints.push_back(x.at(2));
		}

		R = R.t();
		t = -R * t;

		arma::mat E(4, 4);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				E.at(i, j) = R.at(i, j);
			}
			E.at(i, 3) = t.at(i);
			E.at(3, i) = 0.0;
		}
		E.at(3, 3) = 1.0;

		m_pWorldMapView->SetCurrentCameraPose(E.memptr());
		m_pWorldMapView->SetCurrentTrackedMapPoints(vWorldMapPoints);
		m_pWorldMapView->Invalidate(FALSE);

		std::cout << "# of current map points: " << (int)mCurrKeyFrameTrackedMapPoints.size() << std::endl;
		std::cout << "# of initial correspondences: " << SrcInitImagePoints.n_cols << ", # of inlier correspondences" << (int)vRigidInlierIndices.size();
		std::cout << ", # of tracked from key-frame: " << ", " << cnt << std::endl;
	}
	else {
		std::map<int, CMapPoint *>& mPrevTrackedMapPoints = m_lastFrame.GetTrackedMapPoints();
		for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
			if (mPrevTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mPrevTrackedMapPoints.end())
				continue;

			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first];

			if (pMapPoint) {
				const SiftKeyPoint& key2 = vCurrKeyPoints[vCoarseMatchPair[vFInlierIndices[i]].second];
				
				m_currFrame.AddMapPointConnection(vCoarseMatchPair[vFInlierIndices[i]].second, pMapPoint);
			}
		}

		std::map<int, CMapPoint*>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();

		if ((int)mCurrTrackedMapPoints.size() < 50)
			return false;

		std::vector<cv::line_descriptor::KeyLine>& vPrevKeyLines = m_lastFrame.GetKeyLines();
		std::vector<cv::line_descriptor::KeyLine>& vCurrKeyLines = m_currFrame.GetKeyLines();
		cv::Mat cvPrevKeyLineDescirptors = m_lastFrame.GetKeyLineDescriptors();
		cv::Mat cvCurrKeyLineDescriptors = m_currFrame.GetKeyLineDescriptors();

		std::map<int, CMapLine*> mTrackedMapLines = m_lastFrame.GetTrackedMapLines();

		std::vector<std::pair<int, int>> vKeyLineCoarseMatchPair;

		m_lsdWrapper.FindCoarseMatch(vPrevKeyLines, vCurrKeyLines, cvPrevKeyLineDescirptors, cvCurrKeyLineDescriptors,
									 vKeyLineCoarseMatchPair, vPrevKeyPoints, vCurrKeyPoints, vCoarseMatchPair);

		for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
			if (mTrackedMapLines.find(vKeyLineCoarseMatchPair[i].first) == mTrackedMapLines.end())
				continue;
			m_currFrame.AddMapLineConnection(vKeyLineCoarseMatchPair[i].second, mTrackedMapLines[vKeyLineCoarseMatchPair[i].first]);
		}
		COptimizer::OptimizePose(m_currFrame);
		/******************** pose tracking from the previsou frame ********************/
		/*arma::mat SrcInitWorldPoints, DstInitImagePoints;
		std::vector<std::pair<int, int>> vGemetricMatchPair;
		for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
			if (mPrevTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mPrevTrackedMapPoints.end())
				continue;

			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first];
			
			if (pMapPoint) {
				const SiftKeyPoint& key2 = vCurrKeyPoints[vCoarseMatchPair[vFInlierIndices[i]].second];
				arma::vec X = pMapPoint->GetPosition();

				arma::vec x(3);
				x.at(0) = key2.x; x.at(1) = key2.y; x.at(2) = 1.0;
								
				DstInitImagePoints.insert_cols(DstInitImagePoints.n_cols, x);
				SrcInitWorldPoints.insert_cols(SrcInitWorldPoints.n_cols, X);
				
				vGemetricMatchPair.push_back(vCoarseMatchPair[vFInlierIndices[i]]);
			}
		}

		if (vGemetricMatchPair.size() < 50)
			return false;

		epnp projective;
		projective.SetInitData(K, DstInitImagePoints, SrcInitWorldPoints);
		projective.SetThreshold(3.0);
		projective.EstimatePose();
		std::vector<int>& vPInliers = projective.GetInlierDataIndices();

		if ((int)vPInliers.size() < 30)
			return false;

		arma::mat& R = projective.GetR();
		arma::vec& t = projective.GetT();

		m_currFrame.SetR(R);
		m_currFrame.SetT(t);

		for (int i = 0; i < (int)vPInliers.size(); i++) {
			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vGemetricMatchPair[vPInliers[i]].first];
			int nPointFeatureIdx = vGemetricMatchPair[vPInliers[i]].second;
			m_currFrame.AddMapPointConnection(nPointFeatureIdx, pMapPoint);
		}*/

		/****************** add map points from the current key frame ******************/
		int cnt = 0;
		CSiftGPUWrapper::FindCoarseMatch(vCurrKeyFrameDescriptors, vCurrDescriptors, vCoarseMatchPair);

		std::map<int, CMapPoint *> mCurrKeyFrameTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
		//std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();

		if (vCoarseMatchPair.size() > 50) {
			FindGeometricMatchFromTwoView(vCurrKeyFrameKeyPoints, vCurrKeyPoints, vCoarseMatchPair, vFInlierIndices, F);
						
			for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
				if (mCurrTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].second) != mCurrTrackedMapPoints.end())
					continue;
				if (mCurrKeyFrameTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mCurrKeyFrameTrackedMapPoints.end())
					continue;

				m_currFrame.AddMapPointConnection(vCoarseMatchPair[vFInlierIndices[i]].second, mCurrKeyFrameTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first]);
				cnt++;
			}
		}

		std::vector<double> vWorldMapPoints;
		std::map<int, CMapPoint *>::iterator iterTmp = mCurrTrackedMapPoints.begin();
		std::map<int, CMapPoint *>::iterator endTmp = mCurrTrackedMapPoints.end();
		for (; iterTmp != endTmp; iterTmp++) {
			arma::vec x = (iterTmp->second)->GetPosition();
			vWorldMapPoints.push_back(x.at(0));
			vWorldMapPoints.push_back(x.at(1));
			vWorldMapPoints.push_back(x.at(2));
		}

		arma::mat R = m_currFrame.GetR();
		arma::vec t = m_currFrame.GetT();

		R = R.t();
		t = -R * t;

		arma::mat E(4, 4);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				E.at(i, j) = R.at(i, j);
			}
			E.at(i, 3) = t.at(i);
			E.at(3, i) = 0.0;
		}
		E.at(3, 3) = 1.0;

		m_pWorldMapView->SetCurrentCameraPose(E.memptr());
		m_pWorldMapView->SetCurrentTrackedMapPoints(vWorldMapPoints);
		m_pWorldMapView->Invalidate(FALSE);

		/*std::cout << "# of current map points: " << (int)mCurrKeyFrameTrackedMapPoints.size() << std::endl;
		std::cout << "# of initial correspondences: " << DstInitImagePoints.n_cols << ", # of inlier correspondences" << (int)vPInliers.size();
		std::cout << ", # of tracked from key-frame: " << ", " << cnt << std::endl;*/

		return true;
	}

	return true;
}

bool	CTracker::TrackFromCurrKeyFrame(void)
{
	arma::mat F;
	///< find coarse match
	const std::vector<SiftKeyPoint>& vCurrKeyPoints = m_currFrame.GetSiftKeyPoints();
	const std::vector<SiftKeyPoint>& vPrevKeyPoints = m_pCurrKeyFrame->GetSiftKeyPoints();

	const std::vector<float>& vCurrDescriptors = m_currFrame.GetSiftDescriptors();
	const std::vector<float>& vPrevDescriptors = m_pCurrKeyFrame->GetSiftDescriptors();

	std::vector<std::pair<int, int>> vCoarseMatchPair;

	CSiftGPUWrapper::FindCoarseMatch(vPrevDescriptors, vCurrDescriptors, vCoarseMatchPair);

	if (vCoarseMatchPair.size() < 50)
		return false;

	std::vector<int> vFInlierIndices;
	FindGeometricMatchFromTwoView(vPrevKeyPoints, vCurrKeyPoints, vCoarseMatchPair, vFInlierIndices, F);

	if (vFInlierIndices.size() < 50)
		return false;

	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	if (vSLAMConfigure.m_nCameraType == CameraType::RGBD || vSLAMConfigure.m_nCameraType == CameraType::Stereo) {
		std::map<int, CMapPoint *>& mPrevTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
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

		for (int i = 0; i < (int)vRigidInlierIndices.size(); i++) {
			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vGemetricMatchPair[vRigidInlierIndices[i]].first];
			int nPointFeatureIdx = vGemetricMatchPair[vRigidInlierIndices[i]].second;
			m_currFrame.AddMapPointConnection(nPointFeatureIdx, pMapPoint);
		}

		std::vector<double> vWorldMapPoints;
		std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
		std::map<int, CMapPoint *>::iterator iterTmp = mCurrTrackedMapPoints.begin();
		std::map<int, CMapPoint *>::iterator endTmp = mCurrTrackedMapPoints.end();
		for (; iterTmp != endTmp; iterTmp++) {
			arma::vec x = (iterTmp->second)->GetPosition();
			vWorldMapPoints.push_back(x.at(0));
			vWorldMapPoints.push_back(x.at(1));
			vWorldMapPoints.push_back(x.at(2));
		}

		R = R.t();
		t = -R * t;

		arma::mat E(4, 4);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				E.at(i, j) = R.at(i, j);
			}
			E.at(i, 3) = t.at(i);
			E.at(3, i) = 0.0;
		}
		E.at(3, 3) = 1.0;

		m_pWorldMapView->SetCurrentCameraPose(E.memptr());
		m_pWorldMapView->SetCurrentTrackedMapPoints(vWorldMapPoints);
		m_pWorldMapView->Invalidate(FALSE);

		std::cout << "# of current map points: " << (int)mPrevTrackedMapPoints.size() << std::endl;
		std::cout << "# of initial correspondences: " << SrcInitImagePoints.n_cols << ", # of inlier correspondences" << (int)vRigidInlierIndices.size();

		return true;
	}
	else {
		std::map<int, CMapPoint *>& mPrevTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
		int nMapPointsCount = 0;
		for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
			if (mPrevTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mPrevTrackedMapPoints.end())
				continue;

			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first];

			if (pMapPoint) {
				m_currFrame.AddMapPointConnection(vCoarseMatchPair[vFInlierIndices[i]].second, pMapPoint);
				nMapPointsCount++;
			}
		}

		if (nMapPointsCount < 10)
			return false;

		m_currFrame.SetR(m_pCurrKeyFrame->GetR());
		m_currFrame.SetT(m_pCurrKeyFrame->GetT());

		COptimizer::OptimizePose(m_currFrame);
		/******************** pose tracking from the previsou frame ********************/
		/*arma::mat SrcInitWorldPoints, DstInitImagePoints;
		std::vector<std::pair<int, int>> vGemetricMatchPair;
		for (int i = 0; i < (int)vFInlierIndices.size(); i++) {
			if (mPrevTrackedMapPoints.find(vCoarseMatchPair[vFInlierIndices[i]].first) == mPrevTrackedMapPoints.end())
				continue;

			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vCoarseMatchPair[vFInlierIndices[i]].first];

			if (pMapPoint) {
				const SiftKeyPoint& key2 = vCurrKeyPoints[vCoarseMatchPair[vFInlierIndices[i]].second];
				arma::vec X = pMapPoint->GetPosition();

				arma::vec x(3);
				x.at(0) = key2.x; x.at(1) = key2.y; x.at(2) = 1.0;

				DstInitImagePoints.insert_cols(DstInitImagePoints.n_cols, x);
				SrcInitWorldPoints.insert_cols(SrcInitWorldPoints.n_cols, X);

				vGemetricMatchPair.push_back(vCoarseMatchPair[vFInlierIndices[i]]);
			}
		}

		if (vGemetricMatchPair.size() < 50)
			return false;

		epnp projective;
		projective.SetInitData(K, DstInitImagePoints, SrcInitWorldPoints);
		projective.SetThreshold(3.0);
		projective.EstimatePose();
		std::vector<int>& vPInliers = projective.GetInlierDataIndices();

		if ((int)vPInliers.size() < 30)
			return false;

		arma::mat& R = projective.GetR();
		arma::vec& t = projective.GetT();

		m_currFrame.SetR(R);
		m_currFrame.SetT(t);

		for (int i = 0; i < (int)vPInliers.size(); i++) {
			CMapPoint* pMapPoint = mPrevTrackedMapPoints[vGemetricMatchPair[vPInliers[i]].first];
			int nPointFeatureIdx = vGemetricMatchPair[vPInliers[i]].second;
			m_currFrame.AddMapPointConnection(nPointFeatureIdx, pMapPoint);
		}*/
		
		std::vector<double> vWorldMapPoints;
		std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
		std::map<int, CMapPoint *>::iterator iterTmp = mCurrTrackedMapPoints.begin();
		std::map<int, CMapPoint *>::iterator endTmp = mCurrTrackedMapPoints.end();
		for (; iterTmp != endTmp; iterTmp++) {
			arma::vec x = (iterTmp->second)->GetPosition();
			vWorldMapPoints.push_back(x.at(0));
			vWorldMapPoints.push_back(x.at(1));
			vWorldMapPoints.push_back(x.at(2));
		}

		arma::mat R = m_currFrame.GetR();
		arma::vec t = m_currFrame.GetT();

		R = R.t();
		t = -R * t;

		arma::mat E(4, 4);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				E.at(i, j) = R.at(i, j);
			}
			E.at(i, 3) = t.at(i);
			E.at(3, i) = 0.0;
		}
		E.at(3, 3) = 1.0;

		m_pWorldMapView->SetCurrentCameraPose(E.memptr());
		m_pWorldMapView->SetCurrentTrackedMapPoints(vWorldMapPoints);
		m_pWorldMapView->Invalidate(FALSE);

		std::cout << "# of current map points: " << (int)mPrevTrackedMapPoints.size() << std::endl;
		//std::cout << "# of initial correspondences: " << DstInitImagePoints.n_cols << ", # of inlier correspondences" << (int)vPInliers.size();

		return true;
	}
}

void	CTracker::FindGeometricMatchFromTwoView(const std::vector<SiftKeyPoint>& vKeyPoints1, 
												const std::vector<SiftKeyPoint>& vKeyPoints2,
												const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
												std::vector<int>& vInlierIndices, arma::mat& F)
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

	vInlierIndices = eight.GetInlierDataIndices();
	F = eight.GetF();
}

void	CTracker::FindPlanarMatchFromView(const std::vector<SiftKeyPoint>& vKeyPoints1,
										  const std::vector<SiftKeyPoint>& vKeyPoints2,
										  const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
										  std::vector<int>& vInlierIndices, arma::mat& H)
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
	homography.SetIntrinsic(K, K);
	homography.SetThreshold(5.0);
	homography.EstimateHomographyByRANSAC();

	Concurrency::concurrent_vector<int>& inliers = homography.GetInlierDataIndices();
	vInlierIndices.resize(inliers.size());
	std::copy(inliers.begin(), inliers.end(), vInlierIndices.begin());
	
	H = homography.GetHomography();
}

void	CTracker::CheckNewKeyFrame(void)
{
	///< the number of tracking map points
	std::map<int, CMapPoint *>& mCurrTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
	int nTrackedMapPointCount = (int)mCurrTrackedMapPoints.size();
	int nLocalMapPointCountInCurrKeyFrame = (int)m_pCurrKeyFrame->GetMapPointConnections().size();

	double dbTrackedRatio = (double)nTrackedMapPointCount / nLocalMapPointCountInCurrKeyFrame;

	std::cout << "tracking ratio: " << dbTrackedRatio << std::endl;
	CVisualSLAMConfigure& configure = CVisualSLAMConfigure::GetInstance();
	if ((configure.m_nCameraType == CameraType::RGBD || configure.m_nCameraType == CameraType::Stereo) && dbTrackedRatio < 0.3) {
		///< create new key frame
		CSLAMKeyFrame* pPrevKeyFrame = m_pCurrKeyFrame;

		m_pPrevKeyFrame = m_pCurrKeyFrame;
		m_pCurrKeyFrame = new CSLAMKeyFrame(m_currFrame);
		
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
		//m_pMapManager->ProcessNewKeyFrame();
		//m_pMapManager->ProcessMapPlanes();
		//m_pMapManager->ProcessMapLines();

		/*std::map<int, CMapPoint *>& mNewMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
		std::map<int, CMapPoint *>::iterator iterTmp = mNewMapPointConnections.begin();
		std::map<int, CMapPoint *>::iterator endTmp = mNewMapPointConnections.end();
		std::vector<double> vNewMapPoints;
		std::vector<double> vNewMapPlanes;
		std::vector<double> vNewMapPlaneColors;
		for (; iterTmp != endTmp; iterTmp++) {
			arma::vec x = iterTmp->second->GetPosition();
			if (((iterTmp->second)->GetMapPlane())) {
				cv::Scalar color = iterTmp->second->GetMapPlane()->GetColor();

				vNewMapPlanes.push_back(x.at(0));
				vNewMapPlanes.push_back(x.at(1));
				vNewMapPlanes.push_back(x.at(2));
				vNewMapPlaneColors.push_back(color[0] / 255.0);
				vNewMapPlaneColors.push_back(color[1] / 255.0);
				vNewMapPlaneColors.push_back(color[2] / 255.0);
			}
			else {
				vNewMapPoints.push_back(x.at(0));
				vNewMapPoints.push_back(x.at(1));
				vNewMapPoints.push_back(x.at(2));
			}
		}*/

		/*std::map<int, CMapLine*>& mNewMapLineConnections = m_pCurrKeyFrame->GetMapLineConnections();
		std::map<int, CMapLine*>::iterator iterBuff = mNewMapLineConnections.begin();
		std::map<int, CMapLine*>::iterator endBuff = mNewMapLineConnections.end();
		std::vector<double> vNewMapLines;
		for (; iterBuff != endBuff; iterBuff++) {
			arma::vec x1(3), x2(3);

			iterBuff->second->GetPosition(x1, x2);
			vNewMapLines.push_back(x1.at(0));
			vNewMapLines.push_back(x1.at(1));
			vNewMapLines.push_back(x1.at(2));
			vNewMapLines.push_back(x2.at(0));
			vNewMapLines.push_back(x2.at(1));
			vNewMapLines.push_back(x2.at(2));
		}
		m_pWorldMapView->AddMapLines(vNewMapLines);*/

		/*std::set<CMapPlane*> sMapPlanes = m_pMapManager->GetMapPlanes();
		std::set<CMapPlane*>::iterator tmp1 = sMapPlanes.begin();
		std::set<CMapPlane*>::iterator tmp2 = sMapPlanes.end();
		for (; tmp1 != tmp2; tmp1++) {
			std::vector<double> vContourTmp;
			std::vector<arma::vec> vContours = (*tmp1)->GetConvexHull();

			for (int k = 0; k < (int)vContours.size(); k++) {
				vContourTmp.push_back(vContours[k].at(0));
				vContourTmp.push_back(vContours[k].at(1));
				vContourTmp.push_back(vContours[k].at(2));
			}
			m_pWorldMapView->AddMapPlaneContour(vContourTmp);
		}
		m_pWorldMapView->AddMapPoints(vNewMapPoints);
		m_pWorldMapView->AddMapPlanes(vNewMapPlanes, vNewMapPlaneColors);
		m_pWorldMapView->SetCurrentKeyFrameMapPoints(vNewMapPoints);
		m_pWorldMapView->Invalidate(FALSE);*/
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
	else if (configure.m_nCameraType == CameraType::Monocular) {
		if (dbTrackedRatio < 0.4 && (int)mCurrTrackedMapPoints.size() > 30) {
			///< neek new key-frame
			m_pPrevKeyFrame = m_pCurrKeyFrame;
			m_pCurrKeyFrame = new CSLAMKeyFrame(m_currFrame);
			m_pMapManager->AddKeyFrame(m_pCurrKeyFrame);
			m_pMapManager->ProcessMapPoints();
			m_pMapManager->ProcessMapPlanes();
			m_pMapManager->DrawCurrentKeyFrame();
		}
	}
	return;
}

void	CTracker::VisualizeCurrentFrame(void)
{
	cv::Mat img = m_currFrame.GetCurrFrame();

	switch (GetCurrentState()) {
	case TrackingState::Idle:
		cv::imshow("tracking state", img);
		break;
	case TrackingState::Initializing:
		{
			for (int i = 0; i < (int)m_vTrackForInitialization.size(); i++) {
				const SiftKeyPoint& key1 = m_vTrackForInitialization[i].first;
				const SiftKeyPoint& key2 = m_vTrackForInitialization[i].second;

				cv::circle(img, cv::Point(cvRound(key1.x), cvRound(key1.y)), 3, cv::Scalar(0, 0, 255), 2);
				cv::circle(img, cv::Point(cvRound(key2.x), cvRound(key2.y)), 3, cv::Scalar(0, 0, 255), 2);
				cv::line(img, cv::Point(cvRound(key1.x), cvRound(key1.y)), cv::Point(cvRound(key2.x), cvRound(key2.y)), cv::Scalar(0, 255, 0), 2);
			}
			cv::imshow("tracking state", img);
		}
		
		break;
	case TrackingState::Tracking:
		{
			const std::vector<SiftKeyPoint>& vKeyPoints = m_currFrame.GetSiftKeyPoints();
			std::map<int, CMapPoint*>& mTrackedMapPoints = m_currFrame.GetTrackedMapPoints();
			std::map<int, CMapPoint*>::iterator iter = mTrackedMapPoints.begin();
			std::map<int, CMapPoint*>::iterator end = mTrackedMapPoints.end();

			for (; iter != end; iter++) {
				const SiftKeyPoint& key = vKeyPoints[iter->first];

				if ((iter->second)->GetMapPlane())
					cv::circle(img, cv::Point(cvRound(key.x), cvRound(key.y)), 3, (iter->second)->GetMapPlane()->GetColor(), 2);
				else
					cv::circle(img, cv::Point(cvRound(key.x), cvRound(key.y)), 3, cv::Scalar(0, 0, 255), 2);
			}
			cv::imshow("tracking state", img);
		}
		break;
	case TrackingState::TrackingLoss:
		{
			cv::Mat img2 = m_pCurrKeyFrame->GetColorImage();
			cv::Mat edge, tmp;
			cv::Canny(img2, edge, 100, 127, 3, false);
			cv::cvtColor(edge, tmp, cv::COLOR_GRAY2BGR);
			cv::addWeighted(tmp, 0.5, img, 0.5, 0, img2);
			cv::imshow("tracking state", img2);
			/*cv::Mat cvCurrFrame = m_currFrame.GetCurrFrame();
			cv::Mat cvCurrKeyFrameColorImage = m_pCurrKeyFrame->GetColorImage();
			cv::Mat cvCurrKeyFrameGrayImage, cvEdgeResult;

			cv::cvtColor(cvCurrKeyFrameColorImage, cvCurrKeyFrameGrayImage, cv::COLOR_BGR2GRAY);

			cv::Canny(cvCurrKeyFrameGrayImage, cvEdgeResult, 125, 350);*/
		}
		break;
	}
}

void	CTracker::InitializeForRGBD(void)
{
	CSLAMKeyFrame::K = CFrame::K;
	CSLAMKeyFrame::m_dbMinX = CFrame::m_dbMinX;
	CSLAMKeyFrame::m_dbMaxX = CFrame::m_dbMaxX;
	CSLAMKeyFrame::m_dbMinY = CFrame::m_dbMinY;
	CSLAMKeyFrame::m_dbMaxY = CFrame::m_dbMaxY;
	CSLAMKeyFrame::m_dbInvGridColSize = CFrame::m_dbInvGridColSize;
	CSLAMKeyFrame::m_dbInvGridRowSize = CFrame::m_dbInvGridRowSize;

	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	if (vSLAMConfigure.m_nCameraType == CameraType::RGBD || vSLAMConfigure.m_nCameraType == CameraType::Stereo) {
		
		m_pPrevKeyFrame = m_pCurrKeyFrame;
		m_pCurrKeyFrame = new CSLAMKeyFrame(m_currFrame);

		std::vector<WorldPoint>& vLocalMapPoints = m_currFrame.GetLocalMapPoints();
		std::map<int, int>& mLocalMapPointConnections = m_currFrame.GetLocalMapPointConnections();

		std::map<int, int>::iterator iter = mLocalMapPointConnections.begin();
		std::map<int, int>::iterator end = mLocalMapPointConnections.end();
		std::vector<double> vNewMapPointsPosition;
		std::vector<CMapPoint *> vNewMapPoints;
		for (; iter != end; iter++) {
			CMapPoint* pNewMapPoint = new CMapPoint(vLocalMapPoints[iter->second]);
			m_pCurrKeyFrame->AddMapPointConnection(iter->first, pNewMapPoint);
			m_currFrame.AddMapPointConnection(iter->first, pNewMapPoint);

			pNewMapPoint->AddObservedKeyFrame(m_pCurrKeyFrame, iter->first);

			m_pMapManager->AddMapPoint(pNewMapPoint);

			vNewMapPointsPosition.push_back(vLocalMapPoints[iter->second].x);
			vNewMapPointsPosition.push_back(vLocalMapPoints[iter->second].y);
			vNewMapPointsPosition.push_back(vLocalMapPoints[iter->second].z);

			vNewMapPoints.push_back(pNewMapPoint);
		}

		/*std::vector<CMapPlane*> vNewMapPlane = EstimateInitialMapPlane(vNewMapPoints);

		std::vector<double> vMapPlane, vMapPlaneColor;
		for (int i = 0; i < (int)vNewMapPlane.size(); i++) {
			std::vector<CMapPoint *> vMapPoints = vNewMapPlane[i]->GetMapPoints();
			cv::Scalar color = vNewMapPlane[i]->GetColor();
			double pColor[3] = { color[0] / 255.0, color[1] / 255.0, color[2] / 255.0 };

			for (int j = 0; j < (int)vMapPoints.size(); j++) {
				arma::vec x = vMapPoints[j]->GetPosition();
				vMapPlane.push_back(x.at(0));
				vMapPlane.push_back(x.at(1));
				vMapPlane.push_back(x.at(2));
				vMapPlaneColor.push_back(pColor[0]);
				vMapPlaneColor.push_back(pColor[1]);
				vMapPlaneColor.push_back(pColor[2]);
			}
		}

		m_pWorldMapView->AddMapPlanes(vMapPlane, vMapPlaneColor);*/
		arma::mat R = m_pCurrKeyFrame->GetR();
		arma::vec t = -R * m_pCurrKeyFrame->GetT();

		arma::mat E(4, 4);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				E.at(i, j) = R.at(i, j);
			}
			E.at(i, 3) = t.at(i);
			E.at(3, i) = 0.0;
		}
		E.at(3, 3) = 1.0;

		m_pWorldMapView->AddCameraPose(E.memptr());

		m_pMapManager->AddInitKeyFrame(m_pCurrKeyFrame);
		m_pWorldMapView->AddMapPoints(vNewMapPointsPosition);
		SetCurrentState(TrackingState::Tracking);
	}
	else if (vSLAMConfigure.m_nCameraType == CameraType::Monocular) {

	}
}

void	CTracker::InitializingForMonocular(void)
{
	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_initFrame.GetSiftKeyPoints();
	std::vector<SiftKeyPoint>& vDstKeyPoints = m_currFrame.GetSiftKeyPoints();
	std::vector<float>& vSrcDescriptors = m_initFrame.GetSiftDescriptors();
	std::vector<float>& vDstDescriptors = m_currFrame.GetSiftDescriptors();
	std::vector<std::pair<int, int>> vCoarseMatchPair;
	std::vector<int> vFInliers, vHInliers;
	arma::mat F, H;
	
	CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vCoarseMatchPair);

	m_vTrackForInitialization.resize(vCoarseMatchPair.size());
	for (int i = 0; i < (int)vCoarseMatchPair.size(); i++)
		m_vTrackForInitialization[i] = std::pair<SiftKeyPoint, SiftKeyPoint>(vSrcKeyPoints[vCoarseMatchPair[i].first], vDstKeyPoints[vCoarseMatchPair[i].second]);

	if (vCoarseMatchPair.size() < 50) {
		SetCurrentState(TrackingState::Idle);
		return;
	}

	boost::thread thF, thH;

	thF = boost::thread(&CTracker::FindGeometricMatchFromTwoView, this,
						boost::ref(vSrcKeyPoints), boost::ref(vDstKeyPoints), boost::ref(vCoarseMatchPair), boost::ref(vFInliers), boost::ref(F));
	thH = boost::thread(&CTracker::FindPlanarMatchFromView, this,
						boost::ref(vSrcKeyPoints), boost::ref(vDstKeyPoints), boost::ref(vCoarseMatchPair), boost::ref(vHInliers), boost::ref(H));

	thF.join();
	thH.join();

	if (vHInliers.size() > vCoarseMatchPair.size() * 0.8 || vFInliers.size() > vCoarseMatchPair.size() * 0.95)
		return;

	double dbScore = (double)vHInliers.size() / (vFInliers.size() + vHInliers.size());

	CSLAMKeyFrame::K = CFrame::K;
	CSLAMKeyFrame::m_dbMinX = CFrame::m_dbMinX;
	CSLAMKeyFrame::m_dbMaxX = CFrame::m_dbMaxX;
	CSLAMKeyFrame::m_dbMinY = CFrame::m_dbMinY;
	CSLAMKeyFrame::m_dbMaxY = CFrame::m_dbMaxY;
	CSLAMKeyFrame::m_dbInvGridColSize = CFrame::m_dbInvGridColSize;
	CSLAMKeyFrame::m_dbInvGridRowSize = CFrame::m_dbInvGridRowSize;

	bool bSuccess = false;
	if (dbScore > 0.4) {
		if (vHInliers.size() < vCoarseMatchPair.size() * 0.5)
			return;

		///< initialize map for tracking from homography
		bSuccess = ReconstructTwoViewFromH(H, vCoarseMatchPair, vHInliers);
	}
	else {
		///< initialize from homography
		if (vFInliers.size() < vCoarseMatchPair.size() * 0.5)
			return;

		bSuccess = ReconstructTwoViewFromF(F, vCoarseMatchPair, vFInliers);
	}

	if (bSuccess) {
		///< mean depth to sqrt(2)
		std::map<int, CMapPoint*> mCurrTrackedMapPoints = m_pCurrKeyFrame->GetMapPointConnections();
		std::map<int, CMapPoint*>::iterator iterMapPoints = mCurrTrackedMapPoints.begin();
		std::map<int, CMapPoint*>::iterator endMapPoints = mCurrTrackedMapPoints.end();

		double dbMeanDepth = 0;
		for (; iterMapPoints != endMapPoints; iterMapPoints++) {
			arma::vec x = (iterMapPoints->second)->GetPosition();
			dbMeanDepth += x.at(2);
		}
		dbMeanDepth /= (int)mCurrTrackedMapPoints.size();
		double dbScaleFactor = sqrt(2) / dbMeanDepth;

		for (iterMapPoints = mCurrTrackedMapPoints.begin(); iterMapPoints != endMapPoints; iterMapPoints++)
			(iterMapPoints->second)->SetPosition((iterMapPoints->second)->GetPosition()*dbScaleFactor);

		m_pCurrKeyFrame->SetT(m_pCurrKeyFrame->GetT()*dbScaleFactor);
		m_currFrame.SetT(m_currFrame.GetT()*dbScaleFactor);

		if (vHInliers.size() > 100) {
			///< create the initial map plane
			std::map<int, CMapPoint*> mMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
			std::vector<CMapPoint*> vMapPlaneCandidates;
			arma::mat InitMapPlane;
			for (int i = 0; i < (int)vHInliers.size(); i++) {
				if (mMapPointConnections.find(vCoarseMatchPair[vHInliers[i]].second) == mMapPointConnections.end())
					continue;
				CMapPoint* pMapPoint = mMapPointConnections[vCoarseMatchPair[vHInliers[i]].second];
				vMapPlaneCandidates.push_back(pMapPoint);

				InitMapPlane.insert_cols(InitMapPlane.n_cols, pMapPoint->GetPosition());
			}

			CPlaneEstimator plane;
			plane.SetInitData(InitMapPlane);
			plane.SetThreshold(2);
			plane.EstimatePlaneByRANSAC();

			const arma::vec& normal = plane.GetNormalVector();
			double dbDistance = plane.GetDistance();

			const std::vector<int>& vPlaneInliers = plane.GetInlierDataIndices();

			if (vPlaneInliers.size() > vMapPlaneCandidates.size() * 0.8) {
				CMapPlane* pNewMapPlane = new CMapPlane();
				for (int i = 0; i < (int)vPlaneInliers.size(); i++) {
					pNewMapPlane->AddMapPoint(vMapPlaneCandidates[vPlaneInliers[i]]);
					vMapPlaneCandidates[vPlaneInliers[i]]->SetMapPlane(pNewMapPlane);
				}
				arma::vec abcd(4);
				abcd.at(0) = normal.at(0);
				abcd.at(1) = normal.at(1);
				abcd.at(2) = normal.at(2);
				abcd.at(3) = dbDistance;

				pNewMapPlane->SetPlaneEquation(abcd);
				pNewMapPlane->UpdateParameter();

				std::vector<arma::vec>& vMapPlaneContour = pNewMapPlane->GetConvexHull();
				std::vector<double> vMapPlaneBB;

				for (int i = 0; i < (int)vMapPlaneContour.size(); i++) {
					vMapPlaneBB.push_back(vMapPlaneContour[i].at(0));
					vMapPlaneBB.push_back(vMapPlaneContour[i].at(1));
					vMapPlaneBB.push_back(vMapPlaneContour[i].at(2));
				}

				m_pPrevKeyFrame->AddObservedMapPlane(pNewMapPlane);
				m_pCurrKeyFrame->AddObservedMapPlane(pNewMapPlane);

				m_currFrame.AddObservedMapPlanes(pNewMapPlane);

				m_pMapManager->AddMapPlane(pNewMapPlane);

				m_pWorldMapView->AddMapPlaneContour(vMapPlaneBB);

				///< create a new map line on the map plane
				std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines = m_pPrevKeyFrame->GetKeyLines();
				std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines = m_pCurrKeyFrame->GetKeyLines();
				cv::Mat cvSrcKeyLineDescriptors = m_pPrevKeyFrame->GetKeyLineDescriptors();
				cv::Mat cvDstKeyLineDescriptors = m_pCurrKeyFrame->GetKeyLineDescriptors();
				std::vector<std::pair<int, int>> vKeyLineCoarseMatchPair;

				m_lsdWrapper.FindCoarseMatch(vSrcKeyLines, vDstKeyLines, cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vKeyLineCoarseMatchPair,
											 vSrcKeyPoints, vDstKeyPoints, vCoarseMatchPair);

				std::vector<double> vNewMapLines;
				arma::mat R1 = m_pCurrKeyFrame->GetR();
				arma::vec t1 = m_pCurrKeyFrame->GetT();

				R1 = R1.t();
				t1 = -R1 * t1;
				for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
					const cv::line_descriptor::KeyLine& cvKeyLine1 = vSrcKeyLines[vKeyLineCoarseMatchPair[i].first];
					const cv::line_descriptor::KeyLine& cvKeyLine2 = vDstKeyLines[vKeyLineCoarseMatchPair[i].second];

					double a1, b1, c1;
					double a2, b2, c2;

					a1 = cvKeyLine1.startPointY - cvKeyLine1.endPointY;
					b1 = cvKeyLine1.endPointX - cvKeyLine1.startPointX;
					c1 = cvKeyLine1.startPointX*cvKeyLine1.endPointY - cvKeyLine1.startPointY*cvKeyLine1.endPointX;

					a2 = cvKeyLine2.startPointY - cvKeyLine2.endPointY;
					b2 = cvKeyLine2.endPointX - cvKeyLine2.startPointX;
					c2 = cvKeyLine2.startPointX*cvKeyLine2.endPointY - cvKeyLine2.startPointY*cvKeyLine2.endPointX;

					///< line correspondence with homography H, x' = Hx
					///< H^T * l' = l (l^T x = 0)
					double a11 = H.at(0, 0)*a2 + H.at(1, 0)*b2 + H.at(2, 0)*c2;
					double b11 = H.at(0, 1)*a2 + H.at(1, 1)*b2 + H.at(2, 1)*c2;
					double c11 = H.at(0, 2)*a2 + H.at(1, 2)*b2 + H.at(2, 2)*c2;

					a1 /= c1; b1 /= c1;
					a11 /= c11; b11 /= c11;

					if (abs(a1*b11 - a11 * b1) < 0.00001) {
						arma::vec x[4]; double t[4];

						for (int k = 0; k < 4; k++)
							x[k].set_size(3);

						x[0].at(0) = (cvKeyLine1.startPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
						x[0].at(1) = (cvKeyLine1.startPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
						x[0].at(2) = 1.0;

						x[1].at(0) = (cvKeyLine1.endPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
						x[1].at(1) = (cvKeyLine1.endPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
						x[1].at(2) = 1.0;

						x[2].at(0) = (cvKeyLine2.startPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
						x[2].at(1) = (cvKeyLine2.startPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
						x[2].at(2) = 1.0;

						x[3].at(0) = (cvKeyLine2.endPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
						x[3].at(1) = (cvKeyLine2.endPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
						x[3].at(2) = 1.0;

						x[2] = R1 * x[2];
						x[3] = R1 * x[3];

						t[0] = -abcd.at(3) / (abcd.at(0)*x[0].at(0) + abcd.at(1)*x[0].at(1) + abcd.at(2)*x[0].at(2));
						t[1] = -abcd.at(3) / (abcd.at(0)*x[1].at(0) + abcd.at(1)*x[1].at(1) + abcd.at(2)*x[1].at(2));
						t[2] = -(abcd.at(3) + abcd.at(0)*t1.at(0) + abcd.at(1)*t1.at(1) + abcd.at(2)*t1.at(2)) /
							(abcd.at(0)*x[2].at(0) + abcd.at(1)*x[2].at(1) + abcd.at(2)*x[2].at(2));
						t[3] = -(abcd.at(3) + abcd.at(0)*t1.at(0) + abcd.at(1)*t1.at(1) + abcd.at(2)*t1.at(2)) /
							(abcd.at(0)*x[3].at(0) + abcd.at(1)*x[3].at(1) + abcd.at(2)*x[3].at(2));

						x[0] *= t[0]; x[1] *= t[1];
						x[2] = t1 + x[2] * t[2]; x[3] = t1 + x[3] * t[3];

						int nIdx1 = 0, nIdx2 = 1;
						double dbMaxDist = arma::norm(x[0] - x[1]);
						for (int i = 0; i < 3; i++) {
							for (int j = i + 1; j < 4; j++) {
								if (arma::norm(x[i] - x[j]) > dbMaxDist) {
									nIdx1 = i;
									nIdx2 = j;
								}
							}
						}

						arma::vec dir = x[nIdx2] - x[nIdx1];
						dir /= arma::norm(dir);
						CMapLine* pNewMapLine = new CMapLine(x[nIdx1], x[nIdx2], dir);
						pNewMapLine->AddObservedKeyFrame(m_pPrevKeyFrame, vKeyLineCoarseMatchPair[i].first);
						pNewMapLine->AddObservedKeyFrame(m_pCurrKeyFrame, vKeyLineCoarseMatchPair[i].second);

						m_pPrevKeyFrame->AddMapLineConnection(vKeyLineCoarseMatchPair[i].first, pNewMapLine);
						m_pCurrKeyFrame->AddMapLineConnection(vKeyLineCoarseMatchPair[i].second, pNewMapLine);

						m_currFrame.AddMapLineConnection(vKeyLineCoarseMatchPair[i].second, pNewMapLine);

						m_pMapManager->AddMapLine(pNewMapLine);

						pNewMapPlane->AddMapLine(pNewMapLine);

						vNewMapLines.push_back(x[nIdx1].at(0));
						vNewMapLines.push_back(x[nIdx1].at(1));
						vNewMapLines.push_back(x[nIdx1].at(2));
						vNewMapLines.push_back(x[nIdx2].at(0));
						vNewMapLines.push_back(x[nIdx2].at(1));
						vNewMapLines.push_back(x[nIdx2].at(2));
					}

					m_pWorldMapView->AddMapLines(vNewMapLines);
				}
			}
		}
	}
	/*if (bSuccess && vHInliers.size() > 100) {
		///< create the initial map plane
		std::map<int, CMapPoint*> mMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
		std::vector<CMapPoint*> vMapPlaneCandidates;
		arma::mat InitMapPlane;
		for (int i = 0; i < (int)vHInliers.size(); i++) {
			if (mMapPointConnections.find(vCoarseMatchPair[vHInliers[i]].second) == mMapPointConnections.end())
				continue;
			CMapPoint* pMapPoint = mMapPointConnections[vCoarseMatchPair[vHInliers[i]].second];
			vMapPlaneCandidates.push_back(pMapPoint);

			InitMapPlane.insert_cols(InitMapPlane.n_cols, pMapPoint->GetPosition());
		}

		CPlaneEstimator plane;
		plane.SetInitData(InitMapPlane);
		plane.SetThreshold(3);
		plane.EstimatePlaneByRANSAC();

		const arma::vec& normal = plane.GetNormalVector();
		double dbDistance = plane.GetDistance();

		const std::vector<int>& vPlaneInliers = plane.GetInlierDataIndices();

		if (vPlaneInliers.size() > vMapPlaneCandidates.size() * 0.8) {
			CMapPlane* pNewMapPlane = new CMapPlane();
			for (int i = 0; i < (int)vPlaneInliers.size(); i++) {
				pNewMapPlane->AddMapPoint(vMapPlaneCandidates[vPlaneInliers[i]]);
				vMapPlaneCandidates[vPlaneInliers[i]]->SetMapPlane(pNewMapPlane);
			}
			arma::vec abcd(4);
			abcd.at(0) = normal.at(0);
			abcd.at(1) = normal.at(1);
			abcd.at(2) = normal.at(2);
			abcd.at(3) = dbDistance;

			pNewMapPlane->SetPlaneEquation(abcd);
			pNewMapPlane->UpdateParameter();

			std::vector<arma::vec>& vMapPlaneContour = pNewMapPlane->GetConvexHull();
			std::vector<double> vMapPlaneBB;

			for (int i = 0; i < (int)vMapPlaneContour.size(); i++) {
				vMapPlaneBB.push_back(vMapPlaneContour[i].at(0));
				vMapPlaneBB.push_back(vMapPlaneContour[i].at(1));
				vMapPlaneBB.push_back(vMapPlaneContour[i].at(2));
			}

			m_pPrevKeyFrame->AddObservedMapPlane(pNewMapPlane);
			m_pCurrKeyFrame->AddObservedMapPlane(pNewMapPlane);

			m_currFrame.AddObservedMapPlanes(pNewMapPlane);

			m_pMapManager->AddMapPlane(pNewMapPlane);

			m_pWorldMapView->AddMapPlaneContour(vMapPlaneBB);		

			///< create a new map line on the map plane
			std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines = m_pPrevKeyFrame->GetKeyLines();
			std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines = m_pCurrKeyFrame->GetKeyLines();
			cv::Mat cvSrcKeyLineDescriptors = m_pPrevKeyFrame->GetKeyLineDescriptors();
			cv::Mat cvDstKeyLineDescriptors = m_pCurrKeyFrame->GetKeyLineDescriptors();
			std::vector<std::pair<int, int>> vKeyLineCoarseMatchPair;

			CLineSegmentWrapper::FindCoarseMatch(vSrcKeyLines, vDstKeyLines, cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vKeyLineCoarseMatchPair,
				vSrcKeyPoints, vDstKeyPoints, vCoarseMatchPair);


			std::vector<double> vNewMapLines;
			arma::mat R1 = m_pCurrKeyFrame->GetR();
			arma::vec t1 = m_pCurrKeyFrame->GetT();

			R1 = R1.t();
			t1 = -R1 * t1;
			for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
				const cv::line_descriptor::KeyLine& cvKeyLine1 = vSrcKeyLines[vKeyLineCoarseMatchPair[i].first];
				const cv::line_descriptor::KeyLine& cvKeyLine2 = vDstKeyLines[vKeyLineCoarseMatchPair[i].second];

				double a1, b1, c1;
				double a2, b2, c2;

				a1 = cvKeyLine1.startPointY - cvKeyLine1.endPointY;
				b1 = cvKeyLine1.endPointX - cvKeyLine1.startPointX;
				c1 = cvKeyLine1.startPointX*cvKeyLine1.endPointY - cvKeyLine1.startPointY*cvKeyLine1.endPointX;

				a2 = cvKeyLine2.startPointY - cvKeyLine2.endPointY;
				b2 = cvKeyLine2.endPointX - cvKeyLine2.startPointX;
				c2 = cvKeyLine2.startPointX*cvKeyLine2.endPointY - cvKeyLine2.startPointY*cvKeyLine2.endPointX;

				///< line correspondence with homography H, x' = Hx
				///< H^T * l' = l (l^T x = 0)
				double a11 = H.at(0, 0)*a2 + H.at(1, 0)*b2 + H.at(2, 0)*c2;
				double b11 = H.at(0, 1)*a2 + H.at(1, 1)*b2 + H.at(2, 1)*c2;
				double c11 = H.at(0, 2)*a2 + H.at(1, 2)*b2 + H.at(2, 2)*c2;

				a1 /= c1; b1 /= c1;
				a11 /= c11; b11 /= c11;

				if (abs(a1*b11 - a11 * b1) < 0.00001) {
					arma::vec x[4]; double t[4];

					for (int k = 0; k < 4; k++)
						x[k].set_size(3);

					x[0].at(0) = (cvKeyLine1.startPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
					x[0].at(1) = (cvKeyLine1.startPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
					x[0].at(2) = 1.0;

					x[1].at(0) = (cvKeyLine1.endPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
					x[1].at(1) = (cvKeyLine1.endPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
					x[1].at(2) = 1.0;

					x[2].at(0) = (cvKeyLine2.startPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
					x[2].at(1) = (cvKeyLine2.startPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
					x[2].at(2) = 1.0;

					x[3].at(0) = (cvKeyLine2.endPointX - CSLAMKeyFrame::K.at(0, 2)) / CSLAMKeyFrame::K.at(0, 0);
					x[3].at(1) = (cvKeyLine2.endPointY - CSLAMKeyFrame::K.at(1, 2)) / CSLAMKeyFrame::K.at(1, 1);
					x[3].at(2) = 1.0;

					x[2] = R1 * x[2];
					x[3] = R1 * x[3];

					t[0] = -abcd.at(3) / (abcd.at(0)*x[0].at(0) + abcd.at(1)*x[0].at(1) + abcd.at(2)*x[0].at(2));
					t[1] = -abcd.at(3) / (abcd.at(0)*x[1].at(0) + abcd.at(1)*x[1].at(1) + abcd.at(2)*x[1].at(2));
					t[2] = -(abcd.at(3) + abcd.at(0)*t1.at(0) + abcd.at(1)*t1.at(1) + abcd.at(2)*t1.at(2)) / 
										 (abcd.at(0)*x[2].at(0) + abcd.at(1)*x[2].at(1) + abcd.at(2)*x[2].at(2));
					t[3] = -(abcd.at(3) + abcd.at(0)*t1.at(0) + abcd.at(1)*t1.at(1) + abcd.at(2)*t1.at(2)) / 
										 (abcd.at(0)*x[3].at(0) + abcd.at(1)*x[3].at(1) + abcd.at(2)*x[3].at(2));

					x[0] *= t[0]; x[1] *= t[1]; 
					x[2] = t1 + x[2] * t[2]; x[3] = t1 + x[3] * t[3];

					int nIdx1 = 0, nIdx2 = 1;
					double dbMaxDist = arma::norm(x[0] - x[1]);
					for (int i = 0; i < 3; i++) {
						for (int j = i + 1; j < 4; j++) {
							if (arma::norm(x[i] - x[j]) > dbMaxDist) {
								nIdx1 = i;
								nIdx2 = j;
							}
						}
					}

					arma::vec dir = x[nIdx2] - x[nIdx1];
					dir /= arma::norm(dir);
					CMapLine* pNewMapLine = new CMapLine(x[nIdx1], x[nIdx2], dir);
					pNewMapLine->AddObservedKeyFrame(m_pPrevKeyFrame, vKeyLineCoarseMatchPair[i].first);
					pNewMapLine->AddObservedKeyFrame(m_pCurrKeyFrame, vKeyLineCoarseMatchPair[i].second);

					m_pPrevKeyFrame->AddMapLineConnection(vKeyLineCoarseMatchPair[i].first, pNewMapLine);
					m_pCurrKeyFrame->AddMapLineConnection(vKeyLineCoarseMatchPair[i].second, pNewMapLine);

					m_currFrame.AddMapLineConnection(vKeyLineCoarseMatchPair[i].second, pNewMapLine);

					m_pMapManager->AddMapLine(pNewMapLine);

					pNewMapPlane->AddMapLine(pNewMapLine);

					vNewMapLines.push_back(x[nIdx1].at(0));
					vNewMapLines.push_back(x[nIdx1].at(1));
					vNewMapLines.push_back(x[nIdx1].at(2));
					vNewMapLines.push_back(x[nIdx2].at(0));
					vNewMapLines.push_back(x[nIdx2].at(1));
					vNewMapLines.push_back(x[nIdx2].at(2));
				}

				m_pWorldMapView->AddMapLines(vNewMapLines);
			}
		}
	}*/

	m_pWorldMapView->Invalidate(FALSE);

	if (!bSuccess)
		return;
	///< optimize initial structure
	COptimizer::OptimizeInitStructure(m_pPrevKeyFrame, m_pCurrKeyFrame);

	///< after optmization, visualize initial structure and key-frame pose
	arma::mat E(4, 4), R;
	arma::vec t;

	E.eye();
	m_pWorldMapView->AddCameraPose(E.memptr());	///< visualize first key-frame pose
	
	R = m_pCurrKeyFrame->GetR();
	t = m_pCurrKeyFrame->GetT();

	R = R.t();
	t = -R * t;
	E(arma::span(0, 2), arma::span(0, 2)) = R;
	E(arma::span(0, 2), arma::span(3, 3)) = t;
	m_pWorldMapView->AddCameraPose(E.memptr());	///< visualize second key-frame pose

	std::set<CMapPlane*> sInitMapPlanes;
	std::vector<double> vInitMapPoints;
	std::map<int, CMapPoint*> mInitMapPointConnections = m_pCurrKeyFrame->GetMapPointConnections();
	std::map<int, CMapPoint*>::iterator iter = mInitMapPointConnections.begin();
	std::map<int, CMapPoint*>::iterator end = mInitMapPointConnections.end();
	for (; iter != end; iter++) {
		CMapPoint* pMapPoint = iter->second;
		arma::vec x = pMapPoint->GetPosition();

		vInitMapPoints.push_back(x.at(0));
		vInitMapPoints.push_back(x.at(1));
		vInitMapPoints.push_back(x.at(2));

		if (pMapPoint->GetMapPlane())
			sInitMapPlanes.insert(pMapPoint->GetMapPlane());
	}
	m_pWorldMapView->AddMapPoints(vInitMapPoints);

	std::vector<CMapPlane*> vInitMapPlanes(sInitMapPlanes.begin(), sInitMapPlanes.end());
	for (int i = 0; i < (int)vInitMapPlanes.size(); i++) {
		std::vector<double> vInitMapPlaneCountours;
		std::vector<arma::vec> vMapConvexHull = vInitMapPlanes[i]->GetConvexHull();
		for (int j = 0; j < (int)vMapConvexHull.size(); j++) {
			arma::vec x = vMapConvexHull[j];
			vInitMapPlaneCountours.push_back(x.at(0));
			vInitMapPlaneCountours.push_back(x.at(1));
			vInitMapPlaneCountours.push_back(x.at(2));
		}

		m_pWorldMapView->AddMapPlaneContour(vInitMapPlaneCountours);
	}

	m_pWorldMapView->Invalidate(FALSE);
}

bool	CTracker::ReconstructTwoViewFromH(arma::mat& H, std::vector<std::pair<int, int>>& vCoarseMatchPair, std::vector<int>& vInliers)
{
	arma::mat InvK = arma::inv(CFrame::K);
	arma::mat A = InvK*H*K;
	arma::mat U, V;
	arma::vec w;

	arma::svd(U, w, V, A);

	double s = arma::det(U) * arma::det(V);
	double d1 = w.at(0);
	double d2 = w.at(1);
	double d3 = w.at(2);

	if (d1 / d2 < 1.0001 || d2 / d3 < 1.00001)
		return false;

	std::vector<arma::mat> vR;
	std::vector<arma::vec> vT, vN;
	
	//n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
	double aux1 = sqrt((d1*d1 - d2 * d2) / (d1*d1 - d3 * d3));
	double aux3 = sqrt((d2*d2 - d3 * d3) / (d1*d1 - d3 * d3));
	double x1[] = { aux1,aux1,-aux1,-aux1 };
	double x3[] = { aux3,-aux3,aux3,-aux3 };

	//case d'=d2
	double aux_stheta = sqrt((d1*d1 - d2 * d2)*(d2*d2 - d3 * d3)) / ((d1 + d3)*d2);

	double ctheta = (d2*d2 + d1 * d3) / ((d1 + d3)*d2);
	double stheta[] = { aux_stheta, -aux_stheta, -aux_stheta, aux_stheta };

	for (int i = 0; i < 4; i++) {
		arma::mat Rp(3, 3);
		Rp.eye(3, 3);

		Rp.at(0, 0) = ctheta;
		Rp.at(0, 2) = -stheta[i];
		Rp.at(2, 0) = stheta[i];
		Rp.at(2, 2) = ctheta;

		arma::mat R = s * U*Rp*V.t();
		vR.push_back(R);

		arma::vec tp(3);
		tp.at(0) = x1[i];
		tp.at(1) = 0;
		tp.at(2) = -x3[i];
		tp *= d1 - d3;

		arma::vec t = U * tp;
		vT.push_back(t / arma::norm(t));

		arma::vec np(3);
		np.at(0) = x1[i];
		np.at(1) = 0;
		np.at(2) = x3[i];

		arma::vec n = V * np;
		if (n.at(2) < 0)
			n = -n;
		vN.push_back(n);
	}

	//case d'=-d2
	float aux_sphi = sqrt((d1*d1 - d2 * d2)*(d2*d2 - d3 * d3)) / ((d1 - d3)*d2);

	float cphi = (d1*d3 - d2 * d2) / ((d1 - d3)*d2);
	float sphi[] = { aux_sphi, -aux_sphi, -aux_sphi, aux_sphi };

	for (int i = 0; i < 4; i++)
	{
		arma::mat Rp(3, 3);
		Rp.eye(3, 3);

		Rp.at(0, 0) = cphi;
		Rp.at(0, 2) = sphi[i];
		Rp.at(1, 1) = -1;
		Rp.at(2, 0) = sphi[i];
		Rp.at(2, 2) = -cphi;

		arma::mat R = s * U*Rp*V.t();
		vR.push_back(R);

		arma::vec tp(3);
		tp.at(0) = x1[i];
		tp.at(1) = 0;
		tp.at(2) = x3[i];
		tp *= d1 + d3;

		arma::vec t = U * tp;
		vT.push_back(t / arma::norm(t));

		arma::vec np(3);
		np.at(0) = x1[i];
		np.at(1) = 0;
		np.at(2) = x3[i];

		arma::vec n = V * np;
		if (n.at(2) < 0)
			n = -n;
		vN.push_back(n);
	}


	arma::vec X1, X2;
	arma::mat P1(3, 4), P2(3, 4);

	A.set_size(4, 4);
	P1.zeros();
	P1(arma::span(0, 2), arma::span(0, 2)) = arma::eye(3, 3);

	double fx = CFrame::K.at(0, 0), fy = CFrame::K.at(1, 1);
	double cx = CFrame::K.at(0, 2), cy = CFrame::K.at(1, 2); 
	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_initFrame.GetSiftKeyPoints();
	std::vector<SiftKeyPoint>& vDstKeyPoints = m_currFrame.GetSiftKeyPoints();
	
	int nMaxPositive3DPointCount = -1, nMaxPositive3DPointCountIdx;
	for (int i = 0; i < 8; i++) {
		P2(arma::span(0, 2), arma::span(0, 2)) = vR[i];
		P2(arma::span(0, 2), arma::span(3, 3)) = vT[i];
		
		int nPositive3DPointCount = 0;
		for (int j = 0; j < (int)vInliers.size(); j++) {
			SiftKeyPoint& key1 = vSrcKeyPoints[vCoarseMatchPair[vInliers[j]].first];
			SiftKeyPoint& key2 = vDstKeyPoints[vCoarseMatchPair[vInliers[j]].second];

			///< linear triangulation
			double x1 = (key1.x - cx) / fx;
			double y1 = (key1.y - cy) / fy;
			double x2 = (key2.x - cx) / fx;
			double y2 = (key2.y - cy) / fy;

			A.row(0) = x1 * P1.row(2) - P1.row(0);
			A.row(1) = y1 * P1.row(2) - P1.row(1);
			A.row(2) = x2 * P2.row(2) - P2.row(0);
			A.row(3) = y2 * P2.row(2) - P2.row(1);

			arma::svd(U, w, V, A);

			X1 = V.col(3);
			X1 /= X1.at(3);

			X2 = P2 * X1;
			double z1 = X1.at(2);
			double z2 = X2.at(2);

			if (z1 > 0 && z2 > 0)
				nPositive3DPointCount++;
		}

		if (nMaxPositive3DPointCount < nPositive3DPointCount) {
			nMaxPositive3DPointCount = nPositive3DPointCount;
			nMaxPositive3DPointCountIdx = i;
		}
	}

	if (nMaxPositive3DPointCount / (int)vInliers.size() < 0.5 || nMaxPositive3DPointCount < 200)
		return false;

	arma::mat R = vR[nMaxPositive3DPointCountIdx];
	arma::vec t = vT[nMaxPositive3DPointCountIdx];

	m_currFrame.SetR(R);
	m_currFrame.SetT(t);

	//< recompute inlier data of reprojection
	P2(arma::span(0, 2), arma::span(0, 2)) = R;
	P2(arma::span(0, 2), arma::span(3, 3)) = t;

	
	CSLAMKeyFrame* pKeyFrame1 = new CSLAMKeyFrame(m_initFrame);
	CSLAMKeyFrame* pKeyFrame2 = new CSLAMKeyFrame(m_currFrame);
	//std::vector<double> vNewMapPointPositions;
	for (int i = 0; i < (int)vInliers.size(); i++) {
		SiftKeyPoint& key1 = vSrcKeyPoints[vCoarseMatchPair[vInliers[i]].first];
		SiftKeyPoint& key2 = vDstKeyPoints[vCoarseMatchPair[vInliers[i]].second];

		///< linear triangulation
		double x1 = (key1.x - cx) / fx;
		double y1 = (key1.y - cy) / fy;
		double x2 = (key2.x - cx) / fx;
		double y2 = (key2.y - cy) / fy;

		A.row(0) = x1 * P1.row(2) - P1.row(0);
		A.row(1) = y1 * P1.row(2) - P1.row(1);
		A.row(2) = x2 * P2.row(2) - P2.row(0);
		A.row(3) = y2 * P2.row(2) - P2.row(1);

		arma::svd(U, w, V, A);

		X1 = V.col(3);
		X1 /= X1.at(3);

		X2 = P2 * X1;
		double z1 = X1.at(2);
		double z2 = X2.at(2);

		if (z1 > 0 && z2 > 0) {
			CMapPoint* pNewMapPoint = new CMapPoint(X1);
			pNewMapPoint->AddObservedKeyFrame(pKeyFrame1, vCoarseMatchPair[vInliers[i]].first);
			pNewMapPoint->AddObservedKeyFrame(pKeyFrame2, vCoarseMatchPair[vInliers[i]].second);

			pKeyFrame1->AddMapPointConnection(vCoarseMatchPair[vInliers[i]].first, pNewMapPoint);
			pKeyFrame2->AddMapPointConnection(vCoarseMatchPair[vInliers[i]].second, pNewMapPoint);

			m_currFrame.AddMapPointConnection(vCoarseMatchPair[vInliers[i]].second, pNewMapPoint);

			m_pMapManager->AddMapPoint(pNewMapPoint);

			/*vNewMapPointPositions.push_back(X1.at(0));
			vNewMapPointPositions.push_back(X1.at(1));
			vNewMapPointPositions.push_back(X1.at(2));*/
		}
	}

	m_pMapManager->AddInitKeyFrame(pKeyFrame1);
	m_pMapManager->AddInitKeyFrame(pKeyFrame2);

	pKeyFrame1->AddAdjacentKeyFrame(pKeyFrame2);
	pKeyFrame2->AddAdjacentKeyFrame(pKeyFrame1);

	m_pPrevKeyFrame = pKeyFrame1;
	m_pCurrKeyFrame = pKeyFrame2;

	SetCurrentState(TrackingState::Tracking);

	return true;
	/*arma::mat E(4, 4);

	E.eye(4, 4);
	m_pWorldMapView->AddCameraPose(E.memptr());

	R = R.t();
	t = -R * t;
	E(arma::span(0, 2), arma::span(0, 2)) = R;
	E(arma::span(0, 2), arma::span(3, 3)) = t;
	m_pWorldMapView->AddCameraPose(E.memptr());
	m_pWorldMapView->AddMapPoints(vNewMapPointPositions);

	std::vector<double> vKeyFrameConnectivity;
	vKeyFrameConnectivity.push_back(0);
	vKeyFrameConnectivity.push_back(0);
	vKeyFrameConnectivity.push_back(0);
	vKeyFrameConnectivity.push_back(t.at(0));
	vKeyFrameConnectivity.push_back(t.at(1));
	vKeyFrameConnectivity.push_back(t.at(2));

	m_pWorldMapView->AddKeyFramesConnectivity(vKeyFrameConnectivity);
	m_pWorldMapView->Invalidate(FALSE);*/

}

bool	CTracker::ReconstructTwoViewFromF(arma::mat& F, std::vector<std::pair<int, int>>& vCoarseMatchPair, std::vector<int>& vInliers)
{
	std::vector<SiftKeyPoint>& vSrcKeyPoints = m_initFrame.GetSiftKeyPoints();
	std::vector<SiftKeyPoint>& vDstKeyPoints = m_currFrame.GetSiftKeyPoints();

	arma::mat A(4, 4), U, V, VT, W, WT;
	arma::vec s, X1, X2;
	arma::mat I, P[5];
	arma::mat R1, R2;
	arma::vec t1, t2;

	arma::mat E = CFrame::K.t() * F * CFrame::K;

	arma::svd(U, s, V, E);

	W.zeros(3, 3);
	W.at(0, 1) = -1.0;
	W.at(1, 0) = 1.0;
	W.at(2, 2) = 1.0;

	VT = V.t();
	WT = W.t();

	R1 = U * W  * VT;
	R2 = U * WT * VT;

	if (arma::det(R1) < 0)	R1 *= -1;
	if (arma::det(R2) < 0)	R2 *= -1;

	t1 = U(arma::span(0, 2), arma::span(2, 2));
	t2 = -t1;

	I.eye(3, 3);
	P[0].set_size(3, 4);
	P[1].set_size(3, 4);
	P[2].set_size(3, 4);
	P[3].set_size(3, 4);
	P[4].set_size(3, 4);

	///< set rotation matrix
	P[0](arma::span(0, 2), arma::span(0, 2)) = arma::eye(3, 3);
	P[1](arma::span(0, 2), arma::span(0, 2)) = R1;
	P[2](arma::span(0, 2), arma::span(0, 2)) = R1;
	P[3](arma::span(0, 2), arma::span(0, 2)) = R2;
	P[4](arma::span(0, 2), arma::span(0, 2)) = R2;

	///< set translation vector
	P[0](arma::span(0, 2), arma::span(3, 3)) = arma::zeros(3, 1);
	P[1](arma::span(0, 2), arma::span(3, 3)) = t1;
	P[2](arma::span(0, 2), arma::span(3, 3)) = t2;
	P[3](arma::span(0, 2), arma::span(3, 3)) = t1;
	P[4](arma::span(0, 2), arma::span(3, 3)) = t2;

	///< find the real solution for relative pose
	double fx = CFrame::K.at(0, 0), fy = CFrame::K.at(1, 1);
	double cx = CFrame::K.at(0, 2), cy = CFrame::K.at(1, 2);

	int nMaxPositive3DPointCount = 0, nMaxPositive3DPointCountIdx = -1;
	for (int i = 1; i < 5; i++) {
		int nPositive3DPointCount = 0;
		for (int j = 0; j < (int)vInliers.size(); j++) {
			SiftKeyPoint& key1 = vSrcKeyPoints[vCoarseMatchPair[vInliers[j]].first];
			SiftKeyPoint& key2 = vDstKeyPoints[vCoarseMatchPair[vInliers[j]].second];

			///< linear triangulation
			double x1 = (key1.x - cx) / fx;
			double y1 = (key1.y - cy) / fy;
			double x2 = (key2.x - cx) / fx;
			double y2 = (key2.y - cy) / fy;

			A.row(0) = x1 * P[0].row(2) - P[0].row(0);
			A.row(1) = y1 * P[0].row(2) - P[0].row(1);
			A.row(2) = x2 * P[i].row(2) - P[i].row(0);
			A.row(3) = y2 * P[i].row(2) - P[i].row(1);

			arma::svd(U, s, V, A);

			X1 = V.col(3);
			X1 /= X1.at(3);

			X2 = P[i] * X1;
			double z1 = X1.at(2);
			double z2 = X2.at(2);

			if (z1 > 0 && z2 > 0)
				nPositive3DPointCount++;
		}
		
		if (nMaxPositive3DPointCount < nPositive3DPointCount) {
			nMaxPositive3DPointCount = nPositive3DPointCount;
			nMaxPositive3DPointCountIdx = i;
		}
	}

	if (nMaxPositive3DPointCount / (int)vInliers.size() < 0.5 || nMaxPositive3DPointCount < 200)
		return false;

	arma::mat R = P[nMaxPositive3DPointCountIdx](arma::span(0, 2), arma::span(0, 2));
	arma::vec t = P[nMaxPositive3DPointCountIdx](arma::span(0, 2), arma::span(3, 3));
	
	m_currFrame.SetR(R);
	m_currFrame.SetT(t);

	//< recompute inlier data of reprojection
	arma::mat P1 = P[0];
	arma::mat P2 = P[nMaxPositive3DPointCountIdx];

	CSLAMKeyFrame* pKeyFrame1 = new CSLAMKeyFrame(m_initFrame);
	CSLAMKeyFrame* pKeyFrame2 = new CSLAMKeyFrame(m_currFrame);
	//std::vector<double> vNewMapPointPositions;
	for (int i = 0; i < (int)vInliers.size(); i++) {
		SiftKeyPoint& key1 = vSrcKeyPoints[vCoarseMatchPair[vInliers[i]].first];
		SiftKeyPoint& key2 = vDstKeyPoints[vCoarseMatchPair[vInliers[i]].second];

		///< linear triangulation
		double x1 = (key1.x - cx) / fx;
		double y1 = (key1.y - cy) / fy;
		double x2 = (key2.x - cx) / fx;
		double y2 = (key2.y - cy) / fy;

		A.row(0) = x1 * P1.row(2) - P1.row(0);
		A.row(1) = y1 * P1.row(2) - P1.row(1);
		A.row(2) = x2 * P2.row(2) - P2.row(0);
		A.row(3) = y2 * P2.row(2) - P2.row(1);

		arma::svd(U, s, V, A);

		X1 = V.col(3);
		X1 /= X1.at(3);

		X2 = P2 * X1;
		double z1 = X1.at(2);
		double z2 = X2.at(2);

		if (z1 > 0 && z2 > 0) {
			CMapPoint* pNewMapPoint = new CMapPoint(X1);
			pNewMapPoint->AddObservedKeyFrame(pKeyFrame1, vCoarseMatchPair[vInliers[i]].first);
			pNewMapPoint->AddObservedKeyFrame(pKeyFrame2, vCoarseMatchPair[vInliers[i]].second);
			
			pKeyFrame1->AddMapPointConnection(vCoarseMatchPair[vInliers[i]].first, pNewMapPoint);
			pKeyFrame2->AddMapPointConnection(vCoarseMatchPair[vInliers[i]].second, pNewMapPoint);

			m_currFrame.AddMapPointConnection(vCoarseMatchPair[vInliers[i]].second, pNewMapPoint);

			m_pMapManager->AddMapPoint(pNewMapPoint);

			/*vNewMapPointPositions.push_back(X1.at(0));
			vNewMapPointPositions.push_back(X1.at(1));
			vNewMapPointPositions.push_back(X1.at(2));*/
		}
	}

	m_pMapManager->AddInitKeyFrame(pKeyFrame1);
	m_pMapManager->AddInitKeyFrame(pKeyFrame2);
	
	pKeyFrame1->AddAdjacentKeyFrame(pKeyFrame2);
	pKeyFrame2->AddAdjacentKeyFrame(pKeyFrame1);
	
	m_pPrevKeyFrame = pKeyFrame1;
	m_pCurrKeyFrame = pKeyFrame2;

	SetCurrentState(TrackingState::Tracking);

	return true;
	/*E.eye(4, 4);
	m_pWorldMapView->AddCameraPose(E.memptr());

	R = R.t();
	t = -R * t;
	E(arma::span(0, 2), arma::span(0, 2)) = R;
	E(arma::span(0, 2), arma::span(3, 3)) = t;
	m_pWorldMapView->AddCameraPose(E.memptr());
	m_pWorldMapView->AddMapPoints(vNewMapPointPositions);

	std::vector<double> vKeyFrameConnectivity;
	vKeyFrameConnectivity.push_back(0);
	vKeyFrameConnectivity.push_back(0);
	vKeyFrameConnectivity.push_back(0);
	vKeyFrameConnectivity.push_back(t.at(0));
	vKeyFrameConnectivity.push_back(t.at(1));
	vKeyFrameConnectivity.push_back(t.at(2));
	m_pWorldMapView->AddKeyFramesConnectivity(vKeyFrameConnectivity);
	m_pWorldMapView->Invalidate(FALSE);*/
}

std::vector<CMapPlane*>	CTracker::EstimateInitialMapPlane(std::vector<CMapPoint *>& vMapPoints)
{
	std::vector<CMapPlane *> vNewMapPlane;
	std::vector<CMapPoint *> vMapPointsTmp1(vMapPoints);
	std::vector<CMapPoint *> vMapPointsTmp2;

	arma::mat InitWorldPoints;
	
	for (int i = 0; i < (int)vMapPoints.size(); i++)
		InitWorldPoints.insert_cols(InitWorldPoints.n_cols, vMapPoints[i]->GetPosition());

	while (InitWorldPoints.n_cols > vMapPoints.size()*0.1 && InitWorldPoints.n_cols > 50) {
		CPlaneEstimator estimator;
		estimator.SetInitData(InitWorldPoints);
		estimator.SetThreshold(1);
		estimator.EstimatePlaneByRANSAC();

		const std::vector<int>& vPlaneInliers = estimator.GetInlierDataIndices();
		const std::vector<int>& vPlaneOutliers = estimator.GetOutlierDataIndices();

		if (vPlaneInliers.size() < vMapPoints.size()*0.1)
			break;
		else {
			CMapPlane* pNewMapPlane = new CMapPlane();
			
			for (int j = 0; j < (int)vPlaneInliers.size(); j++) {
				pNewMapPlane->AddMapPoint(vMapPointsTmp1[vPlaneInliers[j]]);
				vMapPointsTmp1[vPlaneInliers[j]]->SetMapPlane(pNewMapPlane);
			}
			
			vNewMapPlane.push_back(pNewMapPlane);
			
			vMapPointsTmp2.clear();
			InitWorldPoints.set_size(3, vPlaneOutliers.size());
			for (int j = 0; j < (int)vPlaneOutliers.size(); j++) {
				vMapPointsTmp2.push_back(vMapPointsTmp1[vPlaneOutliers[j]]);
				InitWorldPoints.col(j) = vMapPointsTmp1[vPlaneOutliers[j]]->GetPosition();
			}
			vMapPointsTmp1 = vMapPointsTmp2;
		}
	}

	return vNewMapPlane;
}

void	CTracker::UpdateCurrentKeyFrame(void)
{
	if (m_pCurrKeyFrame->IsBad()) {
		///< current key-frame is not available...
		///< we need to find another reference key-frame

		std::set<CSLAMKeyFrame*> sAdjacentKeyFrames = m_pCurrKeyFrame->GetAdjacentKeyFrames();
		std::set<CSLAMKeyFrame*>::iterator iter = sAdjacentKeyFrames.begin();
		std::set<CSLAMKeyFrame*>::iterator end = sAdjacentKeyFrames.end();

		double dbDist, dbMinDist1 = 9999, dbMinDist2 = 9999;
		CSLAMKeyFrame *pMinDistKeyFrame1, *pMinDistKeyFrame2;
		
		for (; iter != end; iter++) {

		}
		m_pCurrKeyFrame = m_pPrevKeyFrame;

	}
}
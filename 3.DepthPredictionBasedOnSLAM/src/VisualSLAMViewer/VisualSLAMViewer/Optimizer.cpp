#include "stdafx.h"
#include "Optimizer.h"

COptimizer::COptimizer()
{
}

COptimizer::~COptimizer()
{
}

g2o::SE3Quat	COptimizer::ConvertARMAtoSE3(const arma::mat& R, const arma::vec& t)
{
	Eigen::Matrix<double, 3, 3> Rot;
	Rot << R.at(0, 0), R.at(0, 1), R.at(0, 2),
		   R.at(1, 0), R.at(1, 1), R.at(1, 2),
		   R.at(2, 0), R.at(2, 1), R.at(2, 2);

	Eigen::Matrix<double, 3, 1> tran(t.at(0), t.at(1), t.at(2));

	return g2o::SE3Quat(Rot, tran);
}

arma::mat		COptimizer::ConvertSE3toARMA(const g2o::SE3Quat& se3)
{
	arma::mat H(4, 4);
	Eigen::Matrix<double, 4, 4> T = se3.to_homogeneous_matrix();
	
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			H.at(i, j) = T(i, j);
		}
	}

	return H;
}

Eigen::Matrix<double, 3, 1>	COptimizer::ConvertARMAtoVector3d(const arma::vec& pos)
{
	Eigen::Matrix<double, 3, 1> X;
	X << pos.at(0), pos.at(1), pos.at(2);
	
	return X;
}

arma::vec	COptimizer::ConvertVector3dtoARMA(const Eigen::Vector3d& pos)
{
	arma::vec X(3);

	X.at(0) = pos(0);
	X.at(1) = pos(1);
	X.at(2) = pos(2);

	return X;
}

Eigen::Matrix<double, 4, 1>	COptimizer::ConvertARMAtoVector4d(const arma::vec& abcd)
{
	Eigen::Matrix<double, 4, 1> X;
	X << abcd.at(0), abcd.at(1), abcd.at(2), abcd.at(3);

	return X;
}

arma::vec	COptimizer::ConvertVector4dtoARMA(const Eigen::Vector4d& abcd)
{
	arma::vec X(4);
	X.at(0) = abcd(0);
	X.at(1) = abcd(1);
	X.at(2) = abcd(2);
	X.at(3) = abcd(3);

	double norm = sqrt(X.at(0)*X.at(0) +
					   X.at(1)*X.at(1) +
					   X.at(2)*X.at(2));

	X /= norm;

	return X;
}

void	COptimizer::OptimizePose(CFrame& frame)
{
	std::vector<SiftKeyPoint>& vSiftKeyPoints = frame.GetSiftKeyPoints();
	std::map<int, CMapPoint *>& mTrackedMapPointConnections = frame.GetTrackedMapPoints();

	std::unique_ptr<g2o::BlockSolverX::LinearSolverType> pLinearSolver;

	pLinearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();

	std::unique_ptr<g2o::BlockSolverX> pBlockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(pLinearSolver));

	g2o::OptimizationAlgorithm* pOptimizationAlg = new g2o::OptimizationAlgorithmLevenberg(std::move(pBlockSolver));

	g2o::SparseOptimizer* pOptimizer = new g2o::SparseOptimizer;
	pOptimizer->setAlgorithm(pOptimizationAlg);
	pOptimizer->setVerbose(true);

	std::map<int, CMapPoint *>::iterator iter = mTrackedMapPointConnections.begin();
	std::map<int, CMapPoint *>::iterator end = mTrackedMapPointConnections.end();

	g2o::VertexSE3Expmap* pVertexSE3 = new g2o::VertexSE3Expmap();
	pVertexSE3->setEstimate(ConvertARMAtoSE3(frame.GetR(), frame.GetT()));
	pVertexSE3->setId(0);
	pVertexSE3->setFixed(false);
	pOptimizer->addVertex(pVertexSE3);
	
	std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vPoseEdges;
	std::vector<int> vPointFeatureIdx;
	for (; iter != end; iter++) {
		CMapPoint* pMapPoint = iter->second;
		const SiftKeyPoint& key = vSiftKeyPoints[iter->first];

		Eigen::Matrix<double, 2, 1> observation;
		observation << key.x, key.y;

		g2o::EdgeSE3ProjectXYZOnlyPose* pPoseEdge = new g2o::EdgeSE3ProjectXYZOnlyPose();
		pPoseEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(0)));
		pPoseEdge->setMeasurement(observation);
		pPoseEdge->setInformation(Eigen::Matrix2d::Identity());

		g2o::RobustKernelHuber* pRobustKernel = new g2o::RobustKernelHuber;
		pPoseEdge->setRobustKernel(pRobustKernel);
		pRobustKernel->setDelta(std::sqrt(5.991));
		
		pPoseEdge->fx = CFrame::K.at(0, 0);
		pPoseEdge->fy = CFrame::K.at(1, 1);
		pPoseEdge->cx = CFrame::K.at(0, 2);
		pPoseEdge->cy = CFrame::K.at(1, 2);

		arma::vec X = pMapPoint->GetPosition();
		pPoseEdge->Xw[0] = X.at(0);
		pPoseEdge->Xw[1] = X.at(1);
		pPoseEdge->Xw[2] = X.at(2);

		pOptimizer->addEdge(pPoseEdge);

		vPoseEdges.push_back(pPoseEdge);
		vPointFeatureIdx.push_back(iter->first);
	}

	for (int i = 0; i < 4; i++) {
		pVertexSE3->setEstimate(ConvertARMAtoSE3(frame.GetR(), frame.GetT()));
		pOptimizer->initializeOptimization(0);
		pOptimizer->optimize(10);

		for (int j = 0; j < (int)vPoseEdges.size(); j++) {
			g2o::EdgeSE3ProjectXYZOnlyPose* pPoseEdge = vPoseEdges[j];

			double dbError = pPoseEdge->chi2();

			if (dbError > 5.991)
				pPoseEdge->setLevel(1);
			else
				pPoseEdge->setLevel(0);
		}

		if (pOptimizer->edges().size() < 10)
			break;
	}

	// Recover optimized pose and return number of inliers
	g2o::VertexSE3Expmap* pPoseSE3 = static_cast<g2o::VertexSE3Expmap*>((*pOptimizer).vertex(0));
	g2o::SE3Quat SE3quat_recov = pPoseSE3->estimate();
	arma::mat pose = ConvertSE3toARMA(SE3quat_recov);
	frame.SetR(pose(arma::span(0, 2), arma::span(0, 2)));
	frame.SetT(pose(arma::span(0, 2), arma::span(3, 3)));
}

void	COptimizer::OptimizeInitStructure(CSLAMKeyFrame* pPrevKeyFrame, CSLAMKeyFrame* pCurrKeyFrame)
{
	std::map<int, CMapPoint *> mPrevMapPointConnections = pPrevKeyFrame->GetMapPointConnections();
	std::map<int, CMapPoint *> mCurrMapPointConnections = pCurrKeyFrame->GetMapPointConnections();
	std::vector<SiftKeyPoint>& vSrcKeyPoints = pPrevKeyFrame->GetSiftKeyPoints();
	std::vector<SiftKeyPoint>& vDstKeyPoints = pCurrKeyFrame->GetSiftKeyPoints();

	std::unique_ptr<g2o::BlockSolverX::LinearSolverType> pLinearSolver;

	pLinearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>();

	std::unique_ptr<g2o::BlockSolverX> pBlockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(pLinearSolver));

	g2o::OptimizationAlgorithm* pOptimizationAlg = new g2o::OptimizationAlgorithmLevenberg(std::move(pBlockSolver));

	g2o::SparseOptimizer* pOptimizer = new g2o::SparseOptimizer;
	pOptimizer->setAlgorithm(pOptimizationAlg);
	pOptimizer->setVerbose(true);

	std::map<int, CMapPoint *>::iterator iter1 = mPrevMapPointConnections.begin();
	std::map<int, CMapPoint *>::iterator end1 = mPrevMapPointConnections.end();
	
	///< copy all map points, map planes, and key-frames to storage for optimization
	///< we have two key-frames for initiaial structure
	std::set<CSLAMKeyFrame*> sOptimizableKeyFrames;
	sOptimizableKeyFrames.insert(pPrevKeyFrame);
	sOptimizableKeyFrames.insert(pCurrKeyFrame);

	std::vector<CMapPoint*> vOptimizableMapPoints;
	std::set<CMapPlane*> sOptimizableMapPlanes;

	for (; iter1 != end1; iter1++) {
		CMapPoint* pMapPoint = iter1->second;

		std::map<CSLAMKeyFrame*, int> mObservedKeyFrames = pMapPoint->GetObservedKeyFrames();
		std::map<CSLAMKeyFrame*, int>::iterator iter2 = mObservedKeyFrames.begin();
		std::map<CSLAMKeyFrame*, int>::iterator end2 = mObservedKeyFrames.end();
		for (; iter2 != end2; iter2++) {
			if (sOptimizableKeyFrames.find(iter2->first) == sOptimizableKeyFrames.end())
				std::cerr << "this cannot be happened for optimizable key-frame!!" << std::endl;
		}

		CMapPlane* pMapPlane = pMapPoint->GetMapPlane();
		if (pMapPlane)
			sOptimizableMapPlanes.insert(pMapPlane);

		vOptimizableMapPoints.push_back(pMapPoint);
	}

	if (sOptimizableMapPlanes.size() > 1)
		std::cerr << "there are two or more planes in initial structure!!" << std::endl;

	int nOptimizableKeyFrameCount = 0;
	int nOptimizableMapPointCount = 0;
	int nOptimizableMapPlaneCount = 0;
	
	///< set optimizable key-frame to a vertex of g2o framework
	std::set<CSLAMKeyFrame*>::iterator iterOpKeyFrames = sOptimizableKeyFrames.begin();
	std::set<CSLAMKeyFrame*>::iterator endOpKeyFrames = sOptimizableKeyFrames.end();
	
	std::map<CSLAMKeyFrame*, g2o::VertexSE3Expmap*>  mKeyFrameToVertexPose;
	for (; iterOpKeyFrames != endOpKeyFrames; iterOpKeyFrames++) {
		CSLAMKeyFrame* pOptimizableKeyFrame = *iterOpKeyFrames;

		g2o::VertexSE3Expmap* pVertexSE3 = new g2o::VertexSE3Expmap();
		pVertexSE3->setEstimate(ConvertARMAtoSE3(pOptimizableKeyFrame->GetR(), pOptimizableKeyFrame->GetT()));
		pVertexSE3->setId(nOptimizableKeyFrameCount);
		
		if (nOptimizableKeyFrameCount == 0)
			pVertexSE3->setFixed(true);
		else
			pVertexSE3->setFixed(false);
		pOptimizer->addVertex(pVertexSE3);
		
		nOptimizableKeyFrameCount++;
		
		//pOptimizer->addVertex(pVertexSE3);

		mKeyFrameToVertexPose[pOptimizableKeyFrame] = pVertexSE3;
	}

	///< set optimizable map plane to a vertex of g2o framework
	std::set<CMapPlane*>::iterator iterOpMapPlanes = sOptimizableMapPlanes.begin();
	std::set<CMapPlane*>::iterator endOpMapPlanes = sOptimizableMapPlanes.end();
	
	std::map<CMapPlane*, CVertexPlane*> mMapPlaneToVertexPlane;
	for (; iterOpMapPlanes != endOpMapPlanes; iterOpMapPlanes++) {
		CMapPlane* pOptimizableMapPlane = *iterOpMapPlanes;

		CVertexPlane* pVertexPlane = new CVertexPlane();
		pVertexPlane->setEstimate(ConvertARMAtoVector4d(pOptimizableMapPlane->GetParameters()));
		pVertexPlane->setId(nOptimizableKeyFrameCount+ nOptimizableMapPlaneCount);

		pVertexPlane->setFixed(false);
		pOptimizer->addVertex(pVertexPlane);

		mMapPlaneToVertexPlane[pOptimizableMapPlane] = pVertexPlane;

		nOptimizableMapPlaneCount++;
	}

	std::map<CMapPoint*, g2o::VertexSBAPointXYZ*> mMapPointToVertexXYZ;
	for (int i = 0; i < (int)vOptimizableMapPoints.size(); i++) {
		CMapPoint* pMapPoint = vOptimizableMapPoints[i];

		std::map<CSLAMKeyFrame*, int> mObservedKeyFrames = pMapPoint->GetObservedKeyFrames();
		std::map<CSLAMKeyFrame*, int>::iterator iter = mObservedKeyFrames.begin();
		std::map<CSLAMKeyFrame*, int>::iterator end = mObservedKeyFrames.end();

		int nMapPointIDInG2O = nOptimizableKeyFrameCount + nOptimizableMapPlaneCount + nOptimizableMapPointCount;
		g2o::VertexSBAPointXYZ* pXYZPoint = new g2o::VertexSBAPointXYZ();
		pXYZPoint->setEstimate(ConvertARMAtoVector3d(pMapPoint->GetPosition()));
		pXYZPoint->setId(nMapPointIDInG2O);

		///< create map point to key-frame edge
		for (; iter != end; iter++) {
			const SiftKeyPoint key = (iter->first)->GetSiftKeyPoints()[iter->second];
			
			Eigen::Matrix<double, 2, 1> observation;
			observation << key.x, key.y;
			
			mMapPointToVertexXYZ[pMapPoint] = pXYZPoint;

			g2o::EdgeSE3ProjectXYZ* pEdgeXYZ = new g2o::EdgeSE3ProjectXYZ();
			
			pEdgeXYZ->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(mKeyFrameToVertexPose[iter->first]->id())));  ///< set id 0 to map point
			pEdgeXYZ->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(nMapPointIDInG2O)));  ///< set id 0 to map point
			
			pEdgeXYZ->fx = CSLAMKeyFrame::K.at(0, 0);
			pEdgeXYZ->fy = CSLAMKeyFrame::K.at(1, 1);
			pEdgeXYZ->cx = CSLAMKeyFrame::K.at(0, 2);
			pEdgeXYZ->cy = CSLAMKeyFrame::K.at(1, 2);

			pEdgeXYZ->setMeasurement(observation);
			pEdgeXYZ->setInformation(Eigen::Matrix2d::Identity());

			g2o::RobustKernelHuber* pRobustKernel = new g2o::RobustKernelHuber;
			pRobustKernel->setDelta(std::sqrt(5.991));

			pEdgeXYZ->setRobustKernel(pRobustKernel);

			pOptimizer->addEdge(pEdgeXYZ);
		}

		///< create map point to map plane edge
		CMapPlane* pMapPlane = pMapPoint->GetMapPlane();
		if (pMapPlane) {
			CEdgePointOnPlane* pEdgeXYZ = new CEdgePointOnPlane();

			pEdgeXYZ->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(mMapPlaneToVertexPlane[pMapPlane]->id())));  ///< set id 0 to map point
			pEdgeXYZ->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(nMapPointIDInG2O)));  ///< set id 0 to map point

			pEdgeXYZ->setInformation(Eigen::Matrix<double, 1, 1>::Identity());

			g2o::RobustKernelHuber* pRobustKernel = new g2o::RobustKernelHuber;
			pRobustKernel->setDelta(3.0);

			pEdgeXYZ->setRobustKernel(pRobustKernel);

			pOptimizer->addEdge(pEdgeXYZ);
		}

		nOptimizableMapPointCount++;
	}

	pOptimizer->initializeOptimization(0);
	pOptimizer->optimize(10);

	// Recover optimized pose
	g2o::VertexSE3Expmap* pPoseSE3 = static_cast<g2o::VertexSE3Expmap*>((*pOptimizer).vertex(1));
	g2o::SE3Quat SE3quat_recov = pPoseSE3->estimate();
	arma::mat pose = ConvertSE3toARMA(SE3quat_recov);
	pCurrKeyFrame->SetR(pose(arma::span(0, 2), arma::span(0, 2)));
	pCurrKeyFrame->SetT(pose(arma::span(0, 2), arma::span(3, 3)));
	
	// reconver optimized map points
	std::map<CMapPoint*, g2o::VertexSBAPointXYZ*>::iterator iterMapPoint = mMapPointToVertexXYZ.begin();
	std::map<CMapPoint*, g2o::VertexSBAPointXYZ*>::iterator endMapPoint = mMapPointToVertexXYZ.end();
	for (; iterMapPoint != endMapPoint; iterMapPoint++) {
		CMapPoint* pMapPoint = iterMapPoint->first;
		g2o::VertexSBAPointXYZ* pPointXYZ = iterMapPoint->second;

		pMapPoint->SetPosition(ConvertVector3dtoARMA(pPointXYZ->estimate()));
	}

	std::map<CMapPlane*, CVertexPlane*>::iterator iterMapPlane = mMapPlaneToVertexPlane.begin();
	std::map<CMapPlane*, CVertexPlane*>::iterator endMapPlane = mMapPlaneToVertexPlane.end();
	for (; iterMapPlane != endMapPlane; iterMapPlane++) {
		CMapPlane* pMapPlane = iterMapPlane->first;
		CVertexPlane* pVertexPlane = iterMapPlane->second;

		pMapPlane->SetPlaneEquation(ConvertVector4dtoARMA(pVertexPlane->estimate()));

		pMapPlane->UpdateParameter();
	}



	/*std::map<int, CMapPoint *>::iterator iter1 = mPrevMapPointConnections.begin();
	std::map<int, CMapPoint *>::iterator end1 = mPrevMapPointConnections.end();

	g2o::VertexSE3Expmap* pVertexPrevSE3 = new g2o::VertexSE3Expmap();
	pVertexPrevSE3->setEstimate(ConvertARMAtoSE3(pPrevKeyFrame->GetR(), pPrevKeyFrame->GetT()));
	pVertexPrevSE3->setId(0);
	pVertexPrevSE3->setFixed(true);
	pOptimizer->addVertex(pVertexPrevSE3);

	g2o::VertexSE3Expmap* pVertexCurrSE3 = new g2o::VertexSE3Expmap();
	pVertexCurrSE3->setEstimate(ConvertARMAtoSE3(pCurrKeyFrame->GetR(), pCurrKeyFrame->GetT()));
	pVertexCurrSE3->setId(1);
	pVertexCurrSE3->setFixed(false);
	pOptimizer->addVertex(pVertexCurrSE3);

	std::map<CMapPlane*, CVertexPlane*> mMapPlaneToG2OVertex;
	int nMapPointVertexCount = 0;
	std::map<CMapPoint*, int> mMapPointToG2OID;
	for (; iter1 != end1; iter1++) {
		CMapPoint* pMapPoint = iter1->second;

		std::map<CSLAMKeyFrame*, int>& mObservedKeyFrames = pMapPoint->GetObservedKeyFrames();

		if (mObservedKeyFrames.find(pPrevKeyFrame) == mObservedKeyFrames.end()) {
			///< this can not be happend!!!
			std::cerr << "This can not be happend for previous keyframe!!" << std::endl;
		}
		if (mObservedKeyFrames.find(pCurrKeyFrame) == mObservedKeyFrames.end()) {
			///< this can not be happend!!!
			std::cerr << "This can not be happend for current keyframe!!" << std::endl;
		}

		int nSrcKeyPointIdx = mObservedKeyFrames[pPrevKeyFrame];
		int nDstKeyPointIdx = mObservedKeyFrames[pCurrKeyFrame];
		
		const SiftKeyPoint& key1 = vSrcKeyPoints[nSrcKeyPointIdx];
		const SiftKeyPoint& key2 = vDstKeyPoints[nDstKeyPointIdx];

		Eigen::Matrix<double, 2, 1> ob1, ob2;
		ob1 << key1.x, key1.y;
		ob2 << key2.x, key2.y;

		g2o::VertexSBAPointXYZ* pXYZPoint = new g2o::VertexSBAPointXYZ();
		pXYZPoint->setEstimate(ConvertARMAtoVector3d(pMapPoint->GetPosition()));
		pXYZPoint->setId(2 + nMapPointVertexCount);

		mMapPointToG2OID[pMapPoint] = 2 + nMapPointVertexCount;

		g2o::EdgeSE3ProjectXYZ* pPrevPoseEdge = new g2o::EdgeSE3ProjectXYZ();
		g2o::EdgeSE3ProjectXYZ* pCurrPoseEdge = new g2o::EdgeSE3ProjectXYZ();
		
		pPrevPoseEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(2+nMapPointVertexCount)));  ///< set id 0 to map point
		pPrevPoseEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(0)));  ///< set id 1 to key-frame
		pPrevPoseEdge->setMeasurement(ob1);
		pPrevPoseEdge->setInformation(Eigen::Matrix2d::Identity());

		pCurrPoseEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(2+nMapPointVertexCount)));
		pCurrPoseEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(pOptimizer->vertex(1)));
		pCurrPoseEdge->setMeasurement(ob2);
		pCurrPoseEdge->setInformation(Eigen::Matrix2d::Identity());

		g2o::RobustKernelHuber* pPrevRobustKernel = new g2o::RobustKernelHuber;
		pPrevPoseEdge->setRobustKernel(pPrevRobustKernel);
		pPrevRobustKernel->setDelta(std::sqrt(5.991));

		pCurrPoseEdge->fx = pPrevPoseEdge->fx = CFrame::K.at(0, 0);
		pCurrPoseEdge->fy = pPrevPoseEdge->fy = CFrame::K.at(1, 1);
		pCurrPoseEdge->cx = pPrevPoseEdge->cx = CFrame::K.at(0, 2);
		pCurrPoseEdge->cy = pPrevPoseEdge->cy = CFrame::K.at(1, 2);

		g2o::RobustKernelHuber* pCurrRobustKernel = new g2o::RobustKernelHuber;
		pCurrPoseEdge->setRobustKernel(pCurrRobustKernel);
		pCurrRobustKernel->setDelta(std::sqrt(5.991));

		pOptimizer->addEdge(pPrevPoseEdge);
		pOptimizer->addEdge(pCurrPoseEdge);

		nMapPointVertexCount++;
	}

	std::map<CMapPoint*, int>::iterator iter2 = mMapPointToG2OID.begin();
	std::map<CMapPoint*, int>::iterator end2 = mMapPointToG2OID.begin();
	int nMapPlaneCount = 0;
	std::map<CMapPlane*, CVertexPlane*> mg2oMapPlanes;
	for (; iter2 != end2; iter2++) {
		CMapPoint* pMapPoint = iter2->first;
		CMapPlane* pMapPlane = pMapPoint->GetMapPlane();

		if (pMapPlane) {
			CVertexPlane* pVertexPlane;
			if (mg2oMapPlanes.empty() || mg2oMapPlanes.find(pMapPlane) == mg2oMapPlanes.end()) {
				pVertexPlane = new CVertexPlane();
				pVertexPlane->setId(2 + (int)mPrevMapPointConnections.size() + nMapPlaneCount++);
				pVertexPlane->setEstimate(ConvertARMAtoVector4d(pMapPlane->GetParameters()));
			}
			else {
				pVertexPlane = mg2oMapPlanes[pMapPlane];
			}

			CEdgePointOnPlane* pPlaneEdge = new CEdgePointOnPlane();
			pPlaneEdge->setVertex(0, pVertexPlane);
			pPlaneEdge->setVertex(1, pOptimizer->vertex(iter2->second));
			pPlaneEdge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());

			pOptimizer->addEdge(pPlaneEdge);
		}
	}*/

	/*pOptimizer->initializeOptimization();
	pOptimizer->optimize(5);*/

	/*for (int i = 0; i < 4; i++) {
		pVertexSE3->setEstimate(ConvertARMAtoSE3(frame.GetR(), frame.GetT()));
		pOptimizer->initializeOptimization(0);
		pOptimizer->optimize(5.991);

		for (int j = 0; j < (int)vPoseEdges.size(); j++) {
			g2o::EdgeSE3ProjectXYZOnlyPose* pPoseEdge = vPoseEdges[j];

			double dbError = pPoseEdge->chi2();

			if (dbError > 5.991)
				pPoseEdge->setLevel(1);
			else
				pPoseEdge->setLevel(0);
		}

		if (pOptimizer->edges().size() < 10)
			break;
	}*/
}
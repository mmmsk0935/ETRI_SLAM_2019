#include "stdafx.h"
#include "RigidBody.h"

CRigidBody::CRigidBody()
{
	R.eye(3, 3);
	T.zeros(3);
	w.zeros(3);
	SrcK.eye(3, 3);
	DstK.eye(3, 3);

	m_vInlierDataIdx.clear();
	m_nInlierDataCount = (int)m_vInlierDataIdx.size();
}

CRigidBody::~CRigidBody()
{
}

void	CRigidBody::SetInitData(const arma::mat& SrcWorldData, const arma::mat& SrcImageData, 
								const arma::mat& DstWorldData, const arma::mat& DstImageData)
{
	SrcInitWorldPoint = SrcWorldData;
	SrcInitImagePoint = SrcImageData;
	DstInitWorldPoint = DstWorldData;
	DstInitImagePoint = DstImageData;
}

void	CRigidBody::SetIntrinsic(const arma::mat& SrcIntrinsic, const arma::mat& DstIntrinsic)
{
	SrcK = SrcIntrinsic;
	DstK = DstIntrinsic;
}

void	CRigidBody::SetThreshold(double dbThreshold/* = 1.0*/)
{
	m_dbThreshold = dbThreshold;
}

void	CRigidBody::EstimateRigidBodyTransformationByRANSAC(int nRansacIteration/* = 500*/, bool bUseCUDA/* = false*/)
{
	if (bUseCUDA) {
		;
	}
	else {
		std::vector<arma::mat> vRotation(nRansacIteration);
		std::vector<arma::vec> vTranslation(nRansacIteration);
		std::vector<int> vInlierDataCount(nRansacIteration);

		for (int i = 0; i < nRansacIteration; i++) {
		//Concurrency::parallel_for(0, nRansacIteration, [&](int i) {
			int pSampleDataIdx[RIGID_BODY_SAMPLE_COUNT];

			SetSampleData(pSampleDataIdx);

			if (EstimateRigidBodyTransformationFromSample(vRotation[i], vTranslation[i], pSampleDataIdx)) {
				vInlierDataCount[i] = ComputeInlierDataCount(vRotation[i], vTranslation[i]);
			} else
				vInlierDataCount[i] = 0;
		//});
		}

		m_nInlierDataCount = vInlierDataCount[0];
		R = vRotation[0];
		T = vTranslation[0];

		for (int i = 0; i < nRansacIteration; i++) {
			if (m_nInlierDataCount < vInlierDataCount[i]) {
				m_nInlierDataCount = vInlierDataCount[i];
				R = vRotation[i];
				T = vTranslation[i];
			}
		}

		ComputeInlierData();
	}
}

const arma::mat&	CRigidBody::GetR(void)
{
	return R;
}

const arma::vec&	CRigidBody::GetT(void)
{
	return T;
}

int		CRigidBody::GetInlierDataCount(void)
{
	return m_nInlierDataCount;
}

const std::vector<int>&	CRigidBody::GetInlierDataIndices(void)
{
	return m_vInlierDataIdx;
}

void	CRigidBody::SetSampleData(int* pSampleDataIdx)
{
	bool bDuplicate;
	int nRandomIdx;
	
	std::uniform_int_distribution<int> uid(0, (int)SrcInitWorldPoint.n_cols - 1);

	for (int i = 0; i < RIGID_BODY_SAMPLE_COUNT; i++) {
		bDuplicate = false;
		nRandomIdx = uid(en);

		for (int j = 0; j < i; j++) {
			if (pSampleDataIdx[j] == nRandomIdx) {
				bDuplicate = true;
				break;
			}
		}

		if (bDuplicate)	i--;
		else			pSampleDataIdx[i] = nRandomIdx;
	}
}

bool	CRigidBody::EstimateRigidBodyTransformationFromSample(arma::mat& ER, arma::vec& ET, const int* pSampleDataIdx)
{
	arma::vec srcMean(3), dstMean(3);
	arma::vec srcDiff(3), dstDiff(3), s;
	arma::mat C(3, 3), A(3, 3), U, V, D(3, 3);

	ER.eye(3, 3);
	ET.zeros(3);

	srcMean.zeros(), dstMean.zeros();
	for (int i = 0; i < RIGID_BODY_SAMPLE_COUNT; i++) {
		srcMean += SrcInitWorldPoint.col(pSampleDataIdx[i]);
		dstMean += DstInitWorldPoint.col(pSampleDataIdx[i]);
	}

	srcMean /= RIGID_BODY_SAMPLE_COUNT; dstMean /= RIGID_BODY_SAMPLE_COUNT;

	A.zeros();
	for (int i = 0; i < RIGID_BODY_SAMPLE_COUNT; i++) {
		srcDiff = srcMean - SrcInitWorldPoint.col(pSampleDataIdx[i]);
		dstDiff = dstMean - DstInitWorldPoint.col(pSampleDataIdx[i]);

		C = srcDiff * dstDiff.t();
		A += C;
	}

	svd(U, s, V, A);

	D.eye();
	D(2, 2) = det(U*V.t());

	ER = V * D * U.t();
	ET = dstMean - ER*srcMean;

	return true;
}

int		CRigidBody::ComputeInlierDataCount(const arma::mat& ER, const arma::vec& ET)
{
#ifdef E_SYMMETRIC_DISTANCE
	int nInlierDataCount = 0;
	int nInitDataCount = SrcInitWorldPoint.n_cols;

	arma::mat R1 = ER.t();
	arma::vec T1 = -R1*ET;

	arma::mat T2 = repmat(ET, 1, nInitDataCount);
	arma::mat T3 = repmat(T1, 1, nInitDataCount);

	arma::mat SrcToDstImagePoint = DstK * (ER*SrcInitWorldPoint + T2);
	arma::mat DstToSrcImagePoint = SrcK * (R1*DstInitWorldPoint + T3);

	for (int i = 0; i < nInitDataCount; i++) {
		double dbScale = 1 / SrcToDstImagePoint.at(2, i);
		SrcToDstImagePoint.at(0, i) *= dbScale;
		SrcToDstImagePoint.at(1, i) *= dbScale;
		SrcToDstImagePoint.at(2, i) *= dbScale;

		dbScale = 1 / DstToSrcImagePoint.at(2, i);
		DstToSrcImagePoint.at(0, i) *= dbScale;
		DstToSrcImagePoint.at(1, i) *= dbScale;
		DstToSrcImagePoint.at(2, i) *= dbScale;
	}

	arma::mat SrcToDstDiff = SrcToDstImagePoint - DstInitImagePoint;
	arma::mat DstToSrcDiff = DstToSrcImagePoint - SrcInitImagePoint;

	arma::mat Tmp1 = arma::sum(SrcToDstDiff%SrcToDstDiff);
	arma::mat Tmp2 = arma::sum(DstToSrcDiff%DstToSrcDiff);

	arma::mat TotalDistance = (Tmp1 + Tmp2) / 2.0;
	for (int i = 0; i < nInitDataCount; i++) {
		if (TotalDistance.at(i) < m_dbThreshold)
			nInlierDataCount++;
	}

	return nInlierDataCount;
#else
	int nInlierDataCount = 0;
	int nInitDataCount = (int)SrcInitWorldPoint.n_cols;

	arma::mat SrcToDstImagePoint = DstK * (ER*SrcInitWorldPoint + arma::repmat(ET, 1, nInitDataCount));
	
	for (int i = 0; i < nInitDataCount; i++) {
		double dbScale = 1 / SrcToDstImagePoint.at(2, i);
		SrcToDstImagePoint.at(0, i) *= dbScale;
		SrcToDstImagePoint.at(1, i) *= dbScale;
		SrcToDstImagePoint.at(2, i) *= dbScale;
	}

	arma::mat SrcToDstDiff = SrcToDstImagePoint - DstInitImagePoint;
	arma::mat SrcToDstDot = arma::sum(SrcToDstDiff%SrcToDstDiff);
	
	for (int i = 0; i < nInitDataCount; i++) {
		if (SrcToDstDot.at(i) < m_dbThreshold)
			nInlierDataCount++;
	}

	return nInlierDataCount;
#endif // E_SYMMETRIC_DISTANCE
}

void	CRigidBody::ComputeInlierData(void)
{
#ifdef E_SYMMETRIC_DISTANCE
	int nInlierDataCount = 0;
	int nInitDataCount = SrcInitWorldPoint.n_cols;

	arma::mat R1 = R.t();
	arma::vec T1 = -R1*T;

	arma::mat T2 = repmat( T, 1, nInitDataCount);
	arma::mat T3 = repmat(T1, 1, nInitDataCount);

	arma::mat SrcToDstImagePoint = DstK * ( R*SrcInitWorldPoint + T2);
	arma::mat DstToSrcImagePoint = SrcK * (R1*DstInitWorldPoint + T3);

	Concurrency::parallel_for(0, nInitDataCount, [&](int i) {
		double dbScale = 1 / SrcToDstImagePoint.at(2, i);
		SrcToDstImagePoint.at(0, i) *= dbScale;
		SrcToDstImagePoint.at(1, i) *= dbScale;
		SrcToDstImagePoint.at(2, i) *= dbScale;

		dbScale = 1 / DstToSrcImagePoint.at(2, i);
		DstToSrcImagePoint.at(0, i) *= dbScale;
		DstToSrcImagePoint.at(1, i) *= dbScale;
		DstToSrcImagePoint.at(2, i) *= dbScale;
	});

	arma::mat SrcToDstDiff = SrcToDstImagePoint - DstInitImagePoint;
	arma::mat DstToSrcDiff = DstToSrcImagePoint - SrcInitImagePoint;

	arma::mat Tmp1 = arma::sum(SrcToDstDiff%SrcToDstDiff);
	arma::mat Tmp2 = arma::sum(DstToSrcDiff%DstToSrcDiff);

	arma::mat TotalDistance = (Tmp1 + Tmp2) / 2.0;

	m_vInlierDataIdx.clear();
	Concurrency::parallel_for (0, nInitDataCount, [&](int i) {
		if (TotalDistance.at(i) < m_dbThreshold)
			m_vInlierDataIdx.push_back(i);
	});

	return nInlierDataCount;
#else
	int nInlierDataCount = 0;
	int nInitDataCount = (int)SrcInitWorldPoint.n_cols;

	arma::mat SrcToDstImagePoint = DstK * (R*SrcInitWorldPoint + arma::repmat(T, 1, nInitDataCount));

	Concurrency::parallel_for(0, nInitDataCount, [&](int i) {
		double dbScale = 1 / SrcToDstImagePoint.at(2, i);
		SrcToDstImagePoint.at(0, i) *= dbScale;
		SrcToDstImagePoint.at(1, i) *= dbScale;
		SrcToDstImagePoint.at(2, i) *= dbScale;
	});

	arma::mat SrcToDstDiff = SrcToDstImagePoint - DstInitImagePoint;
	arma::mat SrcToDstDot = arma::sum(SrcToDstDiff%SrcToDstDiff);

	m_vInlierDataIdx.clear();
	for (int i = 0; i < nInitDataCount; i++) {
		if (SrcToDstDot.at(i) < m_dbThreshold)
			m_vInlierDataIdx.push_back(i);
	}
	/*Concurrency::parallel_for(0, nInitDataCount, [&](int i) {
		if (SrcToDstDot.at(i) < m_dbThreshold)
			m_vInlierDataIdx.push_back(i);
	});*/
#endif // E_SYMMETRIC_DISTANCE
}
#include "Homography.h"

CHomography::CHomography()
{
	m_nInlierDataCount = 0;
	m_vInlierDataIdx.clear();
	m_dbThreshold = 0;

	R.eye(3, 3);
	t.zeros(3);
	H.zeros(3, 3);

	SrcK.eye(3, 3);
	DstK.eye(3, 3);
}

CHomography::~CHomography()
{
}

void	CHomography::SetIntrinsic(arma::mat& SrcIntrinsic, arma::mat& DstIntrinsic)
{
	SrcK = SrcIntrinsic;
	DstK = DstIntrinsic;
}

void	CHomography::SetInitData(const arma::mat& SrcData, const arma::mat& DstData)
{
	SrcInitData = SrcData;
	DstInitData = DstData;
}

void	CHomography::SetThreshold(double dbThreshold/* = 1.0*/, double dbLambda/* = 1.96*/)
{
	m_dbThreshold = dbThreshold;
	m_dbLambda = dbLambda;
}

void	CHomography::NormalizeSampleData(arma::mat& SrcNormData, arma::mat& DstNormData, arma::mat& SrcT, arma::mat& DstT, const int* pSampleDataIdx)
{
	double pDist[2] = { 0, 0 };
	double pScale[2] = { 0, 0 };
	arma::colvec srcMeanVec(3), dstMeanVec(3);

	srcMeanVec.fill(0);
	dstMeanVec.fill(0);
	for (int i = 0; i < HOMOGRAPHY_SAMPLE_COUNT; i++) {
		srcMeanVec += SrcInitData.col(pSampleDataIdx[i]);
		dstMeanVec += DstInitData.col(pSampleDataIdx[i]);
	}

	srcMeanVec /= HOMOGRAPHY_SAMPLE_COUNT;
	dstMeanVec /= HOMOGRAPHY_SAMPLE_COUNT;

	for (int i = 0; i < HOMOGRAPHY_SAMPLE_COUNT; i++) {
		arma::colvec srcDiff = srcMeanVec - SrcInitData.col(pSampleDataIdx[i]);;
		arma::colvec dstDiff = dstMeanVec - DstInitData.col(pSampleDataIdx[i]);;
		pDist[0] += sqrt(dot(srcDiff, srcDiff));
		pDist[1] += sqrt(dot(dstDiff, dstDiff));
	}

	pDist[0] /= HOMOGRAPHY_SAMPLE_COUNT; pDist[0] /= sqrt(2.0);
	pDist[1] /= HOMOGRAPHY_SAMPLE_COUNT; pDist[1] /= sqrt(2.0);

	pScale[0] = 1.0 / pDist[0];
	pScale[1] = 1.0 / pDist[1];

	SrcT.at(0, 0) = SrcT(1, 1) = pScale[0]; SrcT.at(0, 2) = -pScale[0] * srcMeanVec(0); SrcT.at(1, 2) = -pScale[0] * srcMeanVec(1); SrcT.at(2, 2) = 1.0;
	DstT.at(0, 0) = DstT(1, 1) = pScale[1]; DstT.at(0, 2) = -pScale[1] * dstMeanVec(0); DstT.at(1, 2) = -pScale[1] * dstMeanVec(1); DstT.at(2, 2) = 1.0;

	for (int i = 0; i < HOMOGRAPHY_SAMPLE_COUNT; i++) {
		SrcNormData.col(i) = SrcT * SrcInitData.col(pSampleDataIdx[i]);
		DstNormData.col(i) = DstT * DstInitData.col(pSampleDataIdx[i]);
	}
}

void	CHomography::SetSampleData(int* pSampleDataIdx)
{
	bool bDuplicate;
	int nRandomIdx;
	
	std::uniform_int_distribution<int> uid(0, SrcInitData.n_cols - 1);

	for (int i = 0; i < HOMOGRAPHY_SAMPLE_COUNT; i++) {
		bDuplicate = false;
		nRandomIdx = uid(en);
		//nRandomIdx = rand() % SrcInitData.n_cols;

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

bool	CHomography::EstimateHomographyFromSample(arma::mat& EH, const arma::mat& SrcNormData, const arma::mat& DstNormData)
{
#ifdef SOLVE_H_BY_SVD
	///< solve by Ax = b
	arma::mat A(8, 9), U, V;
	arma::vec s;
	EH[i].eye(3, 3);

	A.zeros();
	for (int i = 0; i < HOMOGRAPHY_SAMPLE_COUNT; i++) {
		int nRowIdx = i * 2;
		
		arma::vec m1 = SrcNormData.col(i);
		arma::vec m2 = DstNormData.col(i);

		A.at(nRowIdx, 0) = m1.at(0) * m2.at(2);
		A.at(nRowIdx, 1) = m1.at(1) * m2.at(2);
		A.at(nRowIdx, 2) = m1.at(2) * m2.at(2);

		A.at(nRowIdx, 6) = -m1.at(0) * m2.at(0);
		A.at(nRowIdx, 7) = -m1.at(1) * m2.at(0);
		A.at(nRowIdx, 8) = -m1.at(2) * m2.at(0);

		nRowIdx++;

		A.at(nRowIdx, 3) = m1.at(0) * m2.at(2);
		A.at(nRowIdx, 4) = m1.at(1) * m2.at(2);
		A.at(nRowIdx, 5) = m1.at(2) * m2.at(2);

		A.at(nRowIdx, 6) = -m1.at(0) * m2.at(1);
		A.at(nRowIdx, 7) = -m1.at(1) * m2.at(1);
		A.at(nRowIdx, 8) = -m1.at(2) * m2.at(1);
	}

	arma::svd(U, s, V, A);
	EH.at(0, 0) = V.at(0, 8); EH.at(0, 1) = V.at(1, 8); EH.at(0, 2) = V.at(2, 8);
	EH.at(1, 0) = V.at(3, 8); EH.at(1, 1) = V.at(4, 8); EH.at(1, 2) = V.at(5, 8);
	EH.at(2, 0) = V.at(6, 8); EH.at(2, 1) = V.at(7, 8); EH.at(2, 2) = V.at(8, 8);

	EH /= EH.at(2, 2);

	return true;
#else
	///< solve by Ax = b
	arma::mat A(8, 8);
	arma::vec b(8), x(8);
	EH.eye(3, 3);

	A.zeros();
	b.zeros();
	x.zeros();

	for (int i = 0; i < HOMOGRAPHY_SAMPLE_COUNT; i++) {
		int nRowIdx = i * 2;
		
		arma::vec m1 = SrcNormData.col(i);
		arma::vec m2 = DstNormData.col(i);

		A.at(nRowIdx, 0) = m1.at(0) * m2.at(2);
		A.at(nRowIdx, 1) = m1.at(1) * m2.at(2);
		A.at(nRowIdx, 2) = m1.at(2) * m2.at(2);

		A.at(nRowIdx, 6) = -m1.at(0) * m2.at(0);
		A.at(nRowIdx, 7) = -m1.at(1) * m2.at(0);
		//A.at(nRowIdx, 8) = -m1.at(2) * m2.at(0);

		b.at(nRowIdx) = m1.at(2) * m2.at(0);

		nRowIdx++;

		A.at(nRowIdx, 3) = m1.at(0) * m2.at(2);
		A.at(nRowIdx, 4) = m1.at(1) * m2.at(2);
		A.at(nRowIdx, 5) = m1.at(2) * m2.at(2);
		
		A.at(nRowIdx, 6) = -m1.at(0) * m2.at(1);
		A.at(nRowIdx, 7) = -m1.at(1) * m2.at(1);
		//A.at(nRowIdx, 8) = -m1.at(2) * m2.at(1);

		b.at(nRowIdx) = m1.at(2) * m2.at(1);
	}

	if (arma::rank(A) == 8) {
		arma::solve(x, A, b);
		EH.at(0, 0) = x.at(0); EH.at(0, 1) = x.at(1); EH.at(0, 2) = x.at(2);
		EH.at(1, 0) = x.at(3); EH.at(1, 1) = x.at(4); EH.at(1, 2) = x.at(5);
		EH.at(2, 0) = x.at(6); EH.at(2, 1) = x.at(7); EH.at(2, 2) = 1.0;

		return true;
	} else
		return false;
#endif // SOLVE_BY_SVD
}

void	CHomography::EstimateHomographyByRANSAC(int nRansacIteration/* = 500*/, bool bUseCUDA/* = false*/)
{
	if (!bUseCUDA) {
		std::vector<int> vInlierDataCount(nRansacIteration);
		std::vector<arma::mat> vHomography(nRansacIteration);

		//Concurrency::parallel_for(0, nRansacIteration, [&](int i) {
		for (int i = 0; i < nRansacIteration; i++) {
			int pSampleDataIdx[HOMOGRAPHY_SAMPLE_COUNT];
			arma::mat SrcNormData(3, HOMOGRAPHY_SAMPLE_COUNT);
			arma::mat DstNormData(3, HOMOGRAPHY_SAMPLE_COUNT);
			arma::mat SrcT(3, 3), DstT(3, 3);
			SrcT.eye(3, 3);
			DstT.eye(3, 3);

			SetSampleData(pSampleDataIdx);
			NormalizeSampleData(SrcNormData, DstNormData, SrcT, DstT, pSampleDataIdx);
			if (EstimateHomographyFromSample(vHomography[i], SrcNormData, DstNormData)) {
				DenormalizeHomography(vHomography[i], SrcT, DstT);
				vInlierDataCount[i] = ComputeInlierDataCount(vHomography[i]);
			}
			else
				vInlierDataCount[i] = 0;
		}
		//});

		///< find maximum inlier data count idx
		m_nInlierDataCount = vInlierDataCount[0];
		H = vHomography[0];

		for (int i = 1; i < nRansacIteration; i++) {
			if (m_nInlierDataCount < vInlierDataCount[i]) {
				m_nInlierDataCount = vInlierDataCount[i];
				H = vHomography[i];
			}
		}
	}
	else {
		;
	}

	ComputeInlierData();
}

void	CHomography::DenormalizeHomography(arma::mat& EH, const arma::mat& SrcT, const arma::mat& DstT)
{
	EH = arma::inv(DstT) * EH * SrcT;
}

int		CHomography::ComputeInlierDataCount(arma::mat& EH)
{
#ifdef H_SYMMETRIC_DISTANCE
	int nInlierDataCount, nInitDataCount = (int)SrcInitData.n_cols;
	arma::mat SrcToDst = EH * SrcInitData;
	arma::mat DstToSrc = arma::inv(EH) * DstInitData;

	arma::mat SrcToDstDiff = SrcToDst - DstInitData;
	arma::mat DstToSrcDiff = DstToSrc - SrcInitData;
	
	arma::mat SrcToDstDot = arma::sum(SrcToDstDiff % SrcToDstDiff);
	arma::mat DstToSrcDot = arma::sum(DstToSrcDiff % DstToSrcDiff);

	arma::mat SymDistance = (SrcToDstDot + DstToSrcDot) * 0.5;

	nInlierDataCount = 0;
	for (int i = 0; i < nInitDataCount; i++) {
		if (SymDistance.at(i) < m_dbThreshold)
			nInlierDataCount++;
	}
	return nInlierDataCount;
#else
	int nInlierDataCount, nInitDataCount = (int)SrcInitData.n_cols;
	arma::mat SrcToDst = EH * SrcInitData;

	double* pSrcToDst = SrcToDst.memptr();
	for (int i = 0; i < nInitDataCount; i++) {
		double* pSrcToDstData = &pSrcToDst[i * 3];
		pSrcToDstData[0] /= pSrcToDstData[2];
		pSrcToDstData[1] /= pSrcToDstData[2];
		pSrcToDstData[2] /= pSrcToDstData[2];
	}
	
	arma::mat SrcToDstDiff = SrcToDst - DstInitData;
	arma::mat SrcToDstDot = arma::sum(SrcToDstDiff % SrcToDstDiff);

	nInlierDataCount = 0;
	for (int i = 0; i < nInitDataCount; i++) {
		if (SrcToDstDot.at(i) < m_dbThreshold)
			nInlierDataCount++;
	}
	return nInlierDataCount;
#endif // H_SYMMETRIC_DISTANCE
}

void	CHomography::ComputeInlierData(void)
{
#ifdef SYMMETRIC_DISTANCE
	int nInitDataCount = (int)SrcInitData.n_cols;
	arma::mat SrcToDst = H * SrcInitData;
	arma::mat DstToSrc = arma::inv(H) * DstInitData;

	arma::mat SrcToDstDiff = SrcToDst - DstInitData;
	arma::mat DstToSrcDiff = DstToSrc - SrcInitData;

	arma::mat SrcToDstDot = arma::sum(SrcToDstDiff % SrcToDstDiff);
	arma::mat DstToSrcDot = arma::sum(DstToSrcDiff % DstToSrcDiff);

	arma::mat SymDistance = (SrcToDstDot + DstToSrcDot) * 0.5;

	m_vInlierDataIdx.clear();
	for (int i = 0; i < nInitDataCount; i++) {
		if (SymDistance.at(i) < m_dbThreshold)
			m_vInlierDataIdx.push_back(i);
	}
	assert((int)m_vInlierDataIdx.size() == m_nInlierDataCount);
#else
	int nInitDataCount = (int)SrcInitData.n_cols;
	arma::mat SrcToDst = H * SrcInitData;

	double* pSrcToDst = SrcToDst.memptr();
	Concurrency::parallel_for (0, nInitDataCount, [&](int i) {
		double* pSrcToDstData = &pSrcToDst[i * 3];
		pSrcToDstData[0] /= pSrcToDstData[2];
		pSrcToDstData[1] /= pSrcToDstData[2];
		pSrcToDstData[2] /= pSrcToDstData[2];
	});

	arma::mat SrcToDstDiff = SrcToDst - DstInitData;
	arma::mat SrcToDstDot = arma::sum(SrcToDstDiff % SrcToDstDiff);

	m_vInlierDataIdx.clear();
	m_vOutlierDataIdx.clear();
	Concurrency::parallel_for(0, nInitDataCount, [&](int i) {
		if (SrcToDstDot.at(i) < m_dbThreshold)
			m_vInlierDataIdx.push_back(i);
		else
			m_vOutlierDataIdx.push_back(i);
	});

	assert((int)m_vInlierDataIdx.size() == m_nInlierDataCount);
#endif // SYMMETRIC_DISTANCE
}

//void	CHomography::RefineHomographyWithInlierData(void)
//{
//
//}

bool	CHomography::ExtractCameraMotionFromHomography(const arma::mat& K)
{
	arma::mat InvK = arma::inv(K);
	arma::mat InvH = InvK * H;
	arma::vec h1 = H.col(0);
	arma::vec h2 = H.col(1);
	arma::vec h3 = H.col(2);

	double dbNormV1 = arma::norm(InvH.col(0));

	if (dbNormV1 != 0) {
		InvK /= dbNormV1;

		arma::vec r1 = InvK * h1;
		arma::vec r2 = InvK * h2;
		arma::vec r3 = arma::cross(r1, r2);

		t = InvK * h3;

		arma::mat R1(3, 3);
		R1.zeros();

		R1.at(0, 0) = r1.at(0); R1.at(1, 0) = r1.at(1); R1.at(2, 0) = r1.at(2);
		R1.at(0, 1) = r2.at(0); R1.at(1, 1) = r2.at(1); R1.at(2, 1) = r2.at(2);
		R1.at(0, 2) = r3.at(0); R1.at(1, 2) = r3.at(1); R1.at(2, 2) = r3.at(2);

		arma::mat U, V;
		arma::vec s;

		arma::svd(U, s, V, R1);

		R = U*V.t();

		return true;
	}
	else
		return false;
}

bool	CHomography::EstimateRelativePose(const arma::mat& K)
{
	std::vector<arma::mat> vR;
	std::vector<arma::vec> vT, vN;

	if (GenerateRelativePoseHyphothesis(K, vR, vT, vN)) {
		SrcNormData = arma::inv(K) * SrcInitData;
		DstNormData = arma::inv(K) * DstInitData;

		arma::mat P[9];

		P[0].zeros(3, 4);
		P[0](arma::span(0, 2), arma::span(0, 2)) = arma::eye(3, 3);
		P[0](arma::span(0, 2), arma::span(3, 3)) = arma::zeros(3);
		int nMaxPositive3DPointCount = 0, nMaxPositive3DPointCountIdx = -1;
		for (int i = 1; i < 8; i++) {
			P[i].zeros(3, 4);
			P[i](arma::span(0, 2), arma::span(0, 2)) = vR[i - 1];
			P[i](arma::span(0, 2), arma::span(3, 3)) = vT[i - 1];

			int nPositive3DPointCount = Get3DPointCountInFrontOfCamera(P[0], P[i]);

			if (nMaxPositive3DPointCount < nPositive3DPointCount) {
				nMaxPositive3DPointCount = nPositive3DPointCount;
				nMaxPositive3DPointCountIdx = i;
			}
		}

		if (nMaxPositive3DPointCount == 0 || (double)nMaxPositive3DPointCount / m_nInlierDataCount < 0.5)
			return false;

		R = P[nMaxPositive3DPointCountIdx](arma::span(0, 2), arma::span(0, 2));
		t = P[nMaxPositive3DPointCountIdx](arma::span(0, 2), arma::span(3, 3));
		return true;
	}
	else
		return false;
}

bool	CHomography::GenerateRelativePoseHyphothesis(const arma::mat& K, std::vector<arma::mat>& vR, std::vector<arma::vec>& vT, std::vector<arma::vec>& vN)
{
	arma::mat HPrime = K * H * arma::inv(K);

	arma::mat U, V;
	arma::vec s;

	arma::svd(U, s, V, HPrime);

	double dbScale = arma::det(U) * arma::det(V.t());

	double d1 = s.at(0);
	double d2 = s.at(1);
	double d3 = s.at(2);

	if (d1 / d2 < 1.00001 || d2 / d3 < 1.00001)
		return false;

	//n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
	double aux1 = sqrt((d1*d1 - d2*d2) / (d1*d1 - d3*d3));
	double aux3 = sqrt((d2*d2 - d3*d3) / (d1*d1 - d3*d3));
	double x1[] = { aux1, aux1, -aux1, -aux1 };
	double x3[] = { aux3, -aux3, aux3, -aux3 };

	//case d'=d2
	double aux_stheta = sqrt((d1*d1 - d2*d2)*(d2*d2 - d3*d3)) / ((d1 + d3)*d2);

	double ctheta = (d2*d2 + d1*d3) / ((d1 + d3)*d2);
	double stheta[] = { aux_stheta, -aux_stheta, -aux_stheta, aux_stheta };

	for (int i = 0; i < 4; i++) {
		arma::mat Rp = arma::eye(3, 3);
		Rp.at(0, 0) = ctheta;
		Rp.at(0, 2) = -stheta[i];
		Rp.at(2, 0) = stheta[i];
		Rp.at(2, 2) = ctheta;

		arma::mat RTmp = dbScale * U * Rp * V.t();
		vR.push_back(RTmp);

		arma::vec tp(3);
		tp.at(0) = x1[i];
		tp.at(1) = 0;
		tp.at(2) = -x3[i];
		tp *= d1 - d3;

		arma::vec TTmp = U*tp;
		vT.push_back(TTmp / arma::norm(TTmp));

		arma::vec np(3);
		np.at(0) = x1[i];
		np.at(1) = 0;
		np.at(2) = x3[i];

		arma::vec n = V*np;
		if (n.at(2) < 0)
			n = -n;
		vN.push_back(n);
	}

	//case d'=-d2
	double aux_sphi = sqrt((d1*d1 - d2*d2)*(d2*d2 - d3*d3)) / ((d1 - d3)*d2);

	double cphi = (d1*d3 - d2*d2) / ((d1 - d3)*d2);
	double sphi[] = { aux_sphi, -aux_sphi, -aux_sphi, aux_sphi };

	for (int i = 0; i < 4; i++) {
		arma::mat Rp = arma::eye(3, 3);
		Rp.at(0, 0) = cphi;
		Rp.at(0, 2) = sphi[i];
		Rp.at(1, 1) = -1;
		Rp.at(2, 0) = sphi[i];
		Rp.at(2, 2) = -cphi;

		arma::mat RTmp = dbScale * U * Rp * V.t();
		vR.push_back(RTmp);

		arma::vec tp(3);
		tp.at(0) = x1[i];
		tp.at(1) = 0;
		tp.at(2) = x3[i];
		tp *= d1 + d3;

		arma::vec TTmp = U * tp;
		vT.push_back(TTmp / arma::norm(TTmp));

		arma::vec np(3);
		np.at(0) = x1[i];
		np.at(1) = 0;
		np.at(2) = x3[i];

		arma::vec n = V * np;
		if (n.at(2) < 0)
			n = -n;
		vN.push_back(n);
	}

	return true;
}

double	CHomography::GetDepth(const arma::vec& X, const arma::mat& P)
{
	// back project
	arma::vec X2 = P*X;

	double det = arma::det(P(arma::span(0, 2), arma::span(0, 2)));
	double w = X2.at(2);
	double W = X.at(3);

	double a = P.at(0, 2);
	double b = P.at(1, 2);
	double c = P.at(2, 2);

	double m3 = sqrt(a*a + b*b + c*c);  // 3rd column of M

	double sign;

	if (det > 0) {
		sign = 1;
	}
	else {
		sign = -1;
	}

	return (w / W)*(sign / m3);
}

int		CHomography::Get3DPointCountInFrontOfCamera(const arma::mat& P1, const arma::mat& P2)
{
	int nPositive = 0;

	///< triangulate
	arma::mat A(4, 4), U, V;
	arma::vec s, X;
	double dbDepth1, dbDepth2;
	for (int i = 0; i < m_nInlierDataCount; i++) {
		double x1 = SrcNormData.at(0, m_vInlierDataIdx[i]);
		double y1 = SrcNormData.at(1, m_vInlierDataIdx[i]);
		double x2 = DstNormData.at(0, m_vInlierDataIdx[i]);
		double y2 = DstNormData.at(1, m_vInlierDataIdx[i]);

		A.at(0, 0) = x1*P1.at(2, 0) - P1.at(0, 0);
		A.at(0, 1) = x1*P1.at(2, 1) - P1.at(0, 1);
		A.at(0, 2) = x1*P1.at(2, 2) - P1.at(0, 2);
		A.at(0, 3) = x1*P1.at(2, 3) - P1.at(0, 3);

		A.at(1, 0) = y1*P1.at(2, 0) - P1.at(1, 0);
		A.at(1, 1) = y1*P1.at(2, 1) - P1.at(1, 1);
		A.at(1, 2) = y1*P1.at(2, 2) - P1.at(1, 2);
		A.at(1, 3) = y1*P1.at(2, 3) - P1.at(1, 3);

		A.at(2, 0) = x2*P2.at(2, 0) - P2.at(0, 0);
		A.at(2, 1) = x2*P2.at(2, 1) - P2.at(0, 1);
		A.at(2, 2) = x2*P2.at(2, 2) - P2.at(0, 2);
		A.at(2, 3) = x2*P2.at(2, 3) - P2.at(0, 3);

		A.at(3, 0) = y2*P2.at(2, 0) - P2.at(1, 0);
		A.at(3, 1) = y2*P2.at(2, 1) - P2.at(1, 1);
		A.at(3, 2) = y2*P2.at(2, 2) - P2.at(1, 2);
		A.at(3, 3) = y2*P2.at(2, 3) - P2.at(1, 3);

		arma::svd(U, s, V, A);

		X = V.col(3);

		///< compute the depth of X
		dbDepth1 = GetDepth(X, P1);
		dbDepth2 = GetDepth(X, P2);

		if (dbDepth1 > 0 && dbDepth2 > 0)
			nPositive++;
	}

	return nPositive;
}

arma::mat&	CHomography::GetR(void)
{
	return R;
}

arma::vec&	CHomography::GetT(void)
{
	return t;
}

arma::mat&	CHomography::GetHomography(void)
{
	return H;
}

int		CHomography::GetInlierDataCount(void)
{
	return m_nInlierDataCount;
}

Concurrency::concurrent_vector<int>&	CHomography::GetInlierDataIndices(void)
{
	return m_vInlierDataIdx;
}

Concurrency::concurrent_vector<int>&	CHomography::GetOutlierDataIndices(void)
{
	return m_vOutlierDataIdx;
}
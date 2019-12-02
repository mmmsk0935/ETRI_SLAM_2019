#include "EightPoint.h"


CEightPoint::CEightPoint()
{
	m_bUseInitK = false;

	SrcK.set_size(3, 3);
	DstK.set_size(3, 3);
	F.set_size(3, 3);
	e1.set_size(3);
	e2.set_size(3);
	
	m_vInlierDataIdx.clear();
	m_nInlierDataCount = (int)m_vInlierDataIdx.size();
}

CEightPoint::~CEightPoint()
{
	m_vInlierDataIdx.clear();
	m_nInlierDataCount = (int)m_vInlierDataIdx.size();
}

void	CEightPoint::SetK(const arma::mat& SrcIntrinsic, const arma::mat& DstIntrinsic)
{
	SrcK = SrcIntrinsic;
	DstK = DstIntrinsic;

	m_bUseInitK = true;
}

void	CEightPoint::SetInitData(double pSrcData[][2], double pDstData[][2], int nInitDataCount)
{
	SrcInitData.set_size(3, nInitDataCount);
	DstInitData.set_size(3, nInitDataCount);
	
	Concurrency::parallel_for (0, nInitDataCount, [&](int i) {
		SrcInitData.at(0, i) = pSrcData[i][0];
		SrcInitData.at(1, i) = pSrcData[i][1];
		SrcInitData.at(1, i) = 1.0;

		DstInitData.at(0, i) = pDstData[i][0];
		DstInitData.at(1, i) = pDstData[i][1];
		DstInitData.at(1, i) = 1.0;
	});
	
	m_nInlierDataCount = 0;
	m_vInlierDataIdx.clear();
}

void	CEightPoint::SetInitData(const arma::mat& SrcData, const arma::mat& DstData)
{
	SrcInitData = SrcData;
	DstInitData = DstData;

	m_nInlierDataCount = 0;
	m_vInlierDataIdx.clear();
}

void	CEightPoint::SetThreshold(double dbThreshold/* = 1.0*/, double dbLambda/* = 1.96*/)
{
	m_dbThreshold = dbThreshold;
	m_dbLambda = dbLambda;
}

void	CEightPoint::SelectSampleData(int* pSampleDataIdx)
{
	bool bDuplicate;
	int nRandomIdx;
	
	std::uniform_int_distribution<int> uid(0, SrcInitData.n_cols - 1);

	for (int i = 0; i < EIGHT_POINT_SAMPLE_COUNT; i++) {
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

void	CEightPoint::EstimateFundamentalMatrixByRANSAC(int nRansacIteration/* = 500*/)
{
	std::vector<int> vInlierDataCount(nRansacIteration);
	std::vector<arma::mat> vFundamental(nRansacIteration);

	if (m_bUseInitK) {
		arma::mat InvSrcK = arma::inv(SrcK);
		arma::mat InvDstK = arma::inv(DstK);

		SrcNormData = InvSrcK * SrcInitData;
		DstNormData = InvDstK * DstInitData;
		Concurrency::parallel_for(0, nRansacIteration, [&](int i) {
		//for (int i = 0; i < nRansacIteration; i++) {
			int pSampleDataIdx[EIGHT_POINT_SAMPLE_COUNT];
			arma::mat SrcSampleNormData(3, EIGHT_POINT_SAMPLE_COUNT);
			arma::mat DstSampleNormData(3, EIGHT_POINT_SAMPLE_COUNT);

			SelectSampleData(pSampleDataIdx);

			for (int j = 0; j < EIGHT_POINT_SAMPLE_COUNT; j++) {
				SrcSampleNormData.col(j) = SrcNormData.col(pSampleDataIdx[j]);
				DstSampleNormData.col(j) = DstNormData.col(pSampleDataIdx[j]);
			}

			if (EstimateFundamentalMatrixFromSample(vFundamental[i], SrcSampleNormData, DstSampleNormData)) {
				EnforceFundamentalMatrixConstraint(vFundamental[i]);

				Denormalize(vFundamental[i], InvSrcK, InvDstK);

				vInlierDataCount[i] = ComputeInlierDataCount(vFundamental[i]);
				vFundamental[i] = vFundamental[i];
			}
			else
				vInlierDataCount[i] = 0;
		//}
		});
	}
	else {
		Concurrency::parallel_for(0, nRansacIteration, [&](int i) {
			int pSampleDataIdx[EIGHT_POINT_SAMPLE_COUNT];
			arma::mat SrcSampleNormData(3, EIGHT_POINT_SAMPLE_COUNT);
			arma::mat DstSampleNormData(3, EIGHT_POINT_SAMPLE_COUNT);
			arma::mat SrcT(3, 3), DstT(3, 3);
			SrcT.eye(3, 3);
			DstT.eye(3, 3);

			SelectSampleData(pSampleDataIdx);
			NormalizeSampleData(SrcSampleNormData, DstSampleNormData, SrcT, DstT, pSampleDataIdx);
			if (EstimateFundamentalMatrixFromSample(vFundamental[i], SrcSampleNormData, DstSampleNormData)) {
				EnforceFundamentalMatrixConstraint(vFundamental[i]);

				Denormalize(vFundamental[i], SrcT, DstT);

				vInlierDataCount[i] = ComputeInlierDataCount(vFundamental[i]);
				vFundamental[i] = vFundamental[i];
			}
			else
				vInlierDataCount[i] = 0;
		});
	}


	///< find maximum inlier data count idx
	m_nInlierDataCount = vInlierDataCount[0];
	F = vFundamental[0];

	for (int i = 1; i < nRansacIteration; i++) {
		if (m_nInlierDataCount < vInlierDataCount[i]) {
			m_nInlierDataCount = vInlierDataCount[i];
			F = vFundamental[i];
		}
	}

	ComputeInlierData();
	ComputeEpipole();
}

void	CEightPoint::NormalizeSampleData(arma::mat& SrcNormData, arma::mat& DstNormData, arma::mat& SrcT, arma::mat& DstT, const int* pSampleDataIdx)
{
	double pDist[2] = { 0, 0 };
	double pScale[2] = { 0, 0 };
	arma::colvec srcMeanVec(3), dstMeanVec(3);

	srcMeanVec.fill(0);
	dstMeanVec.fill(0);
	for (int i = 0; i < EIGHT_POINT_SAMPLE_COUNT; i++) {
		srcMeanVec += SrcInitData.col(pSampleDataIdx[i]);
		dstMeanVec += DstInitData.col(pSampleDataIdx[i]);
	}

	srcMeanVec /= EIGHT_POINT_SAMPLE_COUNT;
	dstMeanVec /= EIGHT_POINT_SAMPLE_COUNT;

	for (int i = 0; i < EIGHT_POINT_SAMPLE_COUNT; i++) {
		arma::colvec srcDiff = srcMeanVec - SrcInitData.col(pSampleDataIdx[i]);;
		arma::colvec dstDiff = dstMeanVec - DstInitData.col(pSampleDataIdx[i]);;
		pDist[0] += sqrt(dot(srcDiff, srcDiff));
		pDist[1] += sqrt(dot(dstDiff, dstDiff));
	}

	pDist[0] /= EIGHT_POINT_SAMPLE_COUNT; pDist[0] /= sqrt(2.0);
	pDist[1] /= EIGHT_POINT_SAMPLE_COUNT; pDist[1] /= sqrt(2.0);

	pScale[0] = 1.0 / pDist[0];
	pScale[1] = 1.0 / pDist[1];

	for (int i = 0; i < EIGHT_POINT_SAMPLE_COUNT; i++) {
		SrcNormData.col(i) = (SrcInitData.col(pSampleDataIdx[i]) - srcMeanVec)*pScale[0];
		DstNormData.col(i) = (DstInitData.col(pSampleDataIdx[i]) - dstMeanVec)*pScale[1];
	}

	SrcT.at(0, 0) = SrcT(1, 1) = pScale[0]; SrcT.at(0, 2) = -pScale[0] * srcMeanVec(0); SrcT.at(1, 2) = -pScale[0] * srcMeanVec(1); SrcT.at(2, 2) = 1.0;
	DstT.at(0, 0) = DstT(1, 1) = pScale[1]; DstT.at(0, 2) = -pScale[1] * dstMeanVec(0); DstT.at(1, 2) = -pScale[1] * dstMeanVec(1); DstT.at(2, 2) = 1.0;
}

bool	CEightPoint::EstimateFundamentalMatrixFromSample(arma::mat& EF, const arma::mat& SrcSampleData, const arma::mat& DstSampleData)
{
#ifdef SOLVE_F_BY_SVD
	int nDataCount = SrcSampleData.n_cols;

	arma::mat A(8, 9), U, V;
	arma::vec S;

	EF.eye(3, 3);
	for (int i = 0; i < 8; i++) {
		arma::colvec srcData = SrcSampleData.col(i);
		arma::colvec dstData = DstSampleData.col(i);
		double x1, y1, x2, y2;

		x1 = srcData(0); y1 = srcData(1);
		x2 = dstData(0); y2 = dstData(1);

		A(i, 0) = x2*x1;
		A(i, 1) = x2*y1;
		A(i, 2) = x2;
		A(i, 3) = y2*x1;
		A(i, 4) = y2*y1;
		A(i, 5) = y2;
		A(i, 6) = x1;
		A(i, 7) = y1;
		A(i, 8) = 1.0;
	}

	arma::svd(U, S, V, A);
	arma::vec x = V.col(8);
	EF.at(0, 0) = x.at(0); EF.at(0, 1) = x.at(1); EF.at(0, 2) = x.at(2);
	EF.at(1, 0) = x.at(3); EF.at(1, 1) = x.at(4); EF.at(1, 2) = x.at(5);
	EF.at(2, 0) = x.at(6); EF.at(2, 1) = x.at(7); EF.at(2, 2) = x.at(8);
	//EF = arma::reshape(V.col(8), 3, 3, 1);
	EF /= EF(2, 2);
	
	return true;
#else
	int nDataCount = SrcSampleData.n_cols;

	arma::mat A(8, 8);
	arma::vec b(8), x(8);

	EF.eye(3, 3);
	for (int i = 0; i < 8; i++) {
		arma::colvec srcData = SrcSampleData.col(i);
		arma::colvec dstData = DstSampleData.col(i);
		double x1, y1, x2, y2;

		x1 = srcData(0); y1 = srcData(1);
		x2 = dstData(0); y2 = dstData(1);

		A(i, 0) = x2*x1;
		A(i, 1) = x2*y1;
		A(i, 2) = x2;
		A(i, 3) = y2*x1;
		A(i, 4) = y2*y1;
		A(i, 5) = y2;
		A(i, 6) = x1;
		A(i, 7) = y1;

		b(i) = -1;
	}

	if (arma::det(A) == 0)
		return false;

	arma::solve(x, A, b);
	EF(0, 0) = x(0); EF(0, 1) = x(1); EF(0, 2) = x(2);
	EF(1, 0) = x(3); EF(1, 1) = x(4); EF(1, 2) = x(5);
	EF(2, 0) = x(6); EF(2, 1) = x(7); EF(2, 2) = 1.0;

	return true;
#endif // SOLVE_F_BY_SVD
}

void	CEightPoint::Denormalize(arma::mat& EF, const arma::mat& SrcT, const arma::mat& DstT)
{
	EF = DstT.t() * EF * SrcT;
}

void	CEightPoint::EnforceFundamentalMatrixConstraint(arma::mat& EF)
{
	arma::mat U(3, 3), D(3, 3), V(3, 3);
	arma::vec s(3);

	arma::svd(U, s, V, EF);

	D = diagmat(s);
	D(2, 2) = 0;

	EF = U * D * V.t();
}

int		CEightPoint::ComputeInlierDataCount(const arma::mat& EF)
{
	int nInitDataCount = SrcInitData.n_cols;
	int nInlierDataCount = 0;

	double dbDistance;

	arma::mat ep1 = EF     * SrcInitData;
	arma::mat ep2 = EF.t() * DstInitData;

	for (int i = 0; i < nInitDataCount; i++) {
		double dbNorm = dot(ep1.col(i), DstInitData.col(i));
		double dbDenominator = 1.0 / (ep1.at(0, i)*ep1.at(0, i) + ep1.at(1, i)*ep1.at(1, i)) + 
							   1.0 / (ep2.at(0, i)*ep2.at(0, i) + ep2.at(1, i)*ep2.at(1, i));
		double dbNumerator = dbNorm*dbNorm;

		dbDistance = dbNumerator*dbDenominator;

		if (dbDistance < m_dbThreshold)
			nInlierDataCount++;
	}

	return nInlierDataCount;
}

void	CEightPoint::ComputeInlierData(void)
{
	int nInitDataCount = SrcInitData.n_cols;
	int nInlierDataCount = 0;

	arma::mat ep1 = F * SrcInitData;
	arma::mat ep2 = F.t() * DstInitData;

	m_dbScore = 0;
	m_vInlierDataIdx.clear();
	for (int i = 0; i < nInitDataCount; i++) {
		double dbNorm = arma::dot(ep1.col(i), DstInitData.col(i));
		double dbDenominator = 1.0 / (ep1.at(0, i)*ep1.at(0, i) + ep1.at(1, i)*ep1.at(1, i)) + 
							   1.0 / (ep2.at(0, i)*ep2.at(0, i) + ep2.at(1, i)*ep2.at(1, i));
		double dbNumerator = dbNorm*dbNorm;

		double dbDistance = dbNumerator*dbDenominator;

		if (dbDistance < m_dbThreshold) {
			m_dbScore += m_dbLambda - dbDistance;
			m_vInlierDataIdx.push_back(i);
		}
	}

	assert(m_nInlierDataCount == (int)m_vInlierDataIdx.size());
}

void	CEightPoint::ComputeEpipole(void)
{
	arma::mat U(3, 3), V(3, 3);
	arma::vec s(3);

	svd(U, s, V, F);

	e1 = V.col(2);
	e2 = U.row(2).t();
}

bool	CEightPoint::EstimateRelativePose(void)
{
	///< extract four solutions of relative pose
	arma::mat U, V, VT, W, WT;
	arma::vec s;
	arma::mat I, P[5];
	arma::mat R1, R2;
	arma::vec t1, t2;

	E = DstK.t() * F * SrcK;

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
	int nMaxPositive3DPointCount = 0, nMaxPositive3DPointCountIdx = -1;
	for (int i = 1; i < 5; i++) {
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

	//arma::vec TmpT = -R.t() * t;
	return true;
}

int		CEightPoint::Get3DPointCountInFrontOfCamera(const arma::mat& P1, const arma::mat& P2)
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

double	CEightPoint::GetDepth(const arma::vec& X, const arma::mat& P)
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

int		CEightPoint::GetInlierDataCount(void)
{
	return m_nInlierDataCount;
}

const std::vector<int>&	CEightPoint::GetInlierDataIndices(void)
{
	return m_vInlierDataIdx;
}

void	CEightPoint::GetEpipoles(arma::vec& ep1, arma::vec& ep2)
{
	ep1 = e1;
	ep2 = e2;
}

const arma::mat&	CEightPoint::GetF(void)
{
	return F;
}

const arma::mat&	CEightPoint::GetR(void)
{
	return R;
}

const arma::vec&	CEightPoint::GetT(void)
{
	return t;
}

double	CEightPoint::GetScore(void)
{
	return m_dbScore;
}
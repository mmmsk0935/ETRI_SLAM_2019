#include "PlaneEstimator.h"

CPlaneEstimator::CPlaneEstimator()
{
	PlaneNormal.zeros(3);
	m_dbDistance = 0;

	m_vInlierDataIdx.clear();
	m_nInlierDataCount = (int)m_vInlierDataIdx.size();
}

CPlaneEstimator::~CPlaneEstimator()
{
}

void	CPlaneEstimator::SetInitData(const arma::mat& WorldData)
{
	InitWorldData = WorldData;
}

void	CPlaneEstimator::SetThreshold(double dbThreshold/* = 0.1*/)
{
	m_dbThreshold = dbThreshold;
}

void	CPlaneEstimator::EstimatePlaneByRANSAC(int nRansacIteration/* = 500*/, bool bUseCUDA/* = false*/)
{
	if (bUseCUDA) {

	} else {
		std::vector<arma::vec> vPlaneNormal(nRansacIteration);
		std::vector<double> vDistance(nRansacIteration);
		std::vector<int> vInlierDataCount(nRansacIteration);

		for (int i = 0; i < nRansacIteration; i++) {
			int pSampleDataIdx[PLANE_SAMPLE_COUNT];

			SetSampleData(pSampleDataIdx);
			if (EstimatePlaneFromSample(vPlaneNormal[i], vDistance[i], pSampleDataIdx)) {
				vInlierDataCount[i] = ComputeInlierDataCount(vPlaneNormal[i], vDistance[i]);
			} else	
				vInlierDataCount[i] = 0;
		}

		m_nInlierDataCount = vInlierDataCount[0];
		PlaneNormal = vPlaneNormal[0];
		m_dbDistance = vDistance[0];

		for (int i = 0; i < nRansacIteration; i++) {
			if (m_nInlierDataCount < vInlierDataCount[i]) {
				m_nInlierDataCount = vInlierDataCount[i];
				PlaneNormal = vPlaneNormal[i];
				m_dbDistance = vDistance[i];
			}
		}
	}
}

void	CPlaneEstimator::SetSampleData(int* pSampleDataIdx)
{
	bool bDuplicate;
	int nRandomIdx;

	std::uniform_int_distribution<int> uid(0, InitWorldData.n_cols - 1);

	for (int i = 0; i < PLANE_SAMPLE_COUNT; i++) {
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

bool	CPlaneEstimator::EstimatePlaneFromSample(arma::vec& EPlaneNormal, double& dbDistance, const int* pSampleDataIdx)
{
	double dbNorm;
	arma::vec v1, v2, v3, dir1, dir2;

	v1 = InitWorldData.col(pSampleDataIdx[0]);
	v2 = InitWorldData.col(pSampleDataIdx[1]);
	v3 = InitWorldData.col(pSampleDataIdx[2]);

	dir1 = v2 - v1;
	dir2 = v3 - v1;

	dir1 /= sqrt(arma::dot(dir1, dir1));
	dir2 /= sqrt(arma::dot(dir2, dir2));

	EPlaneNormal = arma::cross(dir1, dir2);
	dbNorm = sqrt(arma::dot(EPlaneNormal, EPlaneNormal));
	
	if (dbNorm < 1.0e-5)
		return false;

	EPlaneNormal /= dbNorm;
	dbDistance = -arma::dot(EPlaneNormal, v1);
	
	return true;
}

int		CPlaneEstimator::ComputeInlierDataCount(const arma::vec& EPlaneNormal, double dbDistance)
{
	int nInitDataCount = (int)InitWorldData.n_cols, nInlierDataCount = 0;

	arma::rowvec Dot = EPlaneNormal.t() * InitWorldData + dbDistance;

	for (int i = 0; i < nInitDataCount; i++) {
		if (abs(Dot[i]) < m_dbThreshold)
			nInlierDataCount++;
	}

	return nInlierDataCount;
}

void	CPlaneEstimator::ComputeInlierData(void)
{
	int nInitDataCount = (int)InitWorldData.n_cols;
	m_vInlierDataIdx.clear();

	arma::rowvec Dot = PlaneNormal.t() * InitWorldData + m_dbDistance;
	
	Concurrency::parallel_for(0, nInitDataCount, [&](int i) {
		if (abs(Dot[i]) < m_dbThreshold)
			m_vInlierDataIdx.push_back(i);
	});

	assert((int)m_vInlierDataIdx.size() == m_nInlierDataCount);
}

const arma::vec&	CPlaneEstimator::GetNormalVector(void)
{
	return PlaneNormal;
}

double	CPlaneEstimator::GetDistance(void)
{
	return m_dbDistance;
}

int		CPlaneEstimator::GetInlierDataCount(void)
{
	return m_nInlierDataCount;
}

const Concurrency::concurrent_vector<int>&	CPlaneEstimator::GetInlierDataIndices(void)
{
	return m_vInlierDataIdx;
}
#include "MapPlane.h"

CMapPlane::CMapPlane()
{
	cv::RNG rng(12345);

	m_cvColor = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}

CMapPlane::~CMapPlane()
{
}

void	CMapPlane::AddMapPoint(CMapPoint* pMapPoint)
{
	m_vMapPoints.push_back(pMapPoint);
}

void	CMapPlane::UpdateParameter(void)
{
	arma::mat InitWorldPoints(3, (int)m_vMapPoints.size());
	for (int i = 0; i < (int)m_vMapPoints.size(); i++) {
		arma::vec pos = m_vMapPoints[i]->GetPosition();
		InitWorldPoints.at(0, i) = pos.at(0);
		InitWorldPoints.at(1, i) = pos.at(1);
		InitWorldPoints.at(2, i) = pos.at(2);
	}

	CPlaneEstimator estimator;
	estimator.SetInitData(InitWorldPoints);
	estimator.SetThreshold(3.0);
	estimator.EstimatePlaneByRANSAC();

	const arma::vec abc = estimator.GetNormalVector();
	double dbDistance = estimator.GetDistance();

	///< update bounding box
	///< all points are projected onto the plane
	for (int i = 0; i < (int)InitWorldPoints.n_cols; i++) {
		double t = -(abc.at(0)*InitWorldPoints.at(0, i)+ 
					 abc.at(1)*InitWorldPoints.at(1, i)+ 
					 abc.at(2)*InitWorldPoints.at(2, i) - dbDistance);
		InitWorldPoints.at(0, i) += t * abc.at(0);
		InitWorldPoints.at(1, i) += t * abc.at(1);
		InitWorldPoints.at(2, i) += t * abc.at(2);
	}

	arma::vec origin = InitWorldPoints.col(0);
	///< map cooridnates from world to plane, the origin of the plane is the first point
	/*for (int i = 0; i < (int)InitWorldPoints.n_cols; i++) {
		InitWorldPoints.at(0, i) -= InitWorldPoints.at(0, 0);
		InitWorldPoints.at(1, i) -= InitWorldPoints.at(1, 0);
		InitWorldPoints.at(2, i) -= InitWorldPoints.at(2, 0);
	}*/
	InitWorldPoints = InitWorldPoints - arma::repmat(origin, 1, InitWorldPoints.n_cols);

	arma::vec basis1 = InitWorldPoints.col(1);
	arma::vec basis2 = InitWorldPoints.col(2);

	///< normlaization of two basis of the plane
	basis1 /= arma::norm(basis1);
	basis2 /= arma::norm(basis2);
	/*InitWorldPoints.col(1) /= arma::norm(InitWorldPoints.col(1));
	InitWorldPoints.col(2) /= arma::norm(InitWorldPoints.col(2));*/

	///< two basis of the plane are the first to second vecotr, and the first to third vector
	arma::mat PlaneCoords(2, InitWorldPoints.n_cols);
	PlaneCoords.at(0, 0) = 0;
	PlaneCoords.at(0, 1) = 0;
	PlaneCoords.at(1, 0) = 1;
	PlaneCoords.at(1, 1) = 0;
	PlaneCoords.at(2, 0) = 0;
	PlaneCoords.at(2, 1) = 1;
	
	for (int i = 2; i < (int)InitWorldPoints.n_cols; i++) {
		///< project a vector onto the first basis
		PlaneCoords.at(0, i) = arma::dot(InitWorldPoints.col(i), basis1);
		PlaneCoords.at(1, i) = arma::dot(InitWorldPoints.col(i), basis2);
	}

	arma::mat rowMat1 = PlaneCoords.row(0);
	arma::mat rowMat2 = PlaneCoords.row(1);
	arma::uword min1, min2, max1, max2;
	min1 = rowMat1.index_min();
	min2 = rowMat2.index_min();
	max1 = rowMat1.index_max();
	max2 = rowMat2.index_max();
	double dbMinX = rowMat1.at(min1);
	double dbMaxX = rowMat1.at(max1);
	double dbMinY = rowMat2.at(min2);
	double dbMaxY = rowMat2.at(max2);

	arma::vec bb1 = origin + dbMinX*basis1 + dbMinY*basis2;
	arma::vec bb2 = origin + dbMaxX*basis1 + dbMinY*basis2;
	arma::vec bb3 = origin + dbMinX*basis1 + dbMaxY*basis2;
	arma::vec bb4 = origin + dbMaxX*basis1 + dbMaxY*basis2;
}

cv::Scalar	CMapPlane::GetColor(void)
{
	return m_cvColor;
}
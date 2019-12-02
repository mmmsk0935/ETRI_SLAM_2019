#include "stdafx.h"
#include "MapPlane.h"

cv::RNG	CMapPlane::rng = cv::RNG(12345);
CMapPlane::CMapPlane()
{
	m_cvColor = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}

CMapPlane::CMapPlane(arma::vec& abcd)
	: params(abcd)
{
	m_cvColor = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}

CMapPlane::~CMapPlane()
{
}

void	CMapPlane::AddMapPoint(CMapPoint* pMapPoint)
{
	boost::mutex::scoped_lock lock(m_mutexMapPoints);
	m_vMapPoints.push_back(pMapPoint);
}

void	CMapPlane::AddMapLine(CMapLine* pMapLine)
{
	boost::mutex::scoped_lock lock(m_mutexMapLines);
	m_vMapLines.push_back(pMapLine);
}

bool	comp(const arma::vec& a, const arma::vec& b) {
	
	return (a.at(0) < b.at(0)) || (a.at(0) == b.at(0) && a.at(1) < b.at(1));
}

bool right_turn(const arma::vec& a, const arma::vec& b, const arma::vec& c) {
	return ((c.at(0) - a.at(0))*(b.at(1) - a.at(1)) - (c.at(1) - a.at(1))*(b.at(0) - a.at(0))) > 0;
}

void	CMapPlane::UpdateParameter(void)
{
	std::vector<arma::vec> vPositions;
	{
		boost::mutex::scoped_lock lock(m_mutexMapPoints);
		for (int i = 0; i < (int)m_vMapPoints.size(); i++)
			vPositions.push_back(m_vMapPoints[i]->GetPosition());
	}

	arma::vec mean(3), normal(3);
	double d;

	{
		boost::mutex::scoped_lock lock(m_mutexParams);
		normal.at(0) = params.at(0); 
		normal.at(1) = params.at(1); 
		normal.at(2) = params.at(2);
		d = params.at(3);
	}
	
	mean.at(0) = mean.at(1) = mean.at(2) = 0;
	for (int i = 0; i < (int)vPositions.size(); i++) {
		arma::vec x = vPositions[i];
		
		double t = -(arma::dot(normal, x) + d);

		x += t*normal;
		vPositions[i] = x;

		mean += x;
	}

	mean /= (int)vPositions.size();
	for (int i = 0; i < (int)vPositions.size(); i++)
		vPositions[i] -= mean;

	arma::vec b1 = vPositions[0];
	arma::vec b2 = arma::cross(b1, normal);

	b1 /= arma::norm(b1);
	b2 /= arma::norm(b2);

	arma::mat B(3, 2), BT(2, 3), InvBtB(2, 2);

	B.col(0) = b1; B.col(1) = b2;
	BT = B.t();

	B = BT * B;
	
	InvBtB.at(0, 0) =  B.at(1, 1);
	InvBtB.at(0, 1) = -B.at(0, 1);
	InvBtB.at(1, 0) = -B.at(1, 0);
	InvBtB.at(1, 1) =  B.at(0, 0);

	InvBtB /= (InvBtB.at(0, 0)*InvBtB.at(1, 1) - InvBtB.at(0, 1)*InvBtB.at(1, 0));

	InvBtB = InvBtB * BT;
	double dbMinX = 9999, dbMinY = 9999, dbMaxX = -9999, dbMaxY = -9999;

	std::vector<arma::vec> vPointsOnPlane;
	for (int i = 0; i < (int)vPositions.size(); i++) {
		arma::vec b = InvBtB*vPositions[i];
		
		dbMinX = b.at(0) < dbMinX ? b.at(0) : dbMinX;
		dbMinY = b.at(1) < dbMinY ? b.at(1) : dbMinY;
		dbMaxX = b.at(0) > dbMaxX ? b.at(0) : dbMaxX;
		dbMaxY = b.at(1) > dbMaxY ? b.at(1) : dbMaxY;

		vPointsOnPlane.push_back(b);
	}

	std::sort(vPointsOnPlane.begin(), vPointsOnPlane.end(), comp);
	std::vector<arma::vec> upperCH;
	std::vector<arma::vec> lowerCH;

	upperCH.push_back(vPointsOnPlane[0]);
	upperCH.push_back(vPointsOnPlane[1]);
	for (int i = 2; i < (int)vPointsOnPlane.size(); i++)
	{
		while (upperCH.size() > 1 && (!right_turn(upperCH[upperCH.size() - 2], upperCH[upperCH.size() - 1], vPointsOnPlane[i])))
			upperCH.pop_back();
		upperCH.push_back(vPointsOnPlane[i]);
	}

	lowerCH.push_back(vPointsOnPlane[vPointsOnPlane.size() - 1]);
	lowerCH.push_back(vPointsOnPlane[vPointsOnPlane.size() - 2]);
	for (int i = 2; i < (int)vPointsOnPlane.size(); i++)
	{
		while (lowerCH.size() > 1 && (!right_turn(lowerCH[lowerCH.size() - 2], lowerCH[lowerCH.size() - 1], vPointsOnPlane[(int)vPointsOnPlane.size() - i - 1])))
			lowerCH.pop_back();
		lowerCH.push_back(vPointsOnPlane[(int)vPointsOnPlane.size() - i - 1]);
	}

	m_vConvexHull.clear();
	for (int i = 0; i < (int)upperCH.size(); i++)
		m_vConvexHull.push_back(upperCH[i]);
	for (int i = 0; i < (int)lowerCH.size(); i++)
		m_vConvexHull.push_back(lowerCH[i]);

	for (int i = 0; i < (int)m_vConvexHull.size(); i++) {
		m_vConvexHull[i] = (m_vConvexHull[i].at(0)*b1 + m_vConvexHull[i].at(1)*b2) + mean;
	}
	/*m_vConvexHull.push_back((dbMinX*b1 + dbMinY*b2)+mean);
	m_vConvexHull.push_back((dbMaxX*b1 + dbMinY*b2)+mean);
	m_vConvexHull.push_back((dbMaxX*b1 + dbMaxY*b2)+mean);
	m_vConvexHull.push_back((dbMinX*b1 + dbMaxY*b2)+mean);*/
	
	/*std::vector<cv::Point> vPointsOnPlanes;
	std::vector<cv::Point> vContours, vConvexHull;
	int minX = 9999, minY = 9999;
	for (int i = 0; i < (int)vPositions.size(); i++) {
		arma::vec b = InvBtB*BT*vPositions[i];
		vPointsOnPlanes.push_back(cv::Point(cvRound(b.at(0)), cvRound(b.at(1))));

		if (minX > cvRound(b.at(0)))
			minX = cvRound(b.at(0));
		if (minY > cvRound(b.at(1)))
			minY = cvRound(b.at(1));
	}

	for (int i = 0; i < (int)vPointsOnPlanes.size(); i++) {
		vPointsOnPlanes[i].x += minX;
		vPointsOnPlanes[i].y += minY;
	}
	cv::findContours(vPointsOnPlanes, vContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	cv::convexHull(vContours, vConvexHull);

	m_vConvexHull.clear();
	for (int i = 0; i < (int)vConvexHull.size(); i++) {
		arma::vec x = (vConvexHull[i].x+minX)*b1 + (vConvexHull[i].y+minY)*b2;
		m_vConvexHull.push_back(x + mean);
	}*/

	//arma::mat InitWorldPoints(3, (int)m_vMapPoints.size());
	//for (int i = 0; i < (int)m_vMapPoints.size(); i++) {
	//	arma::vec pos = m_vMapPoints[i]->GetPosition();
	//	InitWorldPoints.at(0, i) = pos.at(0);
	//	InitWorldPoints.at(1, i) = pos.at(1);
	//	InitWorldPoints.at(2, i) = pos.at(2);
	//}

	//CPlaneEstimator estimator;
	//estimator.SetInitData(InitWorldPoints);
	//estimator.SetThreshold(3.0);
	//estimator.EstimatePlaneByRANSAC();

	//const arma::vec abc = estimator.GetNormalVector();
	//double dbDistance = estimator.GetDistance();

	/////< update bounding box
	/////< all points are projected onto the plane
	//for (int i = 0; i < (int)InitWorldPoints.n_cols; i++) {
	//	double t = -(abc.at(0)*InitWorldPoints.at(0, i)+ 
	//				 abc.at(1)*InitWorldPoints.at(1, i)+ 
	//				 abc.at(2)*InitWorldPoints.at(2, i) - dbDistance);
	//	InitWorldPoints.at(0, i) += t * abc.at(0);
	//	InitWorldPoints.at(1, i) += t * abc.at(1);
	//	InitWorldPoints.at(2, i) += t * abc.at(2);
	//}

	//arma::vec origin = InitWorldPoints.col(0);
	/////< map cooridnates from world to plane, the origin of the plane is the first point
	///*for (int i = 0; i < (int)InitWorldPoints.n_cols; i++) {
	//	InitWorldPoints.at(0, i) -= InitWorldPoints.at(0, 0);
	//	InitWorldPoints.at(1, i) -= InitWorldPoints.at(1, 0);
	//	InitWorldPoints.at(2, i) -= InitWorldPoints.at(2, 0);
	//}*/
	//InitWorldPoints = InitWorldPoints - arma::repmat(origin, 1, InitWorldPoints.n_cols);

	//arma::vec basis1 = InitWorldPoints.col(1);
	//arma::vec basis2 = InitWorldPoints.col(2);

	/////< normlaization of two basis of the plane
	//basis1 /= arma::norm(basis1);
	//basis2 /= arma::norm(basis2);
	///*InitWorldPoints.col(1) /= arma::norm(InitWorldPoints.col(1));
	//InitWorldPoints.col(2) /= arma::norm(InitWorldPoints.col(2));*/

	/////< two basis of the plane are the first to second vecotr, and the first to third vector
	//arma::mat PlaneCoords(2, InitWorldPoints.n_cols);
	//PlaneCoords.at(0, 0) = 0;
	//PlaneCoords.at(0, 1) = 0;
	//PlaneCoords.at(1, 0) = 1;
	//PlaneCoords.at(1, 1) = 0;
	//PlaneCoords.at(2, 0) = 0;
	//PlaneCoords.at(2, 1) = 1;
	//
	//for (int i = 2; i < (int)InitWorldPoints.n_cols; i++) {
	//	///< project a vector onto the first basis
	//	PlaneCoords.at(0, i) = arma::dot(InitWorldPoints.col(i), basis1);
	//	PlaneCoords.at(1, i) = arma::dot(InitWorldPoints.col(i), basis2);
	//}

	//arma::mat rowMat1 = PlaneCoords.row(0);
	//arma::mat rowMat2 = PlaneCoords.row(1);
	//arma::uword min1, min2, max1, max2;
	//min1 = rowMat1.index_min();
	//min2 = rowMat2.index_min();
	//max1 = rowMat1.index_max();
	//max2 = rowMat2.index_max();
	//double dbMinX = rowMat1.at(min1);
	//double dbMaxX = rowMat1.at(max1);
	//double dbMinY = rowMat2.at(min2);
	//double dbMaxY = rowMat2.at(max2);

	//arma::vec bb1 = origin + dbMinX*basis1 + dbMinY*basis2;
	//arma::vec bb2 = origin + dbMaxX*basis1 + dbMinY*basis2;
	//arma::vec bb3 = origin + dbMinX*basis1 + dbMaxY*basis2;
	//arma::vec bb4 = origin + dbMaxX*basis1 + dbMaxY*basis2;
}

void	CMapPlane::SetColor(cv::Scalar color)
{
	m_cvColor = color;
}

cv::Scalar	CMapPlane::GetColor(void)
{
	return m_cvColor;
}
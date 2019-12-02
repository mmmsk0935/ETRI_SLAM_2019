#pragma once

#include <iostream>
#include <vector>

#include "armadillo"
#include "PlaneEstimator.h"

#include "MapPoint.h"
#include "MapLine.h"

#include "mycv.h"

class CMapPlane
{
private:
	arma::vec	params;
	std::vector<CMapPoint *>	m_vMapPoints;
	std::vector<CMapLine *>		m_vMapLines;

	cv::Scalar	m_cvColor;
public:
	CMapPlane();
	~CMapPlane();

	void	AddMapPoint(CMapPoint* pMapPoint);
	void	UpdateParameter(void);

	cv::Scalar	GetColor(void);
};
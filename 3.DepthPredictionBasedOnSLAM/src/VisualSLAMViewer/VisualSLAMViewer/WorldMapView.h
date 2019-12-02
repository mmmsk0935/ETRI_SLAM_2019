#pragma once

#include "OpenGLWnd.h"

// CWorldMapView
#include <iostream>
#include <fstream>
#include <vector>

#include "armadillo"
#include "Frame.h"

class CWorldMapView : public COpenGLWnd
{
	DECLARE_DYNAMIC(CWorldMapView)

private:
	bool	m_bRenderMapPoints;
	bool	m_bRenderCurrMapPoints;
	bool	m_bRenderCurrKeyFrameMapPoints;
	bool	m_bRenderFloor;
	bool	m_bRenderLocalMapPoints;

	std::vector<double>	m_vMapPoints;
	std::vector<double>	m_vCurrentKeyFrameMapPoints;
	std::vector<double>	m_vCurrentFrameMapPoints;
	std::vector<double>	m_vKeyFrameCameraPoses;
	std::vector<std::vector<double>>	m_vLocalMapPoints;

	std::vector<double>	m_vMapLines;
	std::vector<double>	m_vMapPlanes;
	std::vector<double>	m_vMapPlaneColors;

	std::vector<std::vector<double>>	m_vMapPlaneContuours;
	std::vector<double>	m_vMapPlaneConourColors;

	std::vector<double>	m_vKeyFrameConnections;

	double	m_pCurrCameraPose[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

	std::vector<double>	m_vLineCorrTmp;
public:
	CWorldMapView();
	virtual ~CWorldMapView();

	void	RenderScene(void);

	void	AddCameraPose(double* pCameraPose);
	void	AddMapPoint(double* pMapPoint);
	void	AddMapPoints(std::vector<double>& vMapPoints);
	void	SetCurrentKeyFrameMapPoints(std::vector<double>& vCurrentKeyFrameMapPoints);
	void	SetCurrentTrackedMapPoints(std::vector<double>& vCurrentTrackedMapPoints);
	void	AddLocalMapPoints(std::vector<double>& vLocalMapPoints);
	void	SetRenderLocalMapPoints(bool bRenderLocalMapPoints = false);

	void	AddMapLine(double* pMapLine);
	void	AddMapLines(std::vector<double>& vMapLines);

	void	AddMapPlane(double* pMapPoint, double* pColor);
	void	AddMapPlanes(std::vector<double>& vMapPoints, std::vector<double>& vColors);

	void	AddKeyFramesConnectivity(std::vector<double>& vConnectivities) {
		for (int i = 0; i < (int)vConnectivities.size(); i++)
			m_vKeyFrameConnections.push_back(vConnectivities[i]);
	}

	void	AddMapPlaneContour(std::vector<double>& vMapPlaneContour) {
		m_vMapPlaneContuours.push_back(vMapPlaneContour);
	}

	void	SetCurrentCameraPose(double* pCurrCameraPose) {
		memcpy(m_pCurrCameraPose, pCurrCameraPose, sizeof(double) * 16);
	}
	void	Clear(void);

	void	ToggleRenderMapPoints() {
		m_bRenderMapPoints = !m_bRenderMapPoints;
	}
	void	ToggleRenderCurrMapView() {
		m_bRenderCurrMapPoints = !m_bRenderCurrMapPoints;
	}
	void	ToggleRenderCurrKeyMapView() {
		m_bRenderCurrKeyFrameMapPoints = !m_bRenderCurrKeyFrameMapPoints;
	}
	void	ToggleRenderFloor(void) {
		m_bRenderFloor = !m_bRenderFloor;
	}
	void	SetLineCorrTmp(std::vector<double>& vLineCorrTmp);

	void	ClearMapPlaneContours(void) {
		for (int i = 0; i < (int)m_vMapPlaneContuours.size(); i++)
			m_vMapPlaneContuours[i].clear();
		m_vMapPlaneContuours.clear();
	}
protected:
	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
};
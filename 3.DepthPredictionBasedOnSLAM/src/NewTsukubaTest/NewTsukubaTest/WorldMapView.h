#pragma once

#include "OpenGLWnd.h"

// CWorldMapView
#include <iostream>
#include <fstream>
#include <vector>

class CWorldMapView : public COpenGLWnd
{
	DECLARE_DYNAMIC(CWorldMapView)

private:
	std::vector<double>	m_vMapPoints;
	std::vector<double>	m_vMapColors;
	std::vector<double> m_vCameraPose;
	double m_pCameraPose[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
public:
	CWorldMapView();
	virtual ~CWorldMapView();

	void	RenderScene(void);

	void	SetLocalMap(std::vector<double>& vMapPoints, std::vector<double>& vMapColors);
	void	SetCameraPose(double* pCameraPose);

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
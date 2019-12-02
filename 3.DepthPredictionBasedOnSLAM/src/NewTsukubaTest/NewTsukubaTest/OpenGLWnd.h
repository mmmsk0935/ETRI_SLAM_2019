#pragma once

// COglWnd
#include "stdafx.h"
#include <afxwin.h>
#include <gl\GL.h>
#include <gl\GLU.h>
#include "Point3Dt.h"
#include "Quaternion.h"

#include <vector>

using namespace std;

class COpenGLWnd : public CWnd
{
	DECLARE_DYNAMIC(COpenGLWnd)

public:
	COpenGLWnd();
	virtual ~COpenGLWnd();

protected:
	DECLARE_MESSAGE_MAP()

protected:
	bool	m_bDrawAxis;
	bool	m_bDrawFloorGrid;
	bool	m_bCtrlTranslation;
	bool	m_bCtrlRotation;
	bool	m_bCtrlDepth;

	HDC		m_hDC;
	HGLRC	m_hRC;

	int		m_nWidth;
	int		m_nHeight;
	int		m_nglPixelIndex;

	GLdouble	m_dbAspect;
	GLdouble	m_dbFOV;
	GLdouble	m_dbFar;
	GLdouble	m_dbNear;

	bool	m_bPerspective;

	Point3D<double>	m_T;
	Point3D<double>	m_oT;

	Quaternion	m_R;
	Quaternion	m_oR;

	BOOL	m_bRBDown;
	BOOL	m_bLBDown;

	CPoint	m_oldWinPos;

	GLuint	m_nglFontBase;

	vector<GLdouble>	m_vFloorGrid;
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg void OnSize(UINT nType, int cx, int cy);

	///< rendering method
	virtual	void	RenderScene(void) = 0;
	void	RenderAxis(void);
	void	RenderFloor(void);
	
	BOOL	CreateViewGLContext();
	BOOL	SetWindowPixelFormat();

	void	BuildGLFont(void);
	void	KillGLFont(void);

	void	PrintTextToGLWnd(const char* pText);

	void	SetViewSize(int nWidth, int nHeight);
	void	SetPerspectiveView(int nWidth, int nHeight, double dbAspectRatio, double dbFOV, double dbNear, double dbFar);
	void	SetViewFrustum(int nWidth, int nHeight, double dbLeft, double dbRight, double dbBottom, double dbTop, double dbNear, double dbFar);

	void	AnimateRotation(double dx, double dy);

	void	SetViewControl(bool bTranslation = true, bool bRotation = true, bool bDepth = true);
	void	ToggleDrawAxis(void);
	void	ToggleDrawFloorGrid(void);
};



// OglWnd.cpp : implementation file
//

#include "stdafx.h"
#include "OpenGLWnd.h"


// COglWnd

IMPLEMENT_DYNAMIC(COpenGLWnd, CWnd)

#define	GRID_COUNT	1000

COpenGLWnd::COpenGLWnd()
: m_dbFOV(45.0)
, m_dbFar(150000)
, m_dbNear(0.0001)
, m_bPerspective(TRUE)
, m_bCtrlTranslation(true)
, m_bCtrlRotation(true)
, m_bCtrlDepth(true)
, m_bDrawAxis(true)
, m_bDrawFloorGrid(true)
{
	m_T[0] = -0;
	m_T[1] = -0;
	m_T[2] = -100;
	Quaternion Qt(0.914325223, -0.316902250, -0.099682790, -0.231615391);
	Qt.normalize();
	m_R = Qt;
	m_bRBDown = m_bLBDown = FALSE;

	///< genreate floor grid
	for (int i = 0; i < GRID_COUNT*2; i++) {
		m_vFloorGrid.push_back(-GRID_COUNT);
		//m_vFloorGrid.push_back(120);
		m_vFloorGrid.push_back(0);
		m_vFloorGrid.push_back(GRID_COUNT-i);

		m_vFloorGrid.push_back(GRID_COUNT);
		//m_vFloorGrid.push_back(120);
		m_vFloorGrid.push_back(0);
		m_vFloorGrid.push_back(GRID_COUNT-i);
	}

	for (int i = 0; i < GRID_COUNT*2; i++) {
		m_vFloorGrid.push_back(GRID_COUNT-i);
		//m_vFloorGrid.push_back(120);
		m_vFloorGrid.push_back(0);
		m_vFloorGrid.push_back(GRID_COUNT);

		m_vFloorGrid.push_back(GRID_COUNT-i);
		//m_vFloorGrid.push_back(120);
		m_vFloorGrid.push_back(0);
		m_vFloorGrid.push_back(-GRID_COUNT);
	}
}

COpenGLWnd::~COpenGLWnd()
{
}


BEGIN_MESSAGE_MAP(COpenGLWnd, CWnd)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_SIZE()
END_MESSAGE_MAP()



// COglWnd message handlers



int COpenGLWnd::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here

	if (CWnd::OnCreate(lpCreateStruct) == -1)
		return -1;

	HWND hWnd = GetSafeHwnd();
	m_hDC  = ::GetDC(hWnd);

	if (SetWindowPixelFormat() == FALSE)
		return 0;

	if (CreateViewGLContext() == FALSE)
		return 0;

	m_dbAspect = (GLdouble)m_nWidth/(GLdouble)m_nHeight;
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	wglMakeCurrent(m_hDC, NULL);

	return 0;
}

void COpenGLWnd::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	// Do not call CWnd::OnPaint() for painting messages

	wglMakeCurrent(m_hDC, m_hRC);
	glEnable(GL_DEPTH_TEST);
	RenderScene();
	//RenderFloor();
	SwapBuffers(wglGetCurrentDC());
	glDisable(GL_DEPTH_TEST);
	wglMakeCurrent(m_hDC, NULL);
}

void COpenGLWnd::OnSize(UINT nType, int cx, int cy)
{
	CWnd::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here

	m_nWidth = cx;
	m_nHeight = cy;

	wglMakeCurrent(m_hDC, m_hRC);

	m_dbAspect = (GLdouble)m_nWidth/(GLdouble)m_nHeight;
	glViewport(0, 0, m_nWidth, m_nHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (m_bPerspective)
		gluPerspective(m_dbFOV,m_dbAspect,m_dbNear,m_dbFar);
	else
		glOrtho(-m_nWidth/2, m_nWidth/2, -m_nHeight/2, m_nHeight/2, m_dbNear, m_dbFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	wglMakeCurrent(m_hDC, NULL);
	
}

BOOL COpenGLWnd::CreateViewGLContext()
{
	m_hRC = wglCreateContext(m_hDC);

	if (m_hRC == NULL)
		return FALSE;

	if (wglMakeCurrent(m_hDC, m_hRC) == FALSE)
		return FALSE;

	return TRUE;
}

BOOL COpenGLWnd::SetWindowPixelFormat()
{
	PIXELFORMATDESCRIPTOR pixelDesc;

	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pixelDesc.nVersion = 1;

	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL |PFD_DOUBLEBUFFER | PFD_STEREO;
	pixelDesc.iPixelType = PFD_TYPE_RGBA;
	pixelDesc.cColorBits = 32;
	pixelDesc.cRedBits = 8; 
	pixelDesc.cRedShift = 16;
	pixelDesc.cGreenBits = 8;
	pixelDesc.cGreenShift = 8;
	pixelDesc.cBlueBits = 8;
	pixelDesc.cBlueShift = 0;
	pixelDesc.cAlphaBits = 0;
	pixelDesc.cAlphaShift = 0;
	pixelDesc.cAccumBits = 64;
	pixelDesc.cAccumRedBits = 16;
	pixelDesc.cAccumGreenBits = 16;
	pixelDesc.cAccumBlueBits = 16;
	pixelDesc.cAccumAlphaBits = 0;
	pixelDesc.cDepthBits = 64;
	pixelDesc.cStencilBits = 8;
	pixelDesc.cAuxBuffers = 0;
	pixelDesc.iLayerType = 0;
	pixelDesc.bReserved = 0;
	pixelDesc.dwLayerMask = 0;
	pixelDesc.dwVisibleMask = 0;
	pixelDesc.dwDamageMask = 0;

	m_nglPixelIndex = ChoosePixelFormat(m_hDC, &pixelDesc);
	if (m_nglPixelIndex== 0) { // Choose default
		m_nglPixelIndex = 1;
		if( DescribePixelFormat(m_hDC, m_nglPixelIndex, sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc) == 0)
			return FALSE;
	}
	if (!SetPixelFormat(m_hDC, m_nglPixelIndex, &pixelDesc))
		return FALSE;

	return TRUE;
}

void	COpenGLWnd::BuildGLFont(void)
{
	HFONT font;

	m_nglFontBase = glGenLists(96);

	font = CreateFont(-24,
		0,  
		0,  
		0,  
		FW_BOLD,
		FALSE,
		FALSE,
		FALSE,
		ANSI_CHARSET,
		OUT_TT_PRECIS,  
		CLIP_DEFAULT_PRECIS,  
		ANTIALIASED_QUALITY,  
		FF_DONTCARE|DEFAULT_PITCH,  
		_T("Courier New"));

	SelectObject(m_hDC, font);

	wglUseFontBitmaps(m_hDC, 32, 96, m_nglFontBase);
}

void	COpenGLWnd::KillGLFont(void)
{
	glDeleteLists(m_nglFontBase, 96);
}

void	COpenGLWnd::PrintTextToGLWnd(const char* pText)
{
	glPushAttrib(GL_LIST_BIT);
	glListBase(m_nglFontBase - 32);
	glCallLists((GLsizei)strlen(pText), GL_UNSIGNED_BYTE, pText);
	glPopAttrib();
}

void	COpenGLWnd::SetViewSize(int nWidth, int nHeight)
{
	m_nWidth = nWidth;
	m_nHeight = nHeight;

	wglMakeCurrent(m_hDC, m_hRC);

	m_dbAspect = (GLdouble)m_nWidth/(GLdouble)m_nHeight;
	glViewport(0, 0, m_nWidth, m_nHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	if (m_bPerspective)
		gluPerspective(m_dbFOV,m_dbAspect,m_dbNear,m_dbFar);
	else
		glOrtho(-m_nWidth/2, m_nWidth/2, -m_nHeight/2, m_nHeight/2, m_dbNear, m_dbFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	wglMakeCurrent(m_hDC, NULL);
}

void	COpenGLWnd::SetPerspectiveView(int nWidth, int nHeight, double dbAspectRatio, double dbFOV, double dbNear, double dbFar)
{
	m_nWidth = nWidth;
	m_nHeight = nHeight;
	m_bPerspective = true;

	m_dbAspect = dbAspectRatio;
	m_dbFOV = dbFOV;
	m_dbNear = dbNear;
	m_dbFar = dbFar;

	wglMakeCurrent(m_hDC, m_hRC);

	glViewport(0, 0, m_nWidth, m_nHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(m_dbFOV,m_dbAspect,m_dbNear,m_dbFar);
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	wglMakeCurrent(m_hDC, NULL);
}

void	COpenGLWnd::SetViewFrustum(int nWidth, int nHeight, double dbLeft, double dbRight, double dbBottom, double dbTop, double dbNear, double dbFar)
{
	m_nWidth = nWidth;
	m_nHeight = nHeight;
	m_bPerspective = true;

	wglMakeCurrent(m_hDC, m_hRC);

	glViewport(0, 0, m_nWidth, m_nHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glFrustum(dbLeft, dbRight, dbBottom, dbTop, dbNear, dbFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	wglMakeCurrent(m_hDC, NULL);
}

void	COpenGLWnd::SetViewControl(bool bTranslation, bool bRotation, bool bDepth)
{
	m_bCtrlTranslation = bTranslation;
	m_bCtrlRotation = bRotation;
	m_bCtrlDepth = bDepth;
}

//void	COglWnd::RenderScene(void)
//{
//	RenderAxis();
//}

void	COpenGLWnd::RenderAxis(void)
{
	glLineWidth(1.0f);
	glColor3f(1, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(15000, 0, 0); glVertex3f(0, 0, 0);
	glEnd();

	glLineStipple(2, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINES);
	glVertex3f(-15000, 0, 0);
	glVertex3f(0, 0, 0);
	glEnd();
	glDisable(GL_LINE_STIPPLE);

	glColor3f(0, 1, 0);
	glBegin(GL_LINES);
	glVertex3f(0, 15000, 0);
	glVertex3f(0, 0, 0);
	glEnd();

	glLineStipple(2, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINES);
	glVertex3f(0, -15000, 0);
	glVertex3f(0, 0, 0);
	glEnd();
	glDisable(GL_LINE_STIPPLE);

	glColor3f(0, 0, 1);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 15000);
	glVertex3f(0, 0, 0);
	glEnd();

	glLineStipple(2, 0xAAAA);
	glEnable(GL_LINE_STIPPLE);
	glBegin(GL_LINES);
	glVertex3f(0, 0, -15000);
	glVertex3f(0, 0, 0);
	glEnd();
	glDisable(GL_LINE_STIPPLE);
}

void	COpenGLWnd::RenderFloor(void)
{
	glColor3f(0.5, 0.5, 0.5);

	glLineWidth(1.0);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_DOUBLE, 0, &m_vFloorGrid[0]);
	
	// draw a cube
	glDrawArrays(GL_LINES, 0, (int)m_vFloorGrid.size()/3);

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
}

void	COpenGLWnd::AnimateRotation(double dx, double dy)
{
	Quaternion rot(cos(dx/2.0), sin(dx/2.0), 0.0, 0.0);
	Quaternion rot2(cos(dy/2.0), 0.0, sin(dy/2.0), 0.0);
	rot *= rot2;	
	m_R = rot * m_oR;
}

void	COpenGLWnd::ToggleDrawAxis(void)
{
	m_bDrawAxis = !m_bDrawAxis;
}

void	COpenGLWnd::ToggleDrawFloorGrid(void)
{
	m_bDrawFloorGrid = !m_bDrawFloorGrid;
}
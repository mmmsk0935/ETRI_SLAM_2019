// WorldMapView.cpp: 구현 파일
//

#include "stdafx.h"
#include "NewTsukubaTestView.h"
#include "WorldMapView.h"


// CWorldMapView

IMPLEMENT_DYNAMIC(CWorldMapView, COpenGLWnd)

CWorldMapView::CWorldMapView()
{
}

CWorldMapView::~CWorldMapView()
{
}

void	CWorldMapView::RenderScene(void)
{
	double glrot[4];
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 0.5);

	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_R.ToGLRotate(glrot);

	glTranslated(0.0, 0.0, m_T[2]);
	glRotated(glrot[0], glrot[1], glrot[2], glrot[3]);
	glTranslated(m_T[0], m_T[1], 0.0);

	for (int i = 0; i < (int)m_vCameraPose.size() / 16; i++) {
		static double dbMinX = -320 * 3 / 615.0;
		static double dbMaxX =  320 * 3 / 615.0;
		static double dbMinY = -240 * 3 / 615.0;
		static double dbMaxY =  240 * 3 / 615.0;

		glColor3f(1.0, 0.0, 0.0);
		glLineWidth(2.0);
		glPushMatrix();
			glMultMatrixd(&m_vCameraPose[i * 16]);
			glBegin(GL_LINES);
				glVertex3d(0, 0, 0);
				glVertex3d(dbMinX, dbMinY, 3);

				glVertex3d(0, 0, 0);
				glVertex3d(dbMaxX, dbMinY, 3);

				glVertex3d(0, 0, 0);
				glVertex3d(dbMaxX, dbMaxY, 3);

				glVertex3d(0, 0, 0);
				glVertex3d(dbMinX, dbMaxY, 3);

				glVertex3d(dbMinX, dbMinY, 3);
				glVertex3d(dbMaxX, dbMinY, 3);

				glVertex3d(dbMaxX, dbMinY, 3);
				glVertex3d(dbMaxX, dbMaxY, 3);

				glVertex3d(dbMaxX, dbMaxY, 3);
				glVertex3d(dbMinX, dbMaxY, 3);

				glVertex3d(dbMinX, dbMaxY, 3);
				glVertex3d(dbMinX, dbMinY, 3);
			glEnd();
		glPopMatrix();
	}
	if (!m_vMapPoints.empty() && !m_vMapColors.empty()) {
		if (m_vMapPoints.size() == m_vMapColors.size()) {

			glPushMatrix();
				glMultMatrixd(m_pCameraPose);
			
				glEnableClientState(GL_VERTEX_ARRAY);
				glEnableClientState(GL_COLOR_ARRAY);

				glColorPointer(3, GL_DOUBLE, 0, &m_vMapColors[0]);
				glVertexPointer(3, GL_DOUBLE, 0, &m_vMapPoints[0]);

				glDrawArrays(GL_POINTS, 0, (GLsizei)m_vMapPoints.size() / 3);
				glDrawArrays(GL_COLOR, 0, (GLsizei)m_vMapColors.size() / 3);

				glDisableClientState(GL_VERTEX_ARRAY);
				glDisableClientState(GL_COLOR_ARRAY);
			glPopMatrix();
		}
	}

	glLineWidth(1.0);
	RenderAxis();

	glDisable(GL_DEPTH_TEST);
}

void	CWorldMapView::SetLocalMap(std::vector<double>& vMapPoints, std::vector<double>& vMapColors)
{
	if (m_vMapPoints.size() != vMapPoints.size())
		m_vMapPoints.resize(vMapPoints.size());
	if (m_vMapColors.size() != vMapColors.size())
		m_vMapColors.resize(vMapColors.size());

	std::copy(vMapPoints.begin(), vMapPoints.end(), m_vMapPoints.begin());
	std::copy(vMapColors.begin(), vMapColors.end(), m_vMapColors.begin());
}

void	CWorldMapView::SetCameraPose(double* pCameraPose)
{
	memcpy(m_pCameraPose, pCameraPose, sizeof(double) * 16);
	for (int i = 0; i < 16; i++)
		m_vCameraPose.push_back(pCameraPose[i]);
}

BEGIN_MESSAGE_MAP(CWorldMapView, COpenGLWnd)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
END_MESSAGE_MAP()

// CWorldMapView 메시지 처리기
void CWorldMapView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bLBDown = true;
	m_oldWinPos = point;
	m_oT = m_T;
	m_oR = m_R;

	COpenGLWnd::OnLButtonDown(nFlags, point);
}

void CWorldMapView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bLBDown = false;

	COpenGLWnd::OnLButtonUp(nFlags, point);
}

void CWorldMapView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bRBDown = true;
	m_oldWinPos = point;
	m_oT = m_T;
	m_oR = m_R;

	COpenGLWnd::OnRButtonDown(nFlags, point);
}

void CWorldMapView::OnRButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bRBDown = false;

	COpenGLWnd::OnRButtonUp(nFlags, point);
}

void CWorldMapView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	if (m_bLBDown && m_bRBDown) {

		double	dx, dy;
		dx = (point.y - m_oldWinPos.y) / 50.0;
		dy = (point.x - m_oldWinPos.x) / 50.0;

		m_T[2] += dx;
	}
	else if (m_bLBDown) {
		if (m_bCtrlTranslation) {
			m_T[0] += (point.x - m_oldWinPos.x) / 10.0;
			m_T[1] += (point.y - m_oldWinPos.y) / 10.0;

			m_oldWinPos = point;
		}
	}
	else if (m_bRBDown) {
		if (m_bCtrlRotation) {
			double	dx, dy;
			dx = (point.y - m_oldWinPos.y) / 500.0;
			dy = (point.x - m_oldWinPos.x) / 500.0;
			AnimateRotation(dx, dy);
		}
	}

	COpenGLWnd::OnMouseMove(nFlags, point);

	Invalidate(FALSE);
}

BOOL CWorldMapView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: Add your message handler code here and/or call default
	if (m_bCtrlDepth) {
		/*GLdouble pModelView[16];

		wglMakeCurrent(m_hDC, m_hRC);
		glGetDoublev(GL_MODELVIEW_MATRIX, pModelView);
		wglMakeCurrent(m_hDC, NULL);

		double view[3] = { pModelView[13], pModelView[14], pModelView[15] };
		m_T[0] += view[0] * zDelta / 500.0;
		m_T[1] += view[1] * zDelta / 500.0;*/
		m_T[2] += zDelta / 10.0;
	}

	Invalidate(FALSE);

	return COpenGLWnd::OnMouseWheel(nFlags, zDelta, pt);
}
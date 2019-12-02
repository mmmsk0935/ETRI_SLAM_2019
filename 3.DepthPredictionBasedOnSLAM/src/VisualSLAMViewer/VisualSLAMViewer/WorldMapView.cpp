// WorldMapView.cpp: 구현 파일
//

#include "stdafx.h"
#include "VisualSLAMViewer.h"
#include "WorldMapView.h"


// CWorldMapView

IMPLEMENT_DYNAMIC(CWorldMapView, COpenGLWnd)

CWorldMapView::CWorldMapView()
{
	m_bRenderMapPoints = true;
	m_bRenderCurrMapPoints = true;
	m_bRenderCurrKeyFrameMapPoints = true;
	m_bRenderFloor = true;
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

	///< draw map points at world coordinates
	if (m_bRenderMapPoints && !m_vMapPoints.empty() && (int)m_vMapPoints.size() % 3 == 0) {
		glPointSize(2.0);
		glColor3f(0.5, 0.5, 0.5);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &m_vMapPoints[0]);
		glDrawArrays(GL_POINTS, 0, (GLsizei)m_vMapPoints.size() / 3);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	///< draw map points at current key frame at world coordinates
	if (m_bRenderCurrMapPoints && !m_vCurrentKeyFrameMapPoints.empty() && (int)m_vCurrentKeyFrameMapPoints.size() % 3 == 0) {
		glPointSize(3.0);
		glColor3f(0.0, 1.0, 0.0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &m_vCurrentKeyFrameMapPoints[0]);
		glDrawArrays(GL_POINTS, 0, (GLsizei)m_vCurrentKeyFrameMapPoints.size() / 3);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	///< draw map points at current frame at world coordinates(tracked map points at world coordinates)
	if (m_bRenderCurrKeyFrameMapPoints && !m_vCurrentFrameMapPoints.empty() && (int)m_vCurrentFrameMapPoints.size() % 3 == 0) {
		glPointSize(5.0);
		glColor3f(1.0, 0.0, 0.0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &m_vCurrentFrameMapPoints[0]);
		glDrawArrays(GL_POINTS, 0, (GLsizei)m_vCurrentFrameMapPoints.size() / 3);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	if (!m_vMapLines.empty() && (int)m_vMapLines.size() % 6 == 0) {
		glLineWidth(2.0);
		glColor3f(0.0, 1.0, 1.0);
		glLineWidth(2.0);
		glColor3f(0.0, 1.0, 1.0);
		for (int i = 0; i < (int)m_vMapLines.size() / 6; i++) {
			glBegin(GL_LINES);
			glVertex3dv(&m_vMapLines[(2 * i) * 3]);
			glVertex3dv(&m_vMapLines[(2 * i + 1) * 3]);
			glEnd();
		}
		/*glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &m_vMapLines[0]);
		glDrawArrays(GL_LINES, 0, (GLsizei)m_vMapLines.size() / 6);
		glDisableClientState(GL_VERTEX_ARRAY);*/
	}

	glPointSize(3.0);
	if (!m_vMapPlanes.empty() && (int)m_vMapPlanes.size() % 3 == 0) {
		
		glEnableClientState(GL_VERTEX_ARRAY);

		if (m_vMapPlanes.size() == m_vMapPlaneColors.size()) {
			glEnableClientState(GL_COLOR_ARRAY);
			glColorPointer(3, GL_DOUBLE, 0, &m_vMapPlaneColors[0]);
		}
		else
			glColor3f(1.0, 0.0, 1.0);

		glVertexPointer(3, GL_DOUBLE, 0, &m_vMapPlanes[0]);
		glDrawArrays(GL_POINTS, 0, (GLsizei)m_vMapPlanes.size() / 3);
		
		if (m_vMapPlanes.size() == m_vMapPlaneColors.size())
			glDrawArrays(GL_COLOR, 0, (GLsizei)m_vMapPlaneColors.size() / 3);

		glDisableClientState(GL_VERTEX_ARRAY);
		if (m_vMapPlanes.size() == m_vMapPlaneColors.size())
			glDisableClientState(GL_COLOR_ARRAY);
	}

	if (!m_vMapPlaneContuours.empty()) {
		for (int i = 0; i < (int)m_vMapPlaneContuours.size(); i++) {
			if (!m_vMapPlaneContuours[i].empty() && m_vMapPlaneContuours[i].size() % 3 == 0) {
				glColor4f(1.0, 0.0, 0.0, 0.5);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glEnableClientState(GL_VERTEX_ARRAY);
				glVertexPointer(3, GL_DOUBLE, 0, &m_vMapPlaneContuours[i][0]);
				glDrawArrays(GL_POLYGON, 0, (GLsizei)m_vMapPlaneContuours[i].size() / 3);
				glDisableClientState(GL_VERTEX_ARRAY);
				glDisable(GL_BLEND);
			}
		}
	}
	
	if (!m_vKeyFrameCameraPoses.empty()) {
		static double dbMinX = -CFrame::K.at(0, 2) *0.2 / CFrame::K.at(0, 0);
		static double dbMaxX =  CFrame::K.at(0, 2) *0.2 / CFrame::K.at(0, 0);
		static double dbMinY = -CFrame::K.at(1, 2) *0.2 / CFrame::K.at(1, 1);
		static double dbMaxY =  CFrame::K.at(1, 2) *0.2 / CFrame::K.at(1, 1);
		static double dbCamZ = 0.2;

		glColor3f(0, 0, 1.0);
		for (int i = 0; i < (int)m_vKeyFrameCameraPoses.size() / 16; i++) {
			glPushMatrix();
				glMultMatrixd(&m_vKeyFrameCameraPoses[i * 16]);

				glBegin(GL_LINES);
					glVertex3d(0, 0, 0);
					glVertex3d(dbMinX, dbMinY, dbCamZ);

					glVertex3d(0, 0, 0);
					glVertex3d(dbMaxX, dbMinY, dbCamZ);

					glVertex3d(0, 0, 0);
					glVertex3d(dbMaxX, dbMaxY, dbCamZ);

					glVertex3d(0, 0, 0);
					glVertex3d(dbMinX, dbMaxY, dbCamZ);

					glVertex3d(dbMinX, dbMinY, dbCamZ);
					glVertex3d(dbMaxX, dbMinY, dbCamZ);

					glVertex3d(dbMaxX, dbMinY, dbCamZ);
					glVertex3d(dbMaxX, dbMaxY, dbCamZ);

					glVertex3d(dbMaxX, dbMaxY, dbCamZ);
					glVertex3d(dbMinX, dbMaxY, dbCamZ);

					glVertex3d(dbMinX, dbMaxY, dbCamZ);
					glVertex3d(dbMinX, dbMinY, dbCamZ);
				glEnd();
			
			glPopMatrix();
		}

		///< render current camera pose
		glColor3f(0, 1.0, 0.0f);
		glPushMatrix();
			glMultMatrixd(m_pCurrCameraPose);

			glBegin(GL_LINES);
				glVertex3d(0, 0, 0);
				glVertex3d(dbMinX, dbMinY, dbCamZ);

				glVertex3d(0, 0, 0);
				glVertex3d(dbMaxX, dbMinY, dbCamZ);

				glVertex3d(0, 0, 0);
				glVertex3d(dbMaxX, dbMaxY, dbCamZ);

				glVertex3d(0, 0, 0);
				glVertex3d(dbMinX, dbMaxY, dbCamZ);

				glVertex3d(dbMinX, dbMinY, dbCamZ);
				glVertex3d(dbMaxX, dbMinY, dbCamZ);

				glVertex3d(dbMaxX, dbMinY, dbCamZ);
				glVertex3d(dbMaxX, dbMaxY, dbCamZ);

				glVertex3d(dbMaxX, dbMaxY, dbCamZ);
				glVertex3d(dbMinX, dbMaxY, dbCamZ);

				glVertex3d(dbMinX, dbMaxY, dbCamZ);
				glVertex3d(dbMinX, dbMinY, dbCamZ);
			glEnd();
		glPopMatrix();
	}

	if (!m_vKeyFrameConnections.empty()) {
		glLineWidth(2.0);
		glColor3f(0.0, 1.0, 1.0);
		for (int i = 0; i < (int)m_vKeyFrameConnections.size() / 6; i++) {
			glBegin(GL_LINES);
				glVertex3dv(&m_vKeyFrameConnections[(2*i) * 3]);
				glVertex3dv(&m_vKeyFrameConnections[(2*i+1) * 3]);
			glEnd();
		}
		/*glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_DOUBLE, 0, &m_vKeyFrameConnections[0]);
		glDrawArrays(GL_LINES, 0, (GLsizei)m_vKeyFrameConnections.size() / 6);
		glDisableClientState(GL_VERTEX_ARRAY);*/
	}

	if (!m_vLineCorrTmp.empty()) {
		glColor4f(1.0, 0.0, 0.0, 0.5);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glBegin(GL_TRIANGLES);
			glVertex3dv(&m_vLineCorrTmp[0]);
			glVertex3dv(&m_vLineCorrTmp[3]);
			glVertex3dv(&m_vLineCorrTmp[6]);

			glVertex3dv(&m_vLineCorrTmp[ 9]);
			glVertex3dv(&m_vLineCorrTmp[12]);
			glVertex3dv(&m_vLineCorrTmp[15]);
		glEnd();
		glDisable(GL_BLEND);
	}

	glLineWidth(1.0);
	RenderAxis();

	if (m_bRenderFloor)
		RenderFloor();

	glDisable(GL_DEPTH_TEST);
}

void	CWorldMapView::SetLineCorrTmp(std::vector<double>& vLineCorrTmp)
{
	m_vLineCorrTmp.resize(vLineCorrTmp.size());
	std::copy(vLineCorrTmp.begin(), vLineCorrTmp.end(), m_vLineCorrTmp.begin());
}

void	CWorldMapView::Clear(void)
{
	m_vMapPoints.clear();
	m_vCurrentKeyFrameMapPoints.clear();
	m_vCurrentFrameMapPoints.clear();
	m_vKeyFrameCameraPoses.clear();

	for (int i = 0; i < (int)m_vLocalMapPoints.size(); i++)
		m_vLocalMapPoints[i].clear();

	m_vLocalMapPoints.clear();
	m_vKeyFrameConnections.clear();
}

void	CWorldMapView::AddCameraPose(double* pCameraPose)
{
	for (int i = 0; i < 16; i++)
		m_vKeyFrameCameraPoses.push_back(pCameraPose[i]);
}

void	CWorldMapView::AddMapPoint(double* pMapPoint)
{
	m_vMapPoints.push_back(pMapPoint[0]);
	m_vMapPoints.push_back(pMapPoint[1]);
	m_vMapPoints.push_back(pMapPoint[2]);
}

void	CWorldMapView::AddMapPoints(std::vector<double>& vMapPoints)
{
	for (int i = 0; i < (int)vMapPoints.size(); i++)
		m_vMapPoints.push_back(vMapPoints[i]);
}

void	CWorldMapView::SetCurrentKeyFrameMapPoints(std::vector<double>& vCurrentKeyFrameMapPoints)
{
	m_vCurrentKeyFrameMapPoints.resize(vCurrentKeyFrameMapPoints.size());
	std::copy(vCurrentKeyFrameMapPoints.begin(), vCurrentKeyFrameMapPoints.end(), m_vCurrentKeyFrameMapPoints.begin());
}

void	CWorldMapView::SetCurrentTrackedMapPoints(std::vector<double>& vCurrentTrackedMapPoints)
{
	m_vCurrentFrameMapPoints.resize(vCurrentTrackedMapPoints.size());
	std::copy(vCurrentTrackedMapPoints.begin(), vCurrentTrackedMapPoints.end(), m_vCurrentFrameMapPoints.begin());
}

void	CWorldMapView::AddLocalMapPoints(std::vector<double>& vLocalMapPoints)
{
	m_vLocalMapPoints.push_back(vLocalMapPoints);
}

void	CWorldMapView::AddMapLine(double* pMapLine)
{
	for (int i = 0; i < 6; i++)
		m_vMapLines.push_back(*(pMapLine+i));
}

void	CWorldMapView::AddMapLines(std::vector<double>& vMapLines)
{
	for (int i = 0; i < (int)vMapLines.size(); i++)
		m_vMapLines.push_back(vMapLines[i]);
}

void	CWorldMapView::AddMapPlane(double* pMapPointOnPlane, double* pColors)
{
	m_vMapPlanes.push_back(pMapPointOnPlane[0]);
	m_vMapPlanes.push_back(pMapPointOnPlane[1]);
	m_vMapPlanes.push_back(pMapPointOnPlane[2]);

	m_vMapPlaneColors.push_back(pColors[0]);
	m_vMapPlaneColors.push_back(pColors[1]);
	m_vMapPlaneColors.push_back(pColors[2]);
}

void	CWorldMapView::AddMapPlanes(std::vector<double>& vMapPoints, std::vector<double>& vColors)
{
	for (int i = 0; i < (int)vMapPoints.size(); i++)
		m_vMapPlanes.push_back(vMapPoints[i]);
	for (int i = 0; i < (int)vColors.size(); i++)
		m_vMapPlaneColors.push_back(vColors[i]);
}

void	CWorldMapView::SetRenderLocalMapPoints(bool bRenderLocalMapPoints/* = false*/)
{
	bRenderLocalMapPoints = bRenderLocalMapPoints;
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

// VisualSLAMViewerView.h: CVisualSLAMViewerView 클래스의 인터페이스
//

#pragma once

#include "WorldMapView.h"
#include "Tracker.h"

class CVisualSLAMViewerView : public CView
{
private:
	bool	m_bFormCreated;
	CWorldMapView	m_oglWorldMapView;	///< map view class

	///@ {
	WSADATA	m_wsaData;
	SOCKET	m_hServerSocket;
	SOCKADDR_IN	m_hServerAddress;
	unsigned int	m_unServerPort;
	///@ }

	CTracker	m_visualTracker;
	CMapManager	m_mapManager;
	CWorldMap	m_worldMap;
	CImageGrabber	m_imgGrabber;

protected: // serialization에서만 만들어집니다.
	CVisualSLAMViewerView() noexcept;
	DECLARE_DYNCREATE(CVisualSLAMViewerView)

// 특성입니다.
public:
	CVisualSLAMViewerDoc* GetDocument() const;

// 작업입니다.
public:

// 재정의입니다.
public:
	virtual void OnDraw(CDC* pDC);  // 이 뷰를 그리기 위해 재정의되었습니다.
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// 구현입니다.
public:
	virtual ~CVisualSLAMViewerView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 생성된 메시지 맵 함수
protected:
	afx_msg void OnFilePrintPreview();
//	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
//	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);
	DECLARE_MESSAGE_MAP()

	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
public:
	afx_msg void OnChar(UINT nChar, UINT nRepCnt, UINT nFlags);
};

#ifndef _DEBUG  // VisualSLAMViewerView.cpp의 디버그 버전
inline CVisualSLAMViewerDoc* CVisualSLAMViewerView::GetDocument() const
   { return reinterpret_cast<CVisualSLAMViewerDoc*>(m_pDocument); }
#endif


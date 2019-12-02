
// VisualSLAMViewerView.cpp: CVisualSLAMViewerView 클래스의 구현
//

#include "stdafx.h"
// SHARED_HANDLERS는 미리 보기, 축소판 그림 및 검색 필터 처리기를 구현하는 ATL 프로젝트에서 정의할 수 있으며
// 해당 프로젝트와 문서 코드를 공유하도록 해 줍니다.
#ifndef SHARED_HANDLERS
#include "VisualSLAMViewer.h"
#endif

#include <Windows.h>

#include "mycv.h"
#include "armadillo"
#include "VisualSLAMConfigure.h"
#include "Frame.h"
#include "Tracker.h"
#include "boost/thread.hpp"
#include "boost/threadpool.hpp"

#pragma comment(lib, OPENCV_LIB_EXPAND("imgproc"))
#pragma comment(lib, OPENCV_LIB_EXPAND("imgcodecs"))
#pragma comment(lib, OPENCV_LIB_EXPAND("core"))
#pragma comment(lib, OPENCV_LIB_EXPAND("highgui"))
#pragma comment(lib, OPENCV_LIB_EXPAND("video"))
#pragma comment(lib, OPENCV_LIB_EXPAND("videoio"))
#pragma comment(lib, OPENCV_LIB_EXPAND("videostab"))
#pragma comment(lib, OPENCV_LIB_EXPAND("line_descriptor"))

#pragma comment(lib, "lapack_win64_MT.lib")
#pragma comment(lib, "blas_win64_MT.lib")

#ifdef _DEBUG
#pragma comment(lib, "g2o_core_d.lib")
#pragma comment(lib, "g2o_stuff_d.lib")
#pragma comment(lib, "g2o_types_slam3d_d.lib")
#pragma comment(lib, "g2o_types_sba_d.lib")
#else
#pragma comment(lib, "g2o_core.lib")
#pragma comment(lib, "g2o_stuff.lib")
#pragma comment(lib, "g2o_types_slam3d.lib")
#pragma comment(lib, "g2o_types_sba.lib")
#endif // DEBUG

#pragma comment(linker, "/ENTRY:wWinMainCRTStartup /subsystem:console")


#include "VisualSLAMViewerDoc.h"
#include "VisualSLAMViewerView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CVisualSLAMViewerView

IMPLEMENT_DYNCREATE(CVisualSLAMViewerView, CView)

BEGIN_MESSAGE_MAP(CVisualSLAMViewerView, CView)
	// 표준 인쇄 명령입니다.
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CVisualSLAMViewerView::OnFilePrintPreview)
	ON_WM_CREATE()
	ON_WM_SIZE()
//	ON_WM_CONTEXTMENU()
//	ON_WM_RBUTTONUP()
ON_WM_CHAR()
END_MESSAGE_MAP()

// CVisualSLAMViewerView 생성/소멸

CVisualSLAMViewerView::CVisualSLAMViewerView() noexcept
{
	// TODO: 여기에 생성 코드를 추가합니다.
	m_bFormCreated = false;

	arma::mat K(3, 3);
	arma::vec DistCoeff(5);
	arma::mat P(3, 4);

	if (!CSiftGPUWrapper::InitializeSiftGPU(2)) {
		std::cout << "cannot initialize gpu for sift key points" << std::endl;
		return;
	}

	CVisualSLAMConfigure::GetInstance().ReadWorkspace("D:/[Experimental]/[NTSD]/[NTSD]/workspace_onthefly.txt");
	CVisualSLAMConfigure::GetInstance().ReadCalibrationInfo(P, DistCoeff);

	K = P(arma::span(0, 2), arma::span(0, 2));

	CFrame::K = K;
	CFrame::dist = DistCoeff;

	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	m_imgGrabber.Initialize(vSLAMConfigure.m_nCameraType, vSLAMConfigure.m_nCaptureType);
	
	m_imgGrabber.SetImagePath(vSLAMConfigure.m_strProjectBasePath + "/" + vSLAMConfigure.m_strColorImagePath + "/00/",
							  vSLAMConfigure.m_strProjectBasePath + "/" + vSLAMConfigure.m_strDepthImagePath + "/00/");

	m_mapManager.SetWorldMap(&m_worldMap);
	m_visualTracker.PrepaerTracking(&m_imgGrabber, &m_mapManager, &m_oglWorldMapView, K, DistCoeff);
	
	arma::mat imgSize(2, 4);
	imgSize.at(0, 0) = 0;
	imgSize.at(1, 0) = 0;
	imgSize.at(0, 1) = 640;
	imgSize.at(1, 1) = 0;
	imgSize.at(0, 2) = 640;
	imgSize.at(1, 2) = 480;
	imgSize.at(0, 3) = 0;
	imgSize.at(1, 3) = 480;

	CFrame::ComputeImageBound(imgSize);
	/*tracker.Track();*/
}

CVisualSLAMViewerView::~CVisualSLAMViewerView()
{
	CSiftGPUWrapper::ReleaseSiftGPU();
}

BOOL CVisualSLAMViewerView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: CREATESTRUCT cs를 수정하여 여기에서
	//  Window 클래스 또는 스타일을 수정합니다.

	return CView::PreCreateWindow(cs);
}

// CVisualSLAMViewerView 그리기

void CVisualSLAMViewerView::OnDraw(CDC* /*pDC*/)
{
	CVisualSLAMViewerDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: 여기에 원시 데이터에 대한 그리기 코드를 추가합니다.
}


// CVisualSLAMViewerView 인쇄


void CVisualSLAMViewerView::OnFilePrintPreview()
{
#ifndef SHARED_HANDLERS
	AFXPrintPreview(this);
#endif
}

BOOL CVisualSLAMViewerView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 기본적인 준비
	return DoPreparePrinting(pInfo);
}

void CVisualSLAMViewerView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 인쇄하기 전에 추가 초기화 작업을 추가합니다.
}

void CVisualSLAMViewerView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 인쇄 후 정리 작업을 추가합니다.
}

//void CVisualSLAMViewerView::OnRButtonUp(UINT /* nFlags */, CPoint point)
//{
//	ClientToScreen(&point);
//	OnContextMenu(this, point);
//}
//
//void CVisualSLAMViewerView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
//{
//#ifndef SHARED_HANDLERS
//	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
//#endif
//}


// CVisualSLAMViewerView 진단

#ifdef _DEBUG
void CVisualSLAMViewerView::AssertValid() const
{
	CView::AssertValid();
}

void CVisualSLAMViewerView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CVisualSLAMViewerDoc* CVisualSLAMViewerView::GetDocument() const // 디버그되지 않은 버전은 인라인으로 지정됩니다.
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CVisualSLAMViewerDoc)));
	return (CVisualSLAMViewerDoc*)m_pDocument;
}
#endif //_DEBUG


// CVisualSLAMViewerView 메시지 처리기
int CVisualSLAMViewerView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here
	DWORD dwStyle = WS_VISIBLE | WS_CHILD | CS_OWNDC;

	CRect rect(0, 0, 200, 32);
	if (!m_bFormCreated) {
		m_oglWorldMapView.Create(0, _T("FAMirror View"), dwStyle, rect, this, 100);
		m_oglWorldMapView.ShowWindow(SW_SHOW);
		m_mapManager.SetWorldMapView(&m_oglWorldMapView);
		m_bFormCreated = true;
	}
	return 0;
}


void CVisualSLAMViewerView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	if (m_bFormCreated) {
		m_oglWorldMapView.MoveWindow(CRect(0, 0, cx, cy));
		m_oglWorldMapView.SetViewSize(cx, cy);
	}
}

void CVisualSLAMViewerView::OnChar(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if (nChar == 's') {
		boost::thread thVisualTracking, thMapManage;

		//thMapManage = boost::thread(&CMapManager::Run, &m_mapManager);
		thVisualTracking = boost::thread(&CTracker::Track, &m_visualTracker);

		std::cout << "start visual tracking" << std::endl;
	}
	else if (nChar == 'r') {
		//m_visualTracker.Clear();
		m_oglWorldMapView.Clear();
	}
	else if (nChar == ' ') {
		std::cout << "initializing" << std::endl;
		m_visualTracker.SetCurrentState(TrackingState::Initializing);
	}
	else if (nChar == 27) {
		m_visualTracker.SetCurrentState(TrackingState::Done);
		m_mapManager.SetStopFlag(true);
	}
	else if (nChar == 'm') {
		m_oglWorldMapView.ToggleRenderMapPoints();
		m_oglWorldMapView.Invalidate(FALSE);
	}
	else if (nChar == 'c') {
		m_oglWorldMapView.ToggleRenderCurrMapView();
		m_oglWorldMapView.Invalidate(FALSE);
	}
	else if (nChar == 'k') {
		m_oglWorldMapView.ToggleRenderCurrKeyMapView();
		m_oglWorldMapView.Invalidate(FALSE);
	}
	else if (nChar == 'f') {
		m_oglWorldMapView.ToggleRenderFloor();
		m_oglWorldMapView.Invalidate(FALSE);
	}
	else if (nChar == 'e') {
		m_oglWorldMapView.Clear();
		m_worldMap.Clear();
		m_visualTracker.Reset();
	}
	else if (nChar == 's') {
		m_visualTracker.SetCurrentState(TrackingState::Pause);
	}
	CView::OnChar(nChar, nRepCnt, nFlags);
}

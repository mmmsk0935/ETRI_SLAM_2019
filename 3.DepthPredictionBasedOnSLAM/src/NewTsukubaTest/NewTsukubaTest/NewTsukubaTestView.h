
// NewTsukubaTestView.h: CNewTsukubaTestView 클래스의 인터페이스
//

#pragma once
#include "NewTsukubaTestDoc.h"
#include "WorldMapView.h"
#include "armadillo"
#include "mycv.h"
#include "opencv.hpp"
#include "boost/thread/thread.hpp"
#include "SiftGPUWrapper.h"
#include "LineSegmentWrapper.h"
#include "EightPoint.h"

//#ifdef DEBUG
//#pragma comment(lib, "SiftGPU_d.lib")
//#else
//#pragma comment(lib, "SiftGPU.lib")
//#endif // DEBUG


#pragma comment(lib, OPENCV_LIB_EXPAND("imgproc"))
#pragma comment(lib, OPENCV_LIB_EXPAND("imgcodecs"))
#pragma comment(lib, OPENCV_LIB_EXPAND("core"))
#pragma comment(lib, OPENCV_LIB_EXPAND("highgui"))
#pragma comment(lib, OPENCV_LIB_EXPAND("line_descriptor"))

#pragma comment(lib, "lapack_win64_MT.lib")
#pragma comment(lib, "blas_win64_MT.lib")

class CNewTsukubaTestView : public CView
{
private:
	CWorldMapView	m_oglWorldMapView;
	bool	m_bFormCreated;

	void	RenderGroundTruth(void);
	void	FindGeometricMatch(const std::vector<SiftKeyPoint>& vKeyPoints1,
							   const std::vector<SiftKeyPoint>& vKeyPoints2,
							   const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
							   std::vector<int>& vInliers);
protected: // serialization에서만 만들어집니다.
	CNewTsukubaTestView() noexcept;
	DECLARE_DYNCREATE(CNewTsukubaTestView)

// 특성입니다.
public:
	CNewTsukubaTestDoc* GetDocument() const;

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
	virtual ~CNewTsukubaTestView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 생성된 메시지 맵 함수
protected:
	afx_msg void OnFilePrintPreview();
	/*afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnContextMenu(CWnd* pWnd, CPoint point);*/
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnChar(UINT nChar, UINT nRepCnt, UINT nFlags);
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // NewTsukubaTestView.cpp의 디버그 버전
inline CNewTsukubaTestDoc* CNewTsukubaTestView::GetDocument() const
   { return reinterpret_cast<CNewTsukubaTestDoc*>(m_pDocument); }
#endif


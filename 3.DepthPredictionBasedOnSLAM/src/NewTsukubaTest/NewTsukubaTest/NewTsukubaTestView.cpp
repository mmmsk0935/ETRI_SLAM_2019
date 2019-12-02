
// NewTsukubaTestView.cpp: CNewTsukubaTestView 클래스의 구현
//

#include "stdafx.h"
// SHARED_HANDLERS는 미리 보기, 축소판 그림 및 검색 필터 처리기를 구현하는 ATL 프로젝트에서 정의할 수 있으며
// 해당 프로젝트와 문서 코드를 공유하도록 해 줍니다.
#ifndef SHARED_HANDLERS
#include "NewTsukubaTest.h"
#endif

#include "NewTsukubaTestDoc.h"
#include "NewTsukubaTestView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#pragma comment(linker, "/ENTRY:wWinMainCRTStartup /subsystem:console")

// CNewTsukubaTestView

IMPLEMENT_DYNCREATE(CNewTsukubaTestView, CView)

BEGIN_MESSAGE_MAP(CNewTsukubaTestView, CView)
	// 표준 인쇄 명령입니다.
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CNewTsukubaTestView::OnFilePrintPreview)
	ON_WM_CREATE()
	ON_WM_SIZE()
	ON_WM_CHAR()
END_MESSAGE_MAP()

// CNewTsukubaTestView 생성/소멸

CNewTsukubaTestView::CNewTsukubaTestView() noexcept
{
	// TODO: 여기에 생성 코드를 추가합니다.
	m_bFormCreated = false;
	CSiftGPUWrapper::InitializeSiftGPU();

	typedef struct {
		int accNum;
		char name[20];
		float balance;
	}tAccount;

	FILE* f = fopen("account.bin", "wb+");
	int n;
	tAccount tempact;
	char line[100];
	int count = 0;
	while (1) {
		scanf("%d", &n);
		if (n == 1) {
			while (1) {
				gets(line, 100);
				if (!strcmp(line, "-1"))
					break;
				else {
					sscanf(line, "%d %s %lf", &tempact.accNum, tempact.name, &tempact.balance);
					fwrite(&tempact, sizeof(tAccount), 1, f);
					count++;
				}
			}
		}
	}
}

CNewTsukubaTestView::~CNewTsukubaTestView()
{
	CSiftGPUWrapper::ReleaseSiftGPU();
}

BOOL CNewTsukubaTestView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: CREATESTRUCT cs를 수정하여 여기에서
	//  Window 클래스 또는 스타일을 수정합니다.

	return CView::PreCreateWindow(cs);
}

// CNewTsukubaTestView 그리기

void CNewTsukubaTestView::OnDraw(CDC* /*pDC*/)
{
	CNewTsukubaTestDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: 여기에 원시 데이터에 대한 그리기 코드를 추가합니다.
}


// CNewTsukubaTestView 인쇄


void CNewTsukubaTestView::OnFilePrintPreview()
{
#ifndef SHARED_HANDLERS
	AFXPrintPreview(this);
#endif
}

BOOL CNewTsukubaTestView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// 기본적인 준비
	return DoPreparePrinting(pInfo);
}

void CNewTsukubaTestView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 인쇄하기 전에 추가 초기화 작업을 추가합니다.
}

void CNewTsukubaTestView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: 인쇄 후 정리 작업을 추가합니다.
}

//void CNewTsukubaTestView::OnRButtonUp(UINT /* nFlags */, CPoint point)
//{
//	ClientToScreen(&point);
//	OnContextMenu(this, point);
//}
//
//void CNewTsukubaTestView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
//{
//#ifndef SHARED_HANDLERS
//	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
//#endif
//}


// CNewTsukubaTestView 진단

#ifdef _DEBUG
void CNewTsukubaTestView::AssertValid() const
{
	CView::AssertValid();
}

void CNewTsukubaTestView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CNewTsukubaTestDoc* CNewTsukubaTestView::GetDocument() const // 디버그되지 않은 버전은 인라인으로 지정됩니다.
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CNewTsukubaTestDoc)));
	return (CNewTsukubaTestDoc*)m_pDocument;
}
#endif //_DEBUG


// CNewTsukubaTestView 메시지 처리기
int CNewTsukubaTestView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here
	DWORD dwStyle = WS_VISIBLE | WS_CHILD | CS_OWNDC;

	CRect rect(0, 0, 200, 32);
	if (!m_bFormCreated) {
		m_oglWorldMapView.Create(0, _T("FAMirror View"), dwStyle, rect, this, 100);
		m_oglWorldMapView.ShowWindow(SW_SHOW);
		m_bFormCreated = true;
	}
	return 0;
}


void CNewTsukubaTestView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	if (m_bFormCreated) {
		m_oglWorldMapView.MoveWindow(CRect(0, 0, cx, cy));
		m_oglWorldMapView.SetViewSize(cx, cy);
	}
}

void CNewTsukubaTestView::OnChar(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if (nChar == 's') {
		boost::thread thRenderGT;
		thRenderGT = boost::thread(&CNewTsukubaTestView::RenderGroundTruth, this);
	}
	CView::OnChar(nChar, nRepCnt, nFlags);
}

void	CNewTsukubaTestView::FindGeometricMatch(const std::vector<SiftKeyPoint>& vKeyPoints1,
												const std::vector<SiftKeyPoint>& vKeyPoints2,
												const std::vector<std::pair<int, int>>& vCoarseMatchIdxPair,
												std::vector<int>& vInliers)
{
	arma::mat SrcInitData(3, (int)vCoarseMatchIdxPair.size()), DstInitData(3, (int)vCoarseMatchIdxPair.size());
	Concurrency::parallel_for(0, (int)vCoarseMatchIdxPair.size(), [&](int i) {
		SrcInitData.at(0, i) = vKeyPoints1[vCoarseMatchIdxPair[i].first].x;
		SrcInitData.at(1, i) = vKeyPoints1[vCoarseMatchIdxPair[i].first].y;
		SrcInitData.at(2, i) = 1.0;

		DstInitData.at(0, i) = vKeyPoints2[vCoarseMatchIdxPair[i].second].x;
		DstInitData.at(1, i) = vKeyPoints2[vCoarseMatchIdxPair[i].second].y;
		DstInitData.at(2, i) = 1.0;
	});

	CEightPoint eight;
	eight.SetInitData(SrcInitData, DstInitData);
	eight.SetThreshold(5.0);
	eight.EstimateFundamentalMatrixByRANSAC();

	vInliers = eight.GetInlierDataIndices();
}


#define pi acos(-1.0)
#define	DEG2RAD(x) (x*pi/180)
#define RAD2DEG(x) (x*180/pi)

void	CNewTsukubaTestView::RenderGroundTruth(void)
{
	std::vector<arma::mat> vCameraGT(1800);
	cv::Mat depth = cv::Mat::zeros(480, 640, CV_32FC1);
	std::vector<double> vMapPoints(480 * 640 * 3);
	std::vector<double> vMapColors(480 * 640 * 3);
	cv::Mat img;
	std::ifstream fin("D:/[Experimental]/[NTSD]/[NTSD]/ground_truth.txt");

	arma::mat Ry;
	Ry.eye(4, 4);
	Ry.at(1, 1) = Ry.at(2, 2) = -1;

	for (int i = 0; i < 1800; i++) {
		
		/*for (int j = 0; j < 6; j++) {
			fin >> vCameraGT[i * 6 + j];
		}*/
		vCameraGT[i].eye(4, 4);
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 4; col++) {
				fin >> vCameraGT[i].at(row, col);
			}

			vCameraGT[i].at(row, 3) /= 10;
		}

		vCameraGT[i] = Ry * vCameraGT[i];
		/*vCameraGT[i] *= -1;
		vCameraGT[i].at(3, 3) = 1.0;*/
	}
	fin.close();
	
	/*arma::mat Rx, Ry, Rz, E, R, T;
	
	Rx.eye(4, 4); Ry.eye(4, 4); Rz.eye(4, 4); E.eye(4, 4); T.eye(4, 4);*/

	std::vector<std::pair<int, int>> vCoarseMatchPair;
	int nSrcKeyPointCount, nDstKeyPointCount;
	std::vector<SiftKeyPoint> vSrcKeyPoints, vDstKeyPoints;
	std::vector<float> vSrcDescriptors, vDstDescriptors;
	std::vector<int> vInliers;
	arma::mat K;
	arma::mat P1(3, 4), P2(3, 4);

	K.eye(3, 3);
	K.at(0, 0) = K.at(1, 1) = 615;
	K.at(0, 2) = 320;
	K.at(1, 2) = 240;

	for (int i = 0; i < 1760; i++) {
		char imgNum[20];
		sprintf_s(imgNum, "%06d", i);
		std::string str = "D:/[Experimental]/[NTSD]/[NTSD]/[Image]/00/" + std::string(imgNum) + ".png";
		img = cv::imread(str);
		str = "D:/[Experimental]/[NTSD]/[NTSD]/[DepthMap]/00/" + std::string(imgNum) + ".depth";
		fin.open(str, ios::binary);
		fin.read((char*)depth.data, sizeof(float) * 480 * 640);
		fin.close();

		str = "D:/[Experimental]/[NTSD]/[NTSD]/[Feature]/[PointFeature]/00/" + std::string(imgNum) + ".feat";
		fin.open(str, ios::binary);
		fin.read((char *)&nSrcKeyPointCount, sizeof(int));

		vSrcKeyPoints.resize(nSrcKeyPointCount);
		vSrcDescriptors.resize(nSrcKeyPointCount * 128);

		fin.read((char *)&vSrcKeyPoints[0], sizeof(SiftKeyPoint) * nSrcKeyPointCount);
		fin.read((char *)&vSrcDescriptors[0], sizeof(float) * nSrcKeyPointCount * 128);
		
		fin.close();
		
		/*double a, b, c;
		a = DEG2RAD(vCameraGT[i * 6 + 3]);
		b = DEG2RAD(vCameraGT[i * 6 + 4]);
		c = DEG2RAD(vCameraGT[i * 6 + 5]);
		
		double ca, sa, cb, sb, cc, sc;
		ca = cos(a); sa = sin(a);
		cb = cos(b); sb = sin(b);
		cc = cos(c); sc = sin(c);

		Rx.at(1, 1) = ca; Rx.at(2, 2) = ca; Rx.at(1, 2) = -sa; Rx.at(2, 1) = sa;
		Ry.at(0, 0) = cb; Ry.at(2, 2) = cb; Ry.at(0, 2) = sb; Ry.at(2, 0) = -sb;
		Rz.at(0, 0) = cc; Rz.at(1, 1) = cc; Rz.at(0, 1) = -sc; Rz.at(1, 0) = sc;

		T.at(0, 3) = vCameraGT[i * 6    ] / 10;
		T.at(1, 3) = vCameraGT[i * 6 + 1] / 10;
		T.at(2, 3) = vCameraGT[i * 6 + 2] / 10;

		E = T*Ry*Rx*Rz;*/

		arma::mat R = vCameraGT[i](arma::span(0, 2), arma::span(0, 2));
		arma::vec t = vCameraGT[i](arma::span(0, 2), arma::span(3, 3));

		P1(arma::span(0, 2), arma::span(0, 2)) = R.t();
		P1(arma::span(0, 2), arma::span(3, 3)) = -R.t()*t;
		P1 = K * P1;
		/*arma::mat R1 = R;
		arma::vec t1 = t;

		R = R.t();
		t = -R * t;

		P1 = vCameraGT[i](arma::span(0, 2), arma::span(0, 3));
		P1 = K * P1;*/
		for (int row = 0; row < 480; row++) {
			for (int col = 0; col < 640; col++) {
				int idx = row * 640 + col;
				double z = depth.at<float>(row, col) / 10;
				double x = (col - 320) / 615.0 * z;
				double y = (row - 240) / 615.0 * z;

				cv::Vec3b color = img.at<cv::Vec3b>(row, col);

				double b = color[0] / 255.0;
				double g = color[1] / 255.0;
				double r = color[2] / 255.0;

				vMapPoints[idx * 3] = x;
				vMapPoints[idx * 3 + 1] = y;
				vMapPoints[idx * 3 + 2] = z;

				vMapColors[idx * 3] = r;
				vMapColors[idx * 3 + 1] = g;
				vMapColors[idx * 3 + 2] = b;
			}
		}

		for (int j = 10; j < 11; j++) {
			sprintf_s(imgNum, "%06d", i + j);
			str = "D:/[Experimental]/[NTSD]/[NTSD]/[Feature]/[PointFeature]/00/" + std::string(imgNum) + ".feat";
			fin.open(str, ios::binary);
			fin.read((char *)&nDstKeyPointCount, sizeof(int));

			vDstKeyPoints.resize(nDstKeyPointCount);
			vDstDescriptors.resize(nDstKeyPointCount * 128);

			fin.read((char *)&vDstKeyPoints[0], sizeof(SiftKeyPoint) * nDstKeyPointCount);
			fin.read((char *)&vDstDescriptors[0], sizeof(float) * nDstKeyPointCount * 128);

			fin.close();

			CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vCoarseMatchPair);

			if (vCoarseMatchPair.size() < 50)
				continue;

			FindGeometricMatch(vSrcKeyPoints, vDstKeyPoints, vCoarseMatchPair, vInliers);

			//P2 = vCameraGT[i + j](arma::span(0, 2), arma::span(0, 3));

			arma::mat R2 = vCameraGT[i + j](arma::span(0, 2), arma::span(0, 2));
			arma::vec t2 = vCameraGT[i + j](arma::span(0, 2), arma::span(3, 3));

			P2(arma::span(0, 2), arma::span(0, 2)) = R2.t();
			P2(arma::span(0, 2), arma::span(3, 3)) = -R2.t()*t2;

			P2 = K * P2;
			arma::mat A(4, 4), U, V;
			arma::vec s, x1(3), x2(3);
			for (int k = 0; k < (int)vInliers.size(); k++) {
				SiftKeyPoint& key1 = vSrcKeyPoints[vCoarseMatchPair[vInliers[k]].first];
				SiftKeyPoint& key2 = vDstKeyPoints[vCoarseMatchPair[vInliers[k]].second];

				A.row(0) = key1.x * P1.row(2) - P1.row(0);
				A.row(1) = key1.y * P1.row(2) - P1.row(1);
				A.row(2) = key2.x * P2.row(2) - P2.row(0);
				A.row(3) = key2.y * P2.row(2) - P2.row(1);

				arma::svd(U, s, V, A);
				
				x1.at(0) = V.at(0, 3);
				x1.at(1) = V.at(1, 3);
				x1.at(2) = V.at(2, 3);
				x1 /= V.at(3, 3);
				
				int nX1 = (int)key1.x;	int nX2 = nX1+1;
				int nY1 = (int)key1.y;	int nY2 = nY1+1;

			
				double dbDepth1 = depth.at<float>(nY1, nX1) / 10;
				double dbDepth2 = depth.at<float>(nY1, nX2) / 10;
				double dbDepth3 = depth.at<float>(nY2, nX1) / 10;
				double dbDepth4 = depth.at<float>(nY2, nX2) / 10;

				double dbXRatio = key1.x - nX1;
				double dbYRatio = key1.y - nY1;

				double dbDepth = (1 - dbYRatio)*((1 - dbXRatio)*dbDepth1 + dbXRatio * dbDepth2) + 
									  dbYRatio *((1 - dbXRatio)*dbDepth3 + dbXRatio * dbDepth4);

				x2.at(2) = dbDepth;
				x2.at(0) = (key1.x - 320) * dbDepth / 615;
				x2.at(1) = (key1.y - 240) * dbDepth / 615;
				
				x2 = R * x2 + t;

				arma::vec n1 = x1 - t;
				arma::vec n2 = x1 - t2;

				n1 /= arma::norm(n1);
				n2 /= arma::norm(n2);

				double dbCos = RAD2DEG(acos(arma::dot(n1, n2)));
				double dbDistDiff = arma::norm(x2 - x1);

				std::cout << "[" << key1.x << ",  " << key1.y << "]->["
						  << key2.x << ",  " << key2.y << "]: "
						  << "parallex: " << dbCos << " distance diff: " << dbDistDiff << std::endl;
			}
		}
		//m_oglWorldMapView.SetCameraPose(E.memptr());
		/*m_oglWorldMapView.SetCameraPose(vCameraGT[i].memptr());
		m_oglWorldMapView.SetLocalMap(vMapPoints, vMapColors);
		m_oglWorldMapView.Invalidate(FALSE);*/
	}
}
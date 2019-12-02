#include "stdafx.h"

#include <iostream>
#include <fstream>
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

#pragma comment(lib, "lapack_win64_MT.lib")
#pragma comment(lib, "blas_win64_MT.lib")

int main(int argc, char* argv[])
{
	arma::mat K(3, 3);
	arma::vec DistCoeff(5);
	arma::mat P(3, 4);

	if (!CSiftGPUWrapper::InitializeSiftGPU()) {
		std::cout << "cannot initialize gpu for sift key points" << std::endl;
		return -1;
	}

	CVisualSLAMConfigure::GetInstance().ReadWorkspace("D:/[Experimental]/[NTSD]/[NTSD]/workspace.txt");
	CVisualSLAMConfigure::GetInstance().ReadCalibrationInfo(P, DistCoeff);
	
	K = P(arma::span(0, 2), arma::span(0, 2));

	CFrame::K = K;
	CFrame::dist = DistCoeff;

	CVisualSLAMConfigure& vSLAMConfigure = CVisualSLAMConfigure::GetInstance();
	CImageGrabber imgSource(vSLAMConfigure.m_nCameraType, vSLAMConfigure.m_nCaptureType);
	CWorldMap worldMap;
	CMapManager	mapManager(&worldMap);

	imgSource.SetImagePath(vSLAMConfigure.m_strProjectBasePath + "/" + vSLAMConfigure.m_strColorImagePath + "/00/", 
						   vSLAMConfigure.m_strProjectBasePath + "/" + vSLAMConfigure.m_strDepthImagePath + "/00/");
	CTracker tracker(&imgSource, &mapManager, K, DistCoeff);
	tracker.Track();

	CSiftGPUWrapper::ReleaseSiftGPU();
	return 0;
}
#include <iostream>
#include <fstream>
#include <Windows.h>

#include "mycv.h"
#include "opencv2/line_descriptor.hpp"
#include "armadillo"
#include "VisualSLAMConfigure.h"
#include "Frame.h"
#include "Tracker.h"
#include "LineSegmentWrapper.h"
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

#include <cstdio>

int main(int argc, char* argv[])
{
	arma::mat K(3, 3);
	arma::vec DistCoeff(5);
	arma::mat P(3, 4);

	if (!CSiftGPUWrapper::InitializeSiftGPU()) {
		std::cout << "cannot initialize gpu for sift key points" << std::endl;
		return -1;
	}

	/*CVisualSLAMConfigure::GetInstance().ReadWorkspace("D:/[Experimental]/[NTSD]/[NTSD]/workspace.txt");
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
	tracker.Track();*/

	cv::Mat img1 = cv::imread("D:/[Experimental]/[NTSD]/[NTSD]/[Image]/00/000000.png", 1);
	cv::Mat img2 = cv::imread("D:/[Experimental]/[NTSD]/[NTSD]/[Image]/00/000150.png", 1);
	cv::Mat grayImg1, grayImg2, img3;

	cv::cvtColor(img1, grayImg1, cv::COLOR_BGR2GRAY);
	cv::cvtColor(img2, grayImg2, cv::COLOR_BGR2GRAY);

	std::vector<SiftKeyPoint> vSrcKeyPoints, vDstKeyPoints;
	std::vector<float> vSrcDescriptors, vDstDescriptors;
	std::vector<cv::line_descriptor::KeyLine> vSrcKeyLines, vDstKeyLines;
	cv::Mat cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors;
	std::vector<std::pair<int, int>> vKeyPointCoarseMatchPair, vKeyLineCoarseMatchPair;

	CSiftGPUWrapper::ExtractSiftFeature((char *)grayImg1.data, img1.cols, img1.rows, vSrcKeyPoints, vSrcDescriptors);
	CSiftGPUWrapper::ExtractSiftFeature((char *)grayImg2.data, img2.cols, img2.rows, vDstKeyPoints, vDstDescriptors);
	CSiftGPUWrapper::FindCoarseMatch(vSrcDescriptors, vDstDescriptors, vKeyPointCoarseMatchPair);

	cv::Mat tmp;
	cv::hconcat(img1, img2, tmp);
	arma::mat SrcInitData, DstInitData;
	for (int i = 0; i < (int)vKeyPointCoarseMatchPair.size(); i++) {
		SiftKeyPoint key1 = vSrcKeyPoints[vKeyPointCoarseMatchPair[i].first];
		SiftKeyPoint key2 = vDstKeyPoints[vKeyPointCoarseMatchPair[i].second];

		cv::Point pt1(key1.x, key1.y);
		cv::Point pt2(key2.x + img1.cols, key2.y);
		cv::circle(tmp, pt1, 3, cv::Scalar(0, 0, 255), 2);
		cv::circle(tmp, pt2, 3, cv::Scalar(0, 0, 255), 2);
		cv::line(tmp, pt1, pt2, cv::Scalar(0, 0, 255), 1);

		arma::vec x1(3), x2(3);
		x1.at(0) = key1.x; x1.at(1) = key1.y; x1.at(2) = 1.0;
		x2.at(0) = key2.x; x2.at(1) = key2.y; x2.at(2) = 1.0;
		SrcInitData.insert_cols(SrcInitData.n_cols, x1);
		DstInitData.insert_cols(DstInitData.n_cols, x2);
	}
	cv::imshow("correspondences", tmp);
	cv::waitKey(0);
	
	CEightPoint eight;
	eight.SetInitData(SrcInitData, DstInitData);
	eight.SetThreshold(3.0);
	eight.EstimateFundamentalMatrixByRANSAC();

	const std::vector<int>& vInliers = eight.GetInlierDataIndices();
	for (int i = 0; i < (int)vInliers.size(); i++) {
		SiftKeyPoint key1 = vSrcKeyPoints[vKeyPointCoarseMatchPair[vInliers[i]].first];
		SiftKeyPoint key2 = vDstKeyPoints[vKeyPointCoarseMatchPair[vInliers[i]].second];

		cv::Point pt1(key1.x, key1.y);
		cv::Point pt2(key2.x + img1.cols, key2.y);
		cv::circle(tmp, pt1, 3, cv::Scalar(0, 255, 0), 2);
		cv::circle(tmp, pt2, 3, cv::Scalar(0, 255, 0), 2);
		cv::line(tmp, pt1, pt2, cv::Scalar(0, 255, 0), 2);

		arma::vec x1(3), x2(3);
		x1.at(0) = key1.x; x1.at(1) = key1.y; x1.at(2) = 1.0;
		x2.at(0) = key2.x; x2.at(1) = key2.y; x2.at(2) = 1.0;
		SrcInitData.insert_cols(SrcInitData.n_cols, x1);
		DstInitData.insert_cols(DstInitData.n_cols, x2);
	}
	cv::imshow("correspondences", tmp);
	cv::waitKey(0);

	CLineSegmentWrapper::ExtractLineSegment(img1, vSrcKeyLines, cvSrcKeyLineDescriptors);
	CLineSegmentWrapper::ExtractLineSegment(img2, vDstKeyLines, cvDstKeyLineDescriptors);

	CLineSegmentWrapper::FindCoarseMatch(vSrcKeyLines, vDstKeyLines, cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vKeyLineCoarseMatchPair,
										 vSrcKeyPoints, vDstKeyPoints, vKeyPointCoarseMatchPair);

	cv::hconcat(img1, img2, img3);
	cv::RNG rng(12345);
	for (int i = 0; i < (int)vKeyLineCoarseMatchPair.size(); i++) {
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv::Point pt1, pt2, pt3, pt4, pt5, pt6;

		pt1.x = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].startPointX);
		pt1.y = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].startPointY);

		pt2.x = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].endPointX);
		pt2.y = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].endPointY);

		pt3.x = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].pt.x);
		pt3.y = cvRound(vSrcKeyLines[vKeyLineCoarseMatchPair[i].first].pt.y);

		pt4.x = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].startPointX) + img1.cols;
		pt4.y = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].startPointY);

		pt5.x = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].endPointX) + img1.cols;
		pt5.y = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].endPointY);

		pt6.x = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].pt.x) + img1.cols;
		pt6.y = cvRound(vDstKeyLines[vKeyLineCoarseMatchPair[i].second].pt.y);

		cv::line(img3, pt1, pt2, color, 2);
		cv::line(img3, pt4, pt5, color, 2);
		cv::line(img3, pt3, pt6, color, 2);
	}

	cv::imshow("line matching", img3);
	cv::waitKey(0);

	CSiftGPUWrapper::ReleaseSiftGPU();
	return 0;
}
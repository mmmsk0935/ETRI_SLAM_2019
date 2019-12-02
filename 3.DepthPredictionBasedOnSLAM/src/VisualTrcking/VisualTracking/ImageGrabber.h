#pragma once

#include <iostream>
#include <fstream>
#include <string>

#include "VisualSLAMConfigure.h"
#include "mycv.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"

class CImageGrabber
{
private:
	int	m_nCameraType;
	int	m_nVideoCaptureType;
	
	std::string	m_strColorImagePath;
	std::string	m_strDepthImagePath;
	cv::VideoCapture*	m_pVideoCapture;
public:
	CImageGrabber(int nCameraType = CameraType::Monocular, int nVideoCaptureType = VideoCaptureType::ImageSet);
	~CImageGrabber();

	void	SetImagePath(std::string& strColorImagePath, std::string& strDepthImagePath);
	cv::Mat	GetNextColorFrame(void);
	cv::Mat	GetNextDepthFrame(void);

	cv::Mat	GetNextColorFrame(int nCurrFrameNumber);
	cv::Mat	GetNextDepthFrame(int nCurrFrameNumber);
};


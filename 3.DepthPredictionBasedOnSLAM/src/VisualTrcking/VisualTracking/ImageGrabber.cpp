#include "ImageGrabber.h"

CImageGrabber::CImageGrabber(int nCameraType/* = CameraType::Monocular*/, int nVideoCaptureType/* = VideoCaptureType::ImageSet*/)
	: m_nCameraType(nCameraType), m_nVideoCaptureType(nVideoCaptureType)
{
	switch (m_nCameraType) {
	case 0:
		m_pVideoCapture = new cv::VideoCapture(0);
		if (m_pVideoCapture->isOpened())
			std::cout << "webcam is connected..." << std::endl;
		break;
	}
}

CImageGrabber::~CImageGrabber()
{
	switch (m_nCameraType) {
	case 0:
		m_pVideoCapture->release();
		delete m_pVideoCapture;
		break;
	}
}

void	CImageGrabber::SetImagePath(std::string& strColorImagePath, std::string& strDepthImagePath)
{
	m_strColorImagePath = strColorImagePath;
	m_strDepthImagePath = strDepthImagePath;
}

cv::Mat	CImageGrabber::GetNextColorFrame(void)
{
	cv::Mat img;
	(*m_pVideoCapture) >> img;
	return img;
}

cv::Mat	CImageGrabber::GetNextDepthFrame(void)
{
	char pCurrFrameNumber[255];
	cv::Mat img;

	switch (m_nCameraType) {
	case RGBD:
		/*sprintf_s(pCurrFrameNumber, "%06d", m_nCurrFrameNumber);
		std::string strDepthFile = m_strDepthImagePath + "/" + std::string(pCurrFrameNumber) + ".depth";
		img = cv::Mat::zeros(480, 640, CV_32FC1);
		std::ifstream fin(strDepthFile.c_str(), std::ios::binary);
		fin.read((char *)img.data, sizeof(float) * 480 * 640);
		fin.close();*/
		break;
	}
	return img;
}

cv::Mat	CImageGrabber::GetNextColorFrame(int nCurrFrameNumber)
{
	char pCurrFrameNumber[255];
	sprintf_s(pCurrFrameNumber, "%06d", nCurrFrameNumber);
	std::string strImageFile = m_strColorImagePath + "/" + std::string(pCurrFrameNumber) + ".png";
	cv::Mat cvColorImage = cv::imread(strImageFile.c_str(), 1);
	return cvColorImage.clone();
}
cv::Mat	CImageGrabber::GetNextDepthFrame(int nCurrFrameNumber)
{
	char pCurrFrameNumber[255];
	cv::Mat img;
	
	sprintf_s(pCurrFrameNumber, "%06d", nCurrFrameNumber);
	std::string strDepthFile = m_strDepthImagePath + "/" + std::string(pCurrFrameNumber) + ".depth";
	img = cv::Mat::zeros(480, 640, CV_32FC1);
	std::ifstream fin(strDepthFile.c_str(), std::ios::binary);
	fin.read((char *)img.data, sizeof(float) * 480 * 640);
	fin.close();
	
	return img;
}
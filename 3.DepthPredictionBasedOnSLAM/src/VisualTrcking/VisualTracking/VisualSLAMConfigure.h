#pragma once
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string.h>

#include "armadillo"

enum PointFeatureType
{
	SIFT_GPU, ORB
};

enum CameraType
{
	Monocular, Stereo, RGBD
};

enum VideoCaptureType
{
	ImageSet, OnTheFly
};

enum TrackingState
{
	FirstFrame, Initializing, Tracking, TrackingLoss, Done
};

typedef	struct _sift_key_point
{
	float x, y, s, o;
} SiftKeyPoint;

typedef	struct _world_point
{
	double x, y, z;
} WorldPoint;

#ifndef BORDER_SIZE
#define	BORDER_SIZE	10
#endif // !BORDER_SIZE


class CVisualSLAMConfigure
{
public:
	CVisualSLAMConfigure();
	~CVisualSLAMConfigure();

	int	m_nCameraType;
	int	m_nCaptureType;
	int	m_nPointFeatureType;
	
	std::string	m_strProjectBasePath;
	std::string	m_strCalibInfoFile;
	std::string	m_strColorImagePath;
	std::string	m_strDepthImagePath;
	std::string	m_strPointFeaturePath;
	std::string	m_strLineFeaturePath;
	
	static	CVisualSLAMConfigure&	GetInstance() {
		static CVisualSLAMConfigure* pInstance = new CVisualSLAMConfigure();
		return *pInstance;
	}

	void	ReadWorkspace(const std::string& strWorkspace) {
		char pOption[255], pParams[255];
		std::ifstream fin(strWorkspace.c_str());

		while (fin.peek() != EOF) {
			fin >> pOption >> pParams;
			if (!strcmp(pOption, "-camera_type")) {
				m_nCameraType = std::atoi(pParams);
				if (m_nCameraType < CameraType::Monocular || m_nCameraType > CameraType::RGBD) {
					std::cerr << "there is no supported camera for " << m_nCameraType << " type" << std::endl;
					std::cerr << "0: for monocular camera, 1: for stereo camera, 2: RGBD camera type" << std::endl;
					return;
				}
			}
			else if (!strcmp(pOption, "-capture_type")) {
				m_nCaptureType = std::atoi(pParams);
				if (m_nCaptureType < VideoCaptureType::ImageSet || m_nCaptureType > VideoCaptureType::OnTheFly) {
					std::cerr << "there is no supported video capture for " << m_nCaptureType << " type" << std::endl;
					std::cerr << "0: for image set, 1: for real-time camera" << std::endl;
					return;
				}
			}
			else if (!strcmp(pOption, "-point_feature_type")) {
				m_nPointFeatureType = std::atoi(pParams);
				if (m_nPointFeatureType > PointFeatureType::ORB || m_nPointFeatureType < PointFeatureType::SIFT_GPU) {
					std::cerr << "there is no supported point feature for " << m_nPointFeatureType << " type" << std::endl;
					std::cerr << "0: for ORB blob feature, 1: for SIFT(GPU supported) blob feature" << std::endl;
					return;
				}
			}
			else if (!strcmp(pOption, "-calib_info_file"))
				m_strCalibInfoFile = pParams;
			else if (!strcmp(pOption, "-base_path"))
				m_strProjectBasePath = pParams;
			else if (!strcmp(pOption, "-image_path"))
				m_strColorImagePath = pParams;
			else if (!strcmp(pOption, "-depth_path"))
				m_strDepthImagePath = pParams;
			else if (!strcmp(pOption, "-point_feature_paht"))
				m_strPointFeaturePath = pParams;
			else if (!strcmp(pOption, "-line_feature_path"))
				m_strLineFeaturePath = pParams;
		}
		fin.close();

		///< validation of input parameters for visual tracking
		DWORD dFileType = GetFileAttributesA(m_strProjectBasePath.c_str());
		if (dFileType == INVALID_FILE_ATTRIBUTES || !(dFileType & FILE_ATTRIBUTE_DIRECTORY)) {
			std::cerr << "there is no directory for visual tracking workspace base path " << m_strProjectBasePath << std::endl;
			std::cerr << "Please, check the existence of project base path" << std::endl;
			return;
		}
		
		if (m_nCaptureType == VideoCaptureType::ImageSet) {
			std::string strColorImageAbsPath = m_strProjectBasePath + "/" + m_strColorImagePath;
			dFileType = GetFileAttributesA(strColorImageAbsPath.c_str());
			if (dFileType == INVALID_FILE_ATTRIBUTES || !(dFileType & FILE_ATTRIBUTE_DIRECTORY)) {
				std::cerr << "there is no directory for image sequence path " << strColorImageAbsPath << std::endl;
				std::cerr << "Please, check the existence of image sequence path" << std::endl;
				return;
			}

			std::string strDepthImageAbsPath = m_strProjectBasePath + "/" + m_strDepthImagePath;
			dFileType = GetFileAttributesA(strDepthImageAbsPath.c_str());
			if (dFileType == INVALID_FILE_ATTRIBUTES || !(dFileType & FILE_ATTRIBUTE_DIRECTORY)) {
				std::cerr << "there is no directory for depth sequence path " << strDepthImageAbsPath << std::endl;
				std::cerr << "Please, check the existence of depth sequence path" << std::endl;
				return;
			}
		}

		std::string strPointFeatureAbsPath = m_strProjectBasePath + "/" + m_strPointFeaturePath;
		dFileType = GetFileAttributesA(strPointFeatureAbsPath.c_str());
		if (dFileType == INVALID_FILE_ATTRIBUTES || !(dFileType & FILE_ATTRIBUTE_DIRECTORY)) {
			std::cerr << "there is no directory to save point feature data path " << strPointFeatureAbsPath << std::endl;
			std::cerr << "Please, check the existence of point feature data path" << std::endl;
			return;
		}

		std::string strLineFeatureAbsPath = m_strProjectBasePath + "/" + m_strLineFeaturePath;
		dFileType = GetFileAttributesA(strLineFeatureAbsPath.c_str());
		if (dFileType == INVALID_FILE_ATTRIBUTES || !(dFileType & FILE_ATTRIBUTE_DIRECTORY)) {
			std::cerr << "there is no directory to save point line data path " << strLineFeatureAbsPath << std::endl;
			std::cerr << "Please, check the existence of point line data path" << std::endl;
			return;
		}
	}

	void	ReadCalibrationInfo(arma::mat& P, arma::vec& DistCoeff) {
		std::string strCalibInfoFile = m_strProjectBasePath + "/" + m_strCalibInfoFile;
		std::ifstream fin(strCalibInfoFile.c_str());

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				fin >> P.at(i, j);
			}
		}
		for (int i = 0; i < 5; i++)
			fin >> DistCoeff.at(i);
		fin.close();
	}

	void	ReadCalibrationInfo(arma::mat& P1, arma::mat& P2) {
		std::string strCalibInfoFile = m_strProjectBasePath + "/" + m_strCalibInfoFile;
		std::ifstream fin(strCalibInfoFile.c_str());

		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				fin >> P1.at(3, 4);
			}
		}
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				fin >> P2.at(3, 4);
			}
		}

		fin.close();
	}
};
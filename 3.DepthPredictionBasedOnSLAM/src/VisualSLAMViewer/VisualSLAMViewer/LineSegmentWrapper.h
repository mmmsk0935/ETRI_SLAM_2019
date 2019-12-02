#pragma once

#include "VisualSLAMConfigure.h"
#include "opencv2/line_descriptor.hpp"

class CLineSegmentWrapper
{
public:
	cv::Ptr<cv::line_descriptor::LSDDetector>				m_pLSD;
	cv::Ptr<cv::line_descriptor::BinaryDescriptor>			m_pLSDDescriptors;
	cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>	m_pLineSegmentMatcher;
public:
	CLineSegmentWrapper();
	~CLineSegmentWrapper();
	
	void	ExtractLineSegment(const cv::Mat& cvColorImage, std::vector<cv::line_descriptor::KeyLine>& vKeyLines, cv::Mat& cvKeyLineDescriptors);
	void	FindCoarseMatch(std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines, std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines,
							cv::Mat& cvSrcKeyLineDescriptors, cv::Mat& cvDstKeyLineDescriptors, std::vector<std::pair<int, int>>& vKeyLineCoarseMatchPair,
							const std::vector<SiftKeyPoint>& vSrcKeyPoints, const std::vector<SiftKeyPoint>& vDstKeyPoints, std::vector<std::pair<int, int>>& vKeyPointMatchPair);

	std::vector<int> GetBoundedKeyPoints(const cv::line_descriptor::KeyLine& keyLine, const std::vector<SiftKeyPoint>& vKeyPoints);

	void	FindCoarseMatch(cv::Mat cvSrcKeyLineDescriptors, cv::Mat cvDstKeyLineDescriptors, std::vector<std::pair<int, int>>& vKeyLineCoarseMatchPair);
};


#pragma once

#include "VisualSLAMConfigure.h"
#include "opencv2/line_descriptor.hpp"

class CLineSegmentWrapper
{
public:
	static	cv::Ptr<cv::line_descriptor::BinaryDescriptor>			m_pLineSegmentDetector;
	static	cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>	m_pLineSegmentMatcher;
public:
	CLineSegmentWrapper();
	~CLineSegmentWrapper();
	
	static	void	ExtractLineSegment(const cv::Mat& cvColorImage, std::vector<cv::line_descriptor::KeyLine>& vKeyLines, cv::Mat& cvKeyLineDescriptors);
	static	void	FindCoarseMatch(std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines, std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines,
									cv::Mat& cvSrcKeyLineDescriptors, cv::Mat& cvDstKeyLineDescriptors, std::vector<std::pair<int, int>>& vKeyLineCoarseMatchPair,
									std::vector<SiftKeyPoint>& vSrcKeyPoints, std::vector<SiftKeyPoint>& vDstKeyPoints, std::vector<std::pair<int, int>>& vKeyPointMatchPair);

	static	std::vector<int> GetBoundedKeyPoints(const cv::line_descriptor::KeyLine& keyLine, const std::vector<SiftKeyPoint>& vKeyPoints);
};


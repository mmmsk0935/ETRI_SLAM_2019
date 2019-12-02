#include "LineSegmentWrapper.h"

cv::Ptr<cv::line_descriptor::BinaryDescriptor>			CLineSegmentWrapper::m_pLineSegmentDetector = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher>	CLineSegmentWrapper::m_pLineSegmentMatcher  = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

CLineSegmentWrapper::CLineSegmentWrapper()
{
}

CLineSegmentWrapper::~CLineSegmentWrapper()
{
}

void	CLineSegmentWrapper::ExtractLineSegment(const cv::Mat& cvColorImage, std::vector<cv::line_descriptor::KeyLine>& vKeyLines, cv::Mat& cvKeyLineDescriptors)
{
	cv::Mat cvGrayImage, cvMaskImage = cv::Mat::ones(cvColorImage.size(), CV_8UC1);
	cv::cvtColor(cvColorImage, cvGrayImage, cv::COLOR_BGR2GRAY);

	m_pLineSegmentDetector->detect(cvColorImage, vKeyLines, cvMaskImage);
	m_pLineSegmentDetector->compute(cvColorImage, vKeyLines, cvKeyLineDescriptors);
}

void	CLineSegmentWrapper::FindCoarseMatch(std::vector<cv::line_descriptor::KeyLine>& vSrcKeyLines, 
											 std::vector<cv::line_descriptor::KeyLine>& vDstKeyLines,
											 cv::Mat& cvSrcKeyLineDescriptors, cv::Mat& cvDstKeyLineDescriptors, 
											 std::vector<std::pair<int, int>>& vKeyLineCoarseMatchPair,
											 std::vector<SiftKeyPoint>& vSrcKeyPoints, std::vector<SiftKeyPoint>& vDstKeyPoints, 
											 std::vector<std::pair<int, int>>& vKeyPointMatchPair)
{
	std::map<int, int> mKeyPointMatchPair;
	std::vector<cv::DMatch> vCoarseMatches;
	
	m_pLineSegmentMatcher->match(cvSrcKeyLineDescriptors, cvDstKeyLineDescriptors, vCoarseMatches);

	for (int i = 0; i < (int)vKeyPointMatchPair.size(); i++)
		mKeyPointMatchPair[vKeyPointMatchPair[i].first] = vKeyPointMatchPair[i].second;

	vKeyLineCoarseMatchPair.clear();
	for (int i = 0; i < (int)vCoarseMatches.size(); i++) {
		int nSrcKeyLineIdx = vCoarseMatches[i].queryIdx;
		int nDstKeyLineIdx = vCoarseMatches[i].trainIdx;

		const cv::line_descriptor::KeyLine& srcKeyLine = vSrcKeyLines[nSrcKeyLineIdx];
		const cv::line_descriptor::KeyLine& dstKeyLine = vDstKeyLines[nDstKeyLineIdx];

		if (srcKeyLine.lineLength < 20 || dstKeyLine.lineLength < 20)
			continue;
		std::vector<int> vSrcKeyPointsIdxInKeyLine = GetBoundedKeyPoints(srcKeyLine, vSrcKeyPoints);
		std::vector<int> vDstKeyPointsIdxInKeyLine = GetBoundedKeyPoints(dstKeyLine, vDstKeyPoints);

		if (vSrcKeyPointsIdxInKeyLine.empty() || vDstKeyPointsIdxInKeyLine.empty())
			continue;

		int nMatchingScore = 0;
		for (int j = 0; j < (int)vSrcKeyPointsIdxInKeyLine.size(); j++) {
			std::map<int, int>::iterator iter = mKeyPointMatchPair.find(vSrcKeyPointsIdxInKeyLine[j]);
			if (iter != mKeyPointMatchPair.end()) {
				std::vector<int>::iterator iter2 = std::find(vDstKeyPointsIdxInKeyLine.begin(), vDstKeyPointsIdxInKeyLine.end(), iter->second);

				if (iter2 != vDstKeyPointsIdxInKeyLine.end())
					nMatchingScore++;
			}
		}

		if (2 * nMatchingScore > std::min((int)vSrcKeyPointsIdxInKeyLine.size(), (int)vSrcKeyPointsIdxInKeyLine.size()))
			vKeyLineCoarseMatchPair.push_back(std::pair<int, int>(nSrcKeyLineIdx, nDstKeyLineIdx));
	}
}

std::vector<int> CLineSegmentWrapper::GetBoundedKeyPoints(const cv::line_descriptor::KeyLine& keyLine, const std::vector<SiftKeyPoint>& vKeyPoints)
{
	std::vector<int> vKeyPointIndices;

	double x1 = keyLine.startPointX;
	double x2 = keyLine.endPointX;
	double x3 = keyLine.pt.x;
	double y1 = keyLine.startPointY;
	double y2 = keyLine.endPointY;
	double y3 = keyLine.pt.y;
	double length1 = keyLine.lineLength;
	double length2 = length1 / 2;

	///< line equation ax+by+c=0;
	double a1 = y1-y2, b1 = x2-x1, c1 = x1*y2 - x2*y1; /// key line equation
	double a2 = b1, b2 = -a1, c2 = -b1*x3 + a1*y3; /// perpendicular line equation of the key line
	double norm1 = std::sqrt(a1*a1 + b1*b1);
	double norm2 = std::sqrt(a2*a2 + b2*b2);

	a1 /= norm1, b1 /= norm1, c1 /= norm1;
	a2 /= norm2, b2 /= norm2, c2 /= norm2;
	for (int i = 0; i < (int)vKeyPoints.size(); i++) {
		double x = vKeyPoints[i].x;
		double y = vKeyPoints[i].y;

		double dist1 = abs(x*a1 + y*b1 + c1);	///< distance to key line
		double dist2 = abs(x*a2 + y*b2 + c2);	///< distance to perpendicular line of the key line

		if (dist1 < length1 && dist2 < length2)
			vKeyPointIndices.push_back(i);
	}

	return vKeyPointIndices;
}
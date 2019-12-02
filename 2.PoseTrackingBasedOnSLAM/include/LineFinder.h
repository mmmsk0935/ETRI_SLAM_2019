#ifndef LINEFINDER_H
#define LINEFINDER_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/line_descriptor/descriptor.hpp>
//#include "C:/opencv\3.1.0/opencv/build/include/opencv2/line_descriptor/descriptor.hpp"

#define PI 3.1415926

namespace ORB_SLAM2
{
//     class LineFinder {
//     private:
//     cv::Mat img; // 원 영상
//     std::vector<cv::Vec4i> lines; // 선을 감지하기 위한 마지막 점을 포함한 벡터
//     double deltaRho;
//     double deltaTheta; // 누산기 해상도 파라미터
//     int minVote; // 선을 고려하기 전에 받아야 하는 최소 투표 개수
//     double minLength; // 선에 대한 최소 길이
//     double maxGap; // 선에 따른 최대 허용 간격

//     public:
//     LineFinder() : deltaRho(1), deltaTheta(PI/180), minVote(10), minLength(0.), maxGap(0.) {}
//     // 기본 누적 해상도는 1각도 1화소 
//     // 간격이 없고 최소 길이도 없음
    
//     // 해당 세터 메소드들

//     // 누적기에 해상도 설정
//     void setAccResolution(double dRho, double dTheta) {
//     deltaRho= dRho;
//     deltaTheta= dTheta;
//     }
    
//     // 투표 최소 개수 설정
//     void setMinVote(int minv) {
//     minVote= minv;
//     }

//     // 선 길이와 간격 설정
//     void setLineLengthAndGap(double length, double gap) {
//     minLength= length;
//     maxGap= gap;
//     }
    
//     // 허프 선 세그먼트 감지를 수행하는 메소드
//     // 확률적 허프 변환 적용
//     std::vector<cv::Vec4i> findLines(cv::Mat& binary) {
//     lines.clear();
//     cv::HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote, minLength, maxGap);
//     return lines;
//     } // cv::Vec4i 벡터를 반환하고, 감지된 각 세그먼트의 시작과 마지막 점 좌표를 포함.
    
//     // 위 메소드에서 감지한 선을 다음 메소드를 사용해서 그림
//     // 영상에서 감지된 선을 그리기
//     void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(255,255,255)) {
    
//     // 선 그리기
//     std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
    
//     while (it2!=lines.end()) {
//     cv::Point pt1((*it2)[0],(*it2)[1]);
//     cv::Point pt2((*it2)[2],(*it2)[3]);
//     cv::line( image, pt1, pt2, color);
//     ++it2;
//         }
//      }
//     };
}

#endif //LINDEFINDER_H
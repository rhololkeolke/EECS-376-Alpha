#ifndef _LIB_DEMO_H
#define _LIB_DEMO_H

#include <opencv2/opencv.hpp>

void blobfind(const cv::Mat& src, cv::Mat& out);

void normalizeColors(const cv::Mat& src, cv::Mat& out);

void findLines(const cv::Mat& src, cv::Mat& out);

#endif

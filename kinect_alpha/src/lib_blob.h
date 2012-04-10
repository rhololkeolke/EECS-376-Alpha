#ifndef _LIB_DEMO_H
#define _LIB_DEMO_H

#include <opencv2/opencv.hpp>

void blobfind(int vars[], const cv::Mat& src, cv::Mat& out, int point);

void normalizeColors(const cv::Mat& src, cv::Mat& out);

void findLines(const cv::Mat& src, cv::Mat& out);

#endif

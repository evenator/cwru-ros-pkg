#ifndef _VISIONLIB_H
#define _VISIONLIB_H

#include <opencv2/core/core.hpp>

bool blobfind(const cv::Mat& src, cv::Mat& out, cv::Point& centroid);

void normalizeColors(const cv::Mat& src, cv::Mat& out);

void findLines(const cv::Mat& src, cv::Mat& out);

#endif

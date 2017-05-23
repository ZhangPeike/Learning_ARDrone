/*
 * GridDetector.h
 *
 *  Created on: Jul 19, 2013
 *      Author: tsou
 */

#ifndef GRIDDETECTOR_H_
#define GRIDDETECTOR_H_
#include "opencv2/opencv.hpp"
#include <vector>

using namespace std;
bool detectCircleGrids(const cv::Mat& img, int m, int n,
		vector<cv::Point2f>& featPts, float thres = 20.0f);

#endif /* GRIDDETECTOR_H_ */

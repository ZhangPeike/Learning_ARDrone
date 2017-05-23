/*
 * GridDetector.cpp
 *
 *  Created on: Jul 19, 2013
 *      Author: tsou
 */

#include "GridDetector.h"

bool detectCircleGrids(const cv::Mat& img, int m, int n, vector<cv::Point2f>& featPts, float thres)
{
	using namespace cv;
	SimpleBlobDetector::Params param;
    param.thresholdStep=thres;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(param);
    //param.thresholdStep = thres;
    //SimpleBlobDetector* pblobDetector = new SimpleBlobDetector(param);
	Size patternsize(n, m);
    //return findCirclesGrid(img, patternsize, featPts, CALIB_CB_SYMMETRIC_GRID,pblobDetector);
    return findCirclesGrid(img,patternsize,featPts,CALIB_CB_ADAPTIVE_THRESH,detector);
}

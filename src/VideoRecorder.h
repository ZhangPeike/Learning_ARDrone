/*
 * VideoRecorder.h
 *
 *  Created on: May 16, 2013
 *      Author: tsou
 */

#ifndef VIDEORECORDER_H_
#define VIDEORECORDER_H_
#include "opencv2/opencv.hpp"
#include "imgproc/SL_Image.h"

#include <utility>
#include <fstream>

using namespace std;

class VideoRecorder {
public:
    bool newframe;
	bool running;
	bool toQuit;
	pthread_t threadId;
	pthread_mutex_t _mutex;
	pthread_cond_t _cond;

	deque<double> timeStamp;
	deque<ImgRGB*> vecImg;

	const char* _tsFilePath;
	const char* _videoFilePath;
	ofstream _tsLog;
	cv::VideoWriter _videoWriter;
	int _frmId;
	bool _recording;
	
	ImgRGB curImg;
public:
    VideoRecorder(const char* tsPath, const char* videoPath);
	virtual ~VideoRecorder();

	static void* threadProc(void* data);

	void addBack(double tm, const cv::Mat& img);
	bool popFront(double& tm, ImgRGB& img);

	void lock();
	void unlock();

    void getImage(cv::Mat& img);
    void getImage(ImgRGB& img);
	void start();
	void loop();
	void end();
};

#endif /* VIDEORECORDER_H_ */

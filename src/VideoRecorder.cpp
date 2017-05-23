/*
 * VideoRecorder.cpp
 *
 *  Created on: May 16, 2013
 *      Author: tsou
 */

#include "VideoRecorder.h"
#include "imgproc/SL_ImageOp.h"
#include <iomanip>
#include <unistd.h>

VideoRecorder::VideoRecorder(const char* tsPath, const char* videoPath) {
    newframe=false;
	running = false;
	toQuit = false;
	threadId = 0;
	pthread_mutex_init(&_mutex, 0);
	pthread_cond_init(&_cond, 0);

	_tsFilePath = tsPath;
	_videoFilePath = videoPath;
	_frmId = 0;
	_recording = false;

	start();
}

VideoRecorder::~VideoRecorder() {
	end();
}

void * VideoRecorder::threadProc(void* data) {
	VideoRecorder* recorder = (VideoRecorder*) data;
	recorder->loop();
	return 0;
}

void VideoRecorder::addBack(double tm, const cv::Mat& mat) {
	pthread_mutex_lock(&_mutex);
	ImgRGB* pImg = new ImgRGB(mat.cols, mat.rows);
	memcpy(pImg->data, mat.data, mat.rows * mat.cols * 3);
    timeStamp.push_back(tm);
    vecImg.push_back(pImg);
	cloneImg(*pImg, curImg);
	pthread_cond_signal(&_cond);
	pthread_mutex_unlock(&_mutex);
    //delete pImg;
    newframe= true;
}
bool VideoRecorder::popFront(double& tm, ImgRGB& img) {
	bool res = false;
	pthread_mutex_lock(&_mutex);
	if (vecImg.empty())
		pthread_cond_wait(&_cond, &_mutex);

	if (!vecImg.empty()) {
		tm = timeStamp.front();
		cloneImg(*vecImg.front(), img);
        delete vecImg.front();////////////////////////////////////////
		timeStamp.pop_front();
		vecImg.pop_front();
		res = true;
	}
	pthread_mutex_unlock(&_mutex);
	return res;
}

void VideoRecorder::getImage(cv::Mat &img){
    pthread_mutex_lock(&_mutex);
    cv::Mat tmpImg(curImg.m, curImg.n, CV_8UC3, curImg.data);
    img = tmpImg.clone();
    pthread_mutex_unlock(&_mutex);
}

void VideoRecorder::getImage(ImgRGB &img){
    pthread_mutex_lock(&_mutex);
    cloneImg(curImg, img);
    pthread_mutex_unlock(&_mutex);
}

void VideoRecorder::lock() {
	pthread_mutex_lock(&_mutex);
}
void VideoRecorder::unlock() {
	pthread_mutex_unlock(&_mutex);
}
void VideoRecorder::start() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&threadId, &attr, threadProc, this);
}

void VideoRecorder::loop() {
	bool bOpen = false;
	double tm;
	ImgRGB img;

    //start from '1'!!
	int frmId = 1;
	while (!toQuit) {
		if (popFront(tm, img)) {
			if (_recording) {
				cv::Mat cvImg(img.m, img.n, CV_8UC3, img.data);
				if (!bOpen) {
					if (!_videoWriter.open(_videoFilePath,
							CV_FOURCC('D', 'I', 'V', 'X'), 30, cvImg.size(),
							true))
						repErr("cannot open '%s' to write!", _videoFilePath);

					_tsLog.open(_tsFilePath);
					if (!_tsLog)
						repErr("cannot open '%s' to write!", _tsFilePath);

					bOpen = true;
				}
				_tsLog << frmId++ << " " << setprecision(12) << tm << endl;
				_videoWriter << cvImg;
			} else
				frmId = 0;
		}
		usleep(100);
	}

	while (popFront(tm, img)) {
		if (_recording) {
			cv::Mat cvImg(img.m, img.n, CV_8UC3, img.data);
			if (!bOpen) {
				if (!_videoWriter.open(_videoFilePath,
						CV_FOURCC('D', 'I', 'V', 'X'), 30, cvImg.size(), true))
					repErr("cannot open '%s' to write!", _videoFilePath);
				bOpen = true;
			}
			_tsLog << frmId++ << " " << setprecision(12) << tm << endl;
			_videoWriter << cvImg;
		}

		_videoWriter.release();
		_tsLog.close();
	}
}
void VideoRecorder::end() {
	if (running) {
		toQuit = true;
		cout << "waiting the IMU recording thread to quit..." << endl;
		pthread_mutex_lock(&_mutex);
		pthread_cond_wait(&_cond, &_mutex);
		pthread_mutex_unlock(&_mutex);
		pthread_join(threadId, 0);
		cout << "IMU thread quit!" << endl;
	}

	for (size_t i = 0; i < vecImg.size(); i++)
		delete vecImg[i];

	vecImg.clear();
}


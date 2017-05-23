/*
 * IMURecorder.h
 *
 *  Created on: May 16, 2013
 *      Author: tsou
 */

#ifndef IMURECORDER_H_
#define IMURECORDER_H_

#include "pthread.h"

#include <deque>
#include <fstream>
#include "IMUData.h"
#include <deque>
#include <vector>

using namespace std;

class IMURecorder {
public:
	IMUData curData;
	bool running;
	bool toQuit;
	pthread_t threadId;
	pthread_mutex_t _mutex;
	pthread_cond_t _cond;

	pthread_mutex_t _opmutex;
	deque<IMUData> imuQ;

	const char* _filePath;
	ofstream log;
	bool _recording;

    int maxCacheNum;
    std::deque<IMUData> cachedImuData;
public:
	IMURecorder(const char* fileName);
	virtual ~IMURecorder();

	static void * threadProc(void* data);
	void addBack(const IMUData& data);
	bool popFront(IMUData& data);
	void getCurData(IMUData& data);
    void getCachedAvg(IMUData& data);
    void getCachedMeas(std::vector<IMUData>& cachedMeas);

	void start();
	void loop();
	void end();
};

#endif /* IMURECORDER_H_ */

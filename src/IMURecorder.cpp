/*
 * IMURecorder.cpp
 *
 *  Created on: May 16, 2013
 *      Author: tsou
 */

#include "IMURecorder.h"
#include "SL_error.h"
#include <iomanip>
#include <iostream>

IMURecorder::IMURecorder(const char* filePath) {
	_filePath = filePath;
	running = false;
	toQuit = false;
	threadId = 0;
	_recording = false;
    maxCacheNum = 100;

	pthread_mutex_init(&_mutex, 0);
	pthread_mutex_init(&_opmutex, 0);
	pthread_cond_init(&_cond, 0);

	start();
}

IMURecorder::~IMURecorder() {
	end();
}

void * IMURecorder::threadProc(void* data) {
	IMURecorder* recorder = (IMURecorder*) data;
	recorder->loop();
	return 0;
}

void IMURecorder::addBack(const IMUData& data) {
	pthread_mutex_lock(&_opmutex);
	curData = data;
	pthread_mutex_unlock(&_opmutex);

	pthread_mutex_lock(&_mutex);
	imuQ.push_back(data);

    cachedImuData.push_back(data);
    if (cachedImuData.size() > maxCacheNum)
        cachedImuData.pop_front();

	pthread_cond_signal(&_cond);
	pthread_mutex_unlock(&_mutex);
}
bool IMURecorder::popFront(IMUData& data) {
	bool res = false;
	pthread_mutex_lock(&_mutex);
	if (imuQ.empty())
		pthread_cond_wait(&_cond, &_mutex);

	if (!imuQ.empty()) {
		data = imuQ.front();
		imuQ.pop_front();
		res = true;
	}

	pthread_mutex_unlock(&_mutex);
	return res;
}
void IMURecorder::getCurData(IMUData& data) {
	pthread_mutex_lock(&_opmutex);
	data = curData;
	pthread_mutex_unlock(&_opmutex);
}

void IMURecorder::getCachedAvg(IMUData &data){
    std::fill_n(data.a, 3, 0);
    std::fill_n(data.g, 3, 0);

    pthread_mutex_lock(&_mutex);
    size_t ndata = cachedImuData.size();
    for (size_t i = 0; i < cachedImuData.size(); i++) {
        data.a[0] += cachedImuData[i].a[0];
        data.a[1] += cachedImuData[i].a[1];
        data.a[2] += cachedImuData[i].a[2];

        data.g[0] += cachedImuData[i].g[0];
        data.g[1] += cachedImuData[i].g[1];
        data.g[2] += cachedImuData[i].g[2];
    }

    data.a[0] /= ndata;
    data.a[1] /= ndata;
    data.a[2] /= ndata;

    data.g[0] /= ndata;
    data.g[1] /= ndata;
    data.g[2] /= ndata;
    pthread_mutex_unlock(&_mutex);
}

void IMURecorder::getCachedMeas(std::vector<IMUData>& cachedMeas){
    pthread_mutex_lock(&_mutex);
    cachedMeas.resize(cachedImuData.size());
    for( size_t i = 0 ; i < cachedImuData.size(); i++){
        cachedMeas[i] = cachedImuData[i];
    }
    pthread_mutex_unlock(&_mutex);
}

void IMURecorder::start() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&threadId, &attr, threadProc, this);
}
void IMURecorder::loop() {
	log.open(_filePath);
	if (!log)
		repErr("cannot open '%s' to write!", _filePath);

	IMUData data;
	while (!toQuit) {
		if (popFront(data)) {
			if (_recording) {
                log << setprecision(12) << data.tm ;
                log << setprecision(12) << " " << data.a[0] << " "
						<< data.a[1] << " " << data.a[2] << " ";

                log << setprecision(12) << data.g[0] << " " << data.g[1] << " "
						<< data.g[2] << " ";

				log << data.ar[0] << " " << data.ar[1] << " " << data.ar[2]
						<< " ";
				log << data.gr[0] << " " << data.gr[1] << " " << data.gr[2]
						<< endl;
			}
		}
	}

	while (popFront(data)) {
		if (_recording) {
            log << setprecision(12) << data.tm << " " << data.a[0] << " "
					<< data.a[1] << " " << data.a[2] << " ";

			log << setprecision(12) << data.g[0] << " " << data.g[1] << " "
                << data.g[2] << " ";
			log << data.ar[0] << " " << data.ar[1] << " " << data.ar[2] << " ";
			log << data.gr[0] << " " << data.gr[1] << " " << data.gr[2] << endl;
		}
	}

	log.close();
}

void IMURecorder::end() {
	if (running) {
		toQuit = true;
        std::cout << "waiting the IMU recording thread to quit..." << std::endl;
		pthread_mutex_lock(&_mutex);
		pthread_cond_wait(&_cond, &_mutex);
		pthread_mutex_unlock(&_mutex);
		pthread_join(threadId, 0);
        std::cout << "IMU thread quit!" << std::endl;
	}
}

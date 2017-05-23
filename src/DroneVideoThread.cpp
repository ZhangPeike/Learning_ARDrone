/*
 * DroneVideoThread.cpp
 *
 *  Created on: May 22, 2013
 *      Author: tsou
 */

#include "DroneVideoThread.h"


DroneVideoThread::DroneVideoThread(VideoRecorder& vid) :
		vidRec(vid) {
	running = false;
	toQuit = false;
	threadId = 0;
	start();
}

DroneVideoThread::~DroneVideoThread() {
	end();
}

void* DroneVideoThread::threadProc(void *data) {
	DroneVideoThread* thread = (DroneVideoThread*) data;
	thread->loop();
	return 0;
}

void DroneVideoThread::start() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&threadId, &attr, threadProc, this);
}

void DroneVideoThread::end() {
	if (running) {
		toQuit = true;
		cout << "waiting the DroneVideoThread to quit..." << endl;
		pthread_join(threadId, 0);
		cout << "DroneVideoThread exists!" << endl;
	}
}

void DroneVideoThread::loop() {
	running = true;
	cout << "starting DroneVideoThread ..." << endl;

	ARVideoCore video;
	if (!video.open()) {
		cerr << "cannot read the video from the ARDRone!" << endl;
		running = false;
	} else {

	}
	cout << " DroneVideoThread exists!" << endl;
}
/*
 * DroneVideoThread.h
 *
 *  Created on: May 22, 2013
 *      Author: tsou
 */

#ifndef DRONEVIDEOTHREAD_H_
#define DRONEVIDEOTHREAD_H_
#include "VideoRecorder.h"

class DroneVideoThread {
public:
	bool running;
	bool toQuit;
	pthread_t threadId;

	VideoRecorder& vidRec;
public:
	DroneVideoThread(VideoRecorder& vid);
	virtual ~DroneVideoThread();
	static void* threadProc(void*);

	void start();
	void end();
	void loop();
};

#endif /* DRONEVIDEOTHREAD_H_ */

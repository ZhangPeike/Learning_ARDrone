/*
 * DroneIMUThread.h
 *
 *  Created on: May 19, 2013
 *      Author: tsou
 */

#ifndef DRONEIMUTHREAD_H_
#define DRONEIMUTHREAD_H_
#include "IMURecorder.h"
#include "VideoRecorder.h"

class DroneIMUThread {
public:
	bool running;
	bool toQuit;
	pthread_t imuThreadId;
	pthread_t vidThreadId;
	IMURecorder& imuRec;
	VideoRecorder& vidRec;

	double tmIMU;
public:
	DroneIMUThread(IMURecorder& imu, VideoRecorder& vid);
	virtual ~DroneIMUThread();

	static void * imuThreadProc(void*);
	static void * vidThreadProc(void*);

	void startIMUThread();
	void imuLoop();
	void endIMUThread();

	void startVidThread();
	void vidLoop();
	void endVidThread();
};

#endif /* DRONEIMUTHREAD_H_ */

/*
 * DroneIMUThread.cpp
 *
 *  Created on: May 19, 2013
 *      Author: tsou
 */

#include "DroneIMUThread.h"
#include "ARDrone.h"
#include "ARVideo.h"

#include <iostream>
#include <iomanip>
using namespace std;
DroneIMUThread::DroneIMUThread(IMURecorder& imu, VideoRecorder& vid) :
		imuRec(imu),vidRec(vid) {
	running = false;
	toQuit = false;
	imuThreadId = 0;
	vidThreadId = 0;
	startVidThread();
	startIMUThread();
}

DroneIMUThread::~DroneIMUThread() {
	endVidThread();
	endIMUThread();
	running = false;
}

void* DroneIMUThread::imuThreadProc(void* data) {
	DroneIMUThread* thread = (DroneIMUThread*) data;
	thread->imuLoop();
	return 0;
}

void* DroneIMUThread::vidThreadProc(void* data) {
	DroneIMUThread* thread = (DroneIMUThread*) data;
	thread->vidLoop();
	return 0;
}

void DroneIMUThread::startIMUThread() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&imuThreadId, &attr, imuThreadProc, this);
}

void DroneIMUThread::startVidThread() {
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&vidThreadId, &attr, vidThreadProc, this);

}

void DroneIMUThread::endIMUThread() {
	if (running) {
		toQuit = true;
		cout << "waiting the DroneIMUThread to quit..." << endl;
		pthread_join(imuThreadId, 0);
		cout << "DroneIMUThread exists!" << endl;
	}
}

void DroneIMUThread::endVidThread() {
	if (running) {
		toQuit = true;
		cout << "waiting the DroneVidThread to quit..." << endl;
		pthread_join(vidThreadId, 0);
		cout << "DroneVidThread exists!" << endl;
	}
}

void DroneIMUThread::imuLoop() {
	running = true;
	cout << "starting DroneIMUThread ..." << endl;

	ARDrone drone;
	if (!drone.open()) {
		cerr << "cannot connect to the ARDrone!" << endl;
		running = false;
		return;
	}
	while (!toQuit) {
		drone.recvNav();

		IMUData data;
		tmIMU = (drone.nav.nav_time.tm_stamp & 0x001FFFFF)
				+ (drone.nav.nav_time.tm_stamp >> 21) * 1000000;

		data.tm = tmIMU / 1000000.0f;

		data.a[0] = drone.nav.nav_phys_measures.phys_accs[0] / 1000.0f;
		data.a[1] = drone.nav.nav_phys_measures.phys_accs[1] / 1000.0f;
		data.a[2] = drone.nav.nav_phys_measures.phys_accs[2] / 1000.0f;

		data.g[0] = drone.nav.nav_phys_measures.phys_gyros[0];
		data.g[1] = drone.nav.nav_phys_measures.phys_gyros[1];
		data.g[2] = drone.nav.nav_phys_measures.phys_gyros[2];

		data.ar[0] = drone.nav.nav_raw_measures.raw_accs[0];
		data.ar[1] = drone.nav.nav_raw_measures.raw_accs[1];
		data.ar[2] = drone.nav.nav_raw_measures.raw_accs[2];

		data.gr[0] = drone.nav.nav_raw_measures.raw_gyros[0];
		data.gr[1] = drone.nav.nav_raw_measures.raw_gyros[1];
		data.gr[2] = drone.nav.nav_raw_measures.raw_gyros[2];

		imuRec.addBack(data);
		cout << "time:" << setprecision(12) << data.tm << " " << data.g[0] << " " << data.g[1] << " " << data.g[2] << " " << endl;
	}

	cout << "DroneIMUThread exists!" << endl;
}
static double getVideoTimeByIMUTime(double pavTime, double imutime) {
	static int num = 0;
	static double s_ds = 0;
	static bool bCal = true;

	if (num < 20) {
		s_ds += pavTime - imutime;
		num++;
	} else {
		if (bCal) {
			s_ds /= num;
			bCal = false;
		} else {
			return pavTime - s_ds;
		}
	}
	return imutime;

}
void DroneIMUThread::vidLoop() {
	running = true;
	cout << "starting DroneVideoThread ..." << endl;

	ARVideo video;
	video.open();
	IplImage *img = cvCreateImage(video.size, IPL_DEPTH_8U, 3);
	while (!toQuit) {
		video.queryFrame(&img);
		video.frames++;
		
		double tmVid = getVideoTimeByIMUTime(video.pave.timestamp, tmIMU);
		vidRec.addBack(tmVid, cv::Mat(img));
	}
	cvReleaseImage(&img);
	cout << " DroneVideoThread exists!" << endl;
}
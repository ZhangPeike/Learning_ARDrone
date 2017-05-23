/*
 * DroneThread.h
 *
 *  Created on: May 15, 2013
 *      Author: tsou
 */

#ifndef NAVDATATHREAD_H_
#define NAVDATATHREAD_H_
#include "pthread.h"
#include "ros/ros.h"
#include "ardrone_autonomy/Navdata.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"

#include "tools/SL_Tictoc.h"
#include "imgproc/SL_Image.h"

#include "IMURecorder.h"
#include "VideoRecorder.h"

#include <deque>

using namespace std;
class ROSThread {
public:
	bool running;
	bool toQuit;
	bool recording;
	bool showVideo;
	pthread_t threadId;

	ros::Subscriber imusub;
	ros::Subscriber navsub;
	ros::Subscriber vidsub;
	ros::NodeHandle node;

	IMURecorder& imuRec;
	VideoRecorder& vidRec;

	double tmIMU;
    ////////////////////////////////navdata
    ardrone_autonomy::Navdata navdata;
    /*
    double battery;//Percent: The remaining charge of the drone's battery (%)
    int state;//: The Drone's current state: * 0: Unknown * 1: Inited * 2: Landed * 3,7: Flying * 4: Hovering * 5: Test (?) * 6: Taking off * 8: Landing * 9: Looping (?)
    double rotX,rotY,rotZ;//: Left/right tilt in degrees (rotation about the X axis)
    //rotY: Forward/backward tilt in degrees (rotation about the Y axis)
    //rotZ: Orientation in degrees (rotation about the Z axis)
    double magX, magY, magZ;//: Magnetometer readings (AR-Drone 2.0 Only) (TBA: Convention)
    double pressure;//: Pressure sensed by Drone's barometer (AR-Drone 2.0 Only) (TBA: Unit)
    double temp;// : Temperature sensed by Drone's sensor (AR-Drone 2.0 Only) (TBA: Unit)
    double wind_speed;//: Estimated wind speed (AR-Drone 2.0 Only) (TBA: Unit)
    double wind_angle;//: Estimated wind angle (AR-Drone 2.0 Only) (TBA: Unit)
    double wind_comp_angle;//: Estimated wind angle compensation (AR-Drone 2.0 Only) (TBA: Unit)
    double altd;//: Estimated altitude (mm)
    //double motor1, motor2, motor3, motor4; motor1..4: Motor PWM values
    double vx, vy, vz;//: Linear velocity (mm/s) [TBA: Convention]
    double ax, ay, az;//: Linear acceleration (g) [TBA: Convention]
    //tm: Timestamp of the data returned by the Drone returned as number of micro-seconds passed since Drone's boot-up.
*/
    ///////////////////////////////////

	void (*cbROSThread)(void);
public:
	ROSThread(IMURecorder& imu, VideoRecorder& vidRec);
	virtual ~ROSThread();
	static void * threadProc(void*);

	void imuCb(const sensor_msgs::Imu::ConstPtr imuPtr);
	void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navPtr);
    void vidCb(const sensor_msgs::ImageConstPtr img);

	void setCallback(void (*CallbackROSThread)(void)) {
		cbROSThread = CallbackROSThread;
	}

	void start();
	void end();
	void loop();

	void enableRecord() {
		recording = true;
	}
};

#endif /* NAVDATATHREAD_H_ */

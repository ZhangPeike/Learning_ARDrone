/*
 * IMUVideoSync.cpp
 *
 *  Created on: Jun 15, 2013
 *      Author: tsou
 */

#include "IMUVideoSync.h"

#include "ros/ros.h"
#include "math/SL_Matrix.h"

#include "std_msgs/String.h"

#include "imgproc/SL_Image.h"
#include "imgproc/SL_ImageIO.h"
#include "tools/GUI_ImageViewer.h"
#include "tools/SL_Print.h"
#include "tools/SL_DrawCorners.h"
#include "ROSThread.h"
#include "VideoRecorder.h"
#include "IMURecorder.h"

#include "DroneIMUThread.h"

static char myStr[120];
static VideoRecorder* pvideo = 0;
static IMURecorder* pimu = 0;
static ROSThread* pthread = 0;

//*********************************************
//* RenderToDisplay()                       *
//*********************************************

static void RenderToDisplay() {
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, -25.0f);
	glScalef(0.1, 0.1, 0.1);
	glColor3f(1.0, 1.0, 1.0);

	if (pimu) {
		IMUData data;
		pimu->getCurData(data);
		sprintf(myStr, "%.06f", data.tm);
	}

	int nc = (int) strlen(myStr);

	int FONT_WIDTH = 80;
	int FONT_HEIGHT = 100;

	int xoff = (int) (nc * FONT_WIDTH * 0.5);
	for (int i = 0; i < nc; i++) {
		glPushMatrix();
		glTranslatef((i * FONT_WIDTH) - xoff, -FONT_HEIGHT / 2, 0.0);
		glColor3f(0.0, 0.0, 1.0);
		glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, myStr[i]);
		glPopMatrix();
	}
	glPopMatrix();

	if (pimu && pimu->_recording) {
		glTranslated(-2.f, 0.7f, -1.f);
		glScalef(0.1f, 0.1f, 0.1f);
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_QUADS);									// Draw A Quad
		glVertex3f(-1.0f, 1.0f, 0.0f);					// Top Left
		glVertex3f(1.0f, 1.0f, 0.0f);					// Top Right
		glVertex3f(1.0f, -1.0f, 0.0f);					// Bottom Right
		glVertex3f(-1.0f, -1.0f, 0.0f);					// Bottom Left
		glEnd();
	}
}
//*********************************************
//* glutDisplayFunc(myDisplayFunction);       *
//*********************************************

static void myDisplayFunction(void) {
	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glLoadIdentity();
	RenderToDisplay();
	glutSwapBuffers();
}
static void timeTick(int) {
	myDisplayFunction();
	glutTimerFunc(5, timeTick, 0);
}

//*********************************************
//* glutReshapeFunc(reshape);               *
//*********************************************

static void reshape(int w, int h) {
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);  // Select The Projection Matrix
	glLoadIdentity();             // Reset The Projection Matrix
	if (h == 0)  // Calculate The Aspect Ratio Of The Window
		gluPerspective(90, (float) w, 1.0, 5000.0);
	else
		gluPerspective(90, (float) w / (float) h, 1.0, 5000.0);
	glMatrixMode(GL_MODELVIEW);  // Select The Model View Matrix
	glLoadIdentity();    // Reset The Model View Matrix
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 27:        // When Escape Is Pressed...
		exit(0);   // Exit The Program
		break;        // Ready For Next Case
	case 'q':
		exit(0);
		break;
	case 'r':
		pimu->_recording = true;
		pvideo->_recording = true;
		break;
	case 's':
		break;
	default:        // Now Wrap It Up
		break;
	}
}

int IMUVideoSync_main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	VideoRecorder video("/home/tsou/Record/ts.txt",
			"/home/tsou/Record/video.avi");
	IMURecorder imu("/home/tsou/Record/imu.txt");
	ROSThread thread(imu, video);
	thread.showVideo = true;

	pvideo = &video;
	pimu = &imu;
	pthread = &thread;

	sprintf(myStr, "");
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1600, 600);
	glutCreateWindow("For IMU-Video synchronization");
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glLineWidth(12);

	glutDisplayFunc(myDisplayFunction);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(5, timeTick, 0);
	glutMainLoop();
	return 0;
}
#include "PIDController.h"
//#include "ModelParam.h"
//#include "ControlData.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Triangulate.h"
//#include "app/SL_MoSLAM.h"

#include "SL_error.h"

#include <fstream>
using namespace std;
void PIDController::loadParam(const char* filePath){
	std::ifstream file(filePath);
	file >> kP >> kI >> kD >> kN;
	cout <<" kP:" << kP << " kI:" << kI << " kD:" << kD << " kN:" << kN << endl;
	file.close();
}
void PIDController::setParam(double _kP, double _kI, double _kD, double _kN){
	kP = _kP;
	kI = _kI;
	kD = _kD;
	kN = _kN;
}
void PIDController::reset(){
	old_err = 0;
	cur_err = 0;
	old_D = 0;
	cur_I = 0;
	old_D = 0;
	_bFirstFrame = true;
}
double PIDController::getOutput(double curerr, double dt){
	old_err = cur_err;
	cur_err = curerr;
    ///////////////////////////////
    for (int i =0; i<10; ++i) errs[i+1]=errs[i];
    errs[0]=curerr;
    ///////////////////////////

	double s = 0;
	if( !_bFirstFrame){
		assert(dt > 0);
		cur_I += cur_err*dt;
		_P = cur_err;
		_I = cur_I;
		double Derr = (cur_err - old_err)/dt;
		_D = (kN*dt*Derr + old_D)/(kN*dt + 1);
		old_D = _D;
	}else{
		_P = cur_err;
		_I = 0;
		_D = 0;
		_bFirstFrame = false;
	}
    //////////////////////////////
    double aveerr = (errs[0]+errs[1])/2;
    double pasterr =0;
    for (int i=1; i<=3; ++i) pasterr+=errs[i];
    pasterr/=3;
    _D = aveerr - pasterr;
    cout << "        _P" << _P << " _D" << _D; //<< " _I" << _I
	return kP*_P + kI*_I + kD*_D;
}
/*
static std::ofstream _log;

//#include "app/MyApp.h"
void PositionController::loadParameters(){
	_pidX.loadParam("D:/for_debug/PID_x.txt");
	_pidY.loadParam("D:/for_debug/PID_y.txt");
	_pidZ.loadParam("D:/for_debug/PID_z.txt");

	char fileName[256];
	sprintf(fileName, "D:/for_debug/roll_test_%s.txt", MyApp::timeStr);
	_log.open(fileName);
}
void PositionController::getXYZErrors(double scale, const double* R, const double* t, double err[3]){
	double M[3];
	getCameraCenter(R, t, M);
	err[0] = _refPt.x - M[0]*scale;
	err[1] = _refPt.y - M[1]*scale;
	err[2] = _refPt.z - M[2]*scale;
}
void PositionController::correct(double dt, double scale, const double* R, const double* t,double act_roll, double act_pitch, double act_yaw){
	double err[3],err_cam[3], err_uav[3];

	double curM[3];
	getCameraCenter(R,t,curM);

	getXYZErrors(scale, R, t, err);

    ///* convert to camera coordinates
	mat33ProdVec(R, err, err_cam);

    ///* convert to drone coordinates
	mat33ProdVec(ModelParam::s_Rdrone, err_cam, err_uav);

	double errX = err_uav[0]/1000;
	double errY = err_uav[1]/1000;
	double errZ = err_uav[2]/1000;

	double req_roll = _pidX.getOutput(errX, dt);
	double req_pitch = -_pidY.getOutput(errY, dt);
	double req_speed = _pidZ.getOutput(errZ, dt);

	ControlData::s_ex = errX;
	ControlData::s_ey = errY;
	ControlData::s_ez = errZ;

	ControlData::s_rphi = req_roll;
	ControlData::s_rtheta = req_pitch;
	ControlData::s_rspeed = req_speed;

    ///*_log.print("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", errX, errY, errZ, req_roll, req_pitch, req_speed,
        //act_roll, act_pitch, act_yaw, ControlData::s_vx, ControlData::s_vy, ControlData::s_altitude, _pidX._P, _pidX._I, _pidX._D, dt);

	_log << errX <<" " << errY << " " << errZ << " ";
	_log << err[0] << " " << err[1] << " " << err[2] << " ";
	_log << curM[0]*scale <<" " << curM[1]*scale << " " << curM[2]*scale << " ";
	_log << _refPt.x << " " << _refPt.y << " " << _refPt.z << " ";
	//_log << req_roll << " " << req_pitch << " " << req_speed << " ";
	//_log << act_roll << " " << act_pitch << " " << act_yaw << " ";
	_log << ControlData::s_vx << " " << ControlData::s_vy << " " << ControlData::s_altitude << " " << endl;
	//_log << _pidX._P << " "<< _pidX._I << " " << _pidX.cur_I << " " << _pidX.kI << " "<< _pidX._D << " " << dt << endl;

	sendMove(req_roll, req_pitch, req_speed);
}
//#include "app/MyApp.h"
void sendMove(double req_roll, double req_pitch, double req_speed){
	float rphi = (float) req_roll;
	float rtheta = (float) req_pitch;
	float rspeed = (float) req_speed;
	float ryaw = 0;
	//send control command 
	MyApp::ardrone.move(rphi, rtheta, 0, 0);
}
void PositionController::update(){
	if(  _moSLAM->state != SLAM_STATE_NORMAL)
		return ;

	RefPoint pt;
	if( getRefPointToRun(pt)){
		moveTo(pt);
	}

	if( !_refPt.isRunning())
		return;

	const CamPoseItem* cam = _moSLAM->camPose.current();
	const double* R = cam->R;
	const double* t = cam->t;

	double scale = ControlData::s_scale;
	double roll = _moSLAM->m_navdata.back()._roll;
	double pitch = _moSLAM->m_navdata.back()._pitch;
	double yaw = _moSLAM->m_navdata.back()._yaw;

	uint32_t curts = _moSLAM->camPose.current()->ts; 
	double dt = _oldts == 0 ? 0:double(curts - _oldts)/1000;
	_oldts = curts;
	correct(0.04, scale, R, t, roll, pitch, yaw);
}
*/
//#include "tools/SL_WriteRead.h"
//int main(int argc, char* argv){
//	Mat_d res;
//	readMat(res,"D:/for_debug/roll_test_13-02-06=19-46.txt");
//
//	Mat_d I(res.m, 3);
//	PIDController pid;
//	pid.loadParam("D:/for_debug/PID_x.txt");
//	for( int i = 0; i < res.m; i++){
//		double err = pid.getOutput(res(i,0), 0.04);
//		I(i,0) = pid._P;
//		I(i,1) = pid._I;
//		I(i,2) = pid._D;
//	}
//	writeMat(I, "D:/for_debug/I.txt");
//
//	return 0;
//}

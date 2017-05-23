#ifndef _POSITION_CONTROLLER_H
#define _POSITION_CONTROLLER_H
//#include "RefPoint.h"
#include <iostream>
#include <vector>
#include <stdint.h>

class MoSLAM;
class PIDController{
public:
	/* coefficients for P, I, D*/
	double kP, kI, kD;
	/* coefficient for filtering the derivative values */
	double kN;
	double _P, _I, _D;
	bool _bFirstFrame;
//protected:
	double old_err, cur_err;
    //////////////////////////////
    double errs[20];
	/* error integration*/
	double cur_I;
	/* error derivative*/
	double old_D;
public:
	PIDController():kP(0.0),kI(0.0),kD(0.0){reset();}
	void loadParam(const char* filePath);
	void setParam(double _kP, double _kI, double _kD, double _kN);

	void reset();
	double getOutput(double curerr, double dt);
};
/*
class PositionController{
protected:
	uint32_t _oldts;
    ///* reference point
	RefPoint _refPt;
    ///* error to the reference point
	const MoSLAM* _moSLAM;
	PIDController _pidX, _pidY, _pidZ;
public:
	PositionController():_oldts(0),_moSLAM(0){}
	void loadParameters();
	void setMoSLAM(const MoSLAM* moSLAM){
		_moSLAM = moSLAM;
	}
	void reset(){
		_oldts = -1;
		_pidX.reset();
		_pidY.reset();
		_pidZ.reset();
		_refPt.setFinished();
		//std::cout <<" reset is called!" << std::endl;
	}
	void moveTo(RefPoint refPt){
		reset();
		_refPt = refPt;
		_refPt.setRunning();
	}
	void getXYZErrors(double scale, const double* R, const double* t, double err[3]);
	void correct(double dt, double scale, const double* R, const double* t, double roll, double pitch, double yaw);
	void update();
};
void sendMove(double req_phi, double req_theta, double req_speed);
*/
#endif

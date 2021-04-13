/* KFApp c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2018-09-01
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "PSINS.h"

class CKFApp:public CSINSKF
{
public:
	int reinit;
	CVect3 lvGPS;
	CAVPInterp avpi;

	CKFApp(double ts);
	virtual void Init(const CSINS &sins0, int grade=-1);
	void ReInit(void);
	virtual void SetFt(int nnq=15);
	virtual void SetMeas(void) {};
	virtual void SetMeas(const CVect3 &pgps, const CVect3 &vgps, double dt=0.0);
	int KFUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);
};

typedef struct {
	double wn, sec;
	CVect3 wm, vm, gpsp, gpsv;
	double dt, gpsvalid;
} IMUGNSS;


#endif


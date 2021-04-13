#include "pch.h"
#include "KFApp.h"

/***************************  class CKFApp  *********************************/
CKFApp::CKFApp(double ts):CSINSKF(19, 6)
{
//state: 0-2 phi; 3-5 dvn; 6-8 dpos; 9-11 eb; 12-14 db; 15-17 lever; 18 dt; 19 dKGzz
	lvGPS = CVect3(-0.8, -2.6, 1.0);
	reinit = 0;
}

void CKFApp::Init(const CSINS &sins0, int grade)
{
	CSINSKF::Init(sins0);
	avpi.Init(sins);
	Pmax.Set2(50.0*glv.deg,50.0*glv.deg,100.0*glv.deg,    500.0,500.0,500.0,    1.0e6/glv.Re,1.0e6/glv.Re,1.0e6, 
		5000.0*glv.dph,5000.0*glv.dph,5000.0*glv.dph,    10.0*glv.mg,10.0*glv.mg,10.0*glv.mg, 10.0,10.0,10.0, 0.5, 10000*PPM);
	Pmin.Set2(0.1*glv.min,0.1*glv.min,.30*glv.min,    0.001,0.001,0.01,    .10/glv.Re,.10/glv.Re,0.1, 
		1.10*glv.dph,1.10*glv.dph,1.10*glv.dph,    10.0*glv.ug,10.0*glv.ug,200.0*glv.ug, 0.01,0.01,0.01, 0.01, 0*PPM);
	Pk.SetDiag2(10.0*glv.deg,10.0*glv.deg,3.0*glv.deg,    1.0,1.0,1.0,     10.0/glv.Re,10.0/glv.Re,10.0, 
		100.0*glv.dph,100.0*glv.dph,100.0*glv.dph,    3.0*glv.mg,3.0*glv.mg,10.0*glv.mg, 1.0,1.0,1.0,  0.1, 0*PPM);
	Qt.Set2(1.20*glv.dpsh,1.20*glv.dpsh,1.50*glv.dpsh,    1.0*glv.ugpsHz,1.0*glv.ugpsHz,10.0*glv.ugpsHz,    0.0,0.0,0.0,
		0.0*glv.dphpsh,0.0*glv.dphpsh,0.0*glv.dphpsh,    0.0*glv.ugpsh,0.0*glv.ugpsh,0.0*glv.ugpsh, 0.0,0.0,0.0, 0.0, 0.0);
	Xmax.Set(INF,INF,INF, INF,INF,INF, INF,INF,INF,  1.0*glv.dps,1.0*glv.dps,1.0*glv.dps,  50.0*glv.mg,50.0*glv.mg,50.0*glv.mg, 10.0,10.0,10.0, 0.5, 10000*PPM);
	Rt.Set2(0.2,0.2,0.6,   10.0/glv.Re,10.0/glv.Re,30.0);
//	Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.95;
	FBTau.Set(1.0,1.0,1.0,     1.0,1.0,1.0,     1.0,1.0,1.0,    1.0,1.0,1.0,    1.0,1.0,1.0,   INF,INF,INF, INF, INF);
}

void CKFApp::ReInit(void)
{
	Pmin.Set2(0.1*glv.min,0.1*glv.min,.30*glv.min,    0.001,0.001,0.01,    .10/glv.Re,.10/glv.Re,0.1, 
		.10*glv.dph,.10*glv.dph,1.10*glv.dph,    10.0*glv.ug,10.0*glv.ug,20.0*glv.ug, 0.01,0.01,0.01, 0.01, 0*PPM);
	Qt.Set2(0.20*glv.dpsh,0.20*glv.dpsh,0.50*glv.dpsh,    1.0*glv.ugpsHz,1.0*glv.ugpsHz,10.0*glv.ugpsHz,    0.0,0.0,0.0,
		0.0*glv.dphpsh,0.0*glv.dphpsh,0.0*glv.dphpsh,    0.0*glv.ugpsh,0.0*glv.ugpsh,0.0*glv.ugpsh, 0.0,0.0,0.0, 0.0, 0.0);
	Rb = 0.95;
}

void CKFApp::SetFt(int nnq)
{
	CSINSKF::SetFt(19);
	if(nnq==20) {
		CMat3 Cwz=-sins.wib.k*sins.Cnb;
		Ft(2,19) = Cwz.e22;  // dKGzz
	}
}

void CKFApp::SetMeas(const CVect3 &pgps, const CVect3 &vgps, double dt)
{
	if(!avpi.Interp(-dt)) return;
	if(!IsZero(pgps))
	{
		*(CVect3*)&Zk.dd[3] = avpi.pos - pgps;
		SetMeasFlag(000070);
	}
	if(!IsZero(vgps))
	{
		*(CVect3*)&Zk.dd[0] = avpi.vn - vgps;
		SetMeasFlag(000003);
	}
}

int CKFApp::KFUpdate(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	int res=Update(pwm, pvm, nSamples, ts);
	avpi.Push(sins);

	if(reinit==0 && Pk(2,2)<pow2(1*DEG)) { ReInit(); reinit=1; }

	return res;
}


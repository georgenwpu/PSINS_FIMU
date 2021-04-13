#include "pch.h"
#include "NaviThread.h"
#include "InputInitInfoDlg.h"

IMPLEMENT_DYNCREATE(NaviThread, CWinThread)

CKFApp kf(TS);

NaviThread::NaviThread()
{
}

NaviThread::~NaviThread()
{
}

BEGIN_MESSAGE_MAP(NaviThread, CWinThread)
	ON_THREAD_MESSAGE(NT_ALIGNINIT, AlignInit)
	ON_THREAD_MESSAGE(NT_ALIGNUPDATE, AlignUpdate)
	ON_THREAD_MESSAGE(NT_NaviINIT, NaviInit)
	ON_THREAD_MESSAGE(NT_NAVIUPDATE, NaviUpdate)
	ON_THREAD_MESSAGE(NT_KFINIT, KFInit)
	ON_THREAD_MESSAGE(NT_KFTUPDATE, KFTimeUdt)
	ON_THREAD_MESSAGE(NT_KFMUPDATE, KFMeasUdt)
END_MESSAGE_MAP()

BOOL NaviThread::InitInstance()
{
	return TRUE;
}

int NaviThread::ExitInstance()
{
	return CWinThread::ExitInstance();
}

void NaviThread::AlignInit(WPARAM wParam, LPARAM lParam)
{
	double* avp0 = (double*)wParam;
	for (size_t i = 0; i < 9; i++)
	{
		NTavp[i] = avp0[i];
	}
	/*NTavp[6] *= DEG;
	NTavp[7] *= DEG;*/
	v0 = avp0 + 3;
	p0 = avp0 + 6;
	p0.i *= DEG;
	p0.j *= DEG;
	aligni0.Init(v0, p0, 0);
	AlignInitFinish = TRUE;
}

void NaviThread::AlignUpdate(WPARAM wParam, LPARAM lParam)
{
	aligni0.Update(wmi, vmi, Samples, TS);
	NTavp[0] = q2att(aligni0.qnb).i / DEG;
	NTavp[1] = q2att(aligni0.qnb).j / DEG;
	NTavp[2] = q2att(aligni0.qnb).k / DEG;
	SendMessage((AfxGetMainWnd())->m_hWnd, NT_AVPUDT, (WPARAM)NTavp, NULL);
}

void NaviThread::NaviInit(WPARAM wParam, LPARAM lParam)
{
	kf.Init(CSINS(aligni0.qnb, v0, p0, (*(int*)wParam) * TS), 1);
	kf.avpi.Init(kf.sins);  kf.avpi.ts = 1 * TS;
	NaviInitFinished = TRUE;
}

void NaviThread::NaviUpdate(WPARAM wParam, LPARAM lParam)
{
	kf.sins.Update(wmi, vmi, Samples, TS);
	NTavp[1] = kf.sins.att.j / DEG;
	NTavp[0] = kf.sins.att.i / DEG;
	NTavp[2] = kf.sins.att.k / DEG;
	NTavp[3] = kf.sins.vn.i;
	NTavp[4] = kf.sins.vn.j;
	NTavp[5] = kf.sins.vn.k;
	NTavp[6] = kf.sins.pos.i / DEG;			// deg
	NTavp[7] = kf.sins.pos.j / DEG;		// deg
	NTavp[8] = kf.sins.pos.k;
	SendMessage((AfxGetMainWnd())->m_hWnd, NT_AVPUDT, (WPARAM)NTavp, NULL);
}

void NaviThread::KFInit(WPARAM wParam, LPARAM lParam)
{
	kf.Init(kf.sins, 1);
}

void NaviThread::KFTimeUdt(WPARAM wParam, LPARAM lParam)
{
	kf.Update(wmi, vmi, Samples, TS);
	NTavp[1] = kf.sins.att.j / DEG;
	NTavp[0] = kf.sins.att.i / DEG;
	NTavp[2] = kf.sins.att.k / DEG;
	NTavp[3] = kf.sins.vn.i;
	NTavp[4] = kf.sins.vn.j;
	NTavp[5] = kf.sins.vn.k;
	NTavp[6] = kf.sins.pos.i / DEG;			// deg
	NTavp[7] = kf.sins.pos.j / DEG;		// deg
	NTavp[8] = kf.sins.pos.k;
	SendMessage((AfxGetMainWnd())->m_hWnd, NT_AVPUDT, (WPARAM)NTavp, NULL);
}

void NaviThread::KFMeasUdt(WPARAM wParam, LPARAM lParam)
{
	if (lParam == 0)
	{
		kf.SetMeas(p0, CVect3(0.0), 0);
	}
	else
	{
		double* gpsvp = (double*)wParam;
		kf.SetMeas(CVect3(gpsvp + 3), CVect3(gpsvp), 0);
	}
}

void NaviThread::reInit()
{
	AlignFished = FALSE;
	AlignInitFinish = FALSE;
	NaviInitFinished = FALSE;
}



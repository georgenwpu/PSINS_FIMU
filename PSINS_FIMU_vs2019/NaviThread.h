#pragma once
#include "PSINSCore/KFApp.h"
#include "PSINSCore/PSINS.h"

#define TS (1.0/400.0)
#define Samples 2
#define NT_ALIGNINIT		WM_USER+1101
#define NT_ALIGNUPDATE	WM_USER+1102
#define NT_NaviINIT		WM_USER+1111
#define NT_NAVIUPDATE	WM_USER+1112			// 与Kalman滤波时间更新重合
#define NT_KFINIT		WM_USER+1113
#define NT_KFTUPDATE		WM_USER+1114
#define NT_KFMUPDATE		WM_USER+1115
#define NT_AVPUDT		WM_USER+1201

typedef struct {
	BOOL ComputerONOFF;
	int alignType;
	int alignTimeC, alignTImeF;
}NaviParaS;

typedef struct {
	CVect3* wm;
	CVect3* vm;
	int nSamples;
	double ts;
	CVect3 vel;
} AlignUdtPkg;

class NaviThread :
    public CWinThread
{
	DECLARE_DYNCREATE(NaviThread)
public:
	NaviThread();
	virtual ~NaviThread();
public:
	virtual BOOL InitInstance();
	virtual int  ExitInstance();
protected:
	afx_msg void AlignInit(WPARAM wParam, LPARAM lParam);
	afx_msg void AlignUpdate(WPARAM wParam, LPARAM lParam);
	afx_msg void NaviInit(WPARAM wParam, LPARAM lParam);
	afx_msg void NaviUpdate(WPARAM wParam, LPARAM lParam);
	afx_msg void KFInit(WPARAM wParam, LPARAM lParam);
	afx_msg void KFTimeUdt(WPARAM wParam, LPARAM lParam);
	afx_msg void KFMeasUdt(WPARAM wParam, LPARAM lParam);
	DECLARE_MESSAGE_MAP()

public:
	CSINS sins;
	HWND NTMainDlg;
	double NTavp[9] = { 0.0 };
	NaviParaS NPS;
	CVect3 wmi[Samples], vmi[Samples];		// samples used to compute
	CVect3 v0, p0;

	BOOL AlignFished = FALSE, AlignInitFinish = FALSE;
	BOOL NaviInitFinished = FALSE;
	double tk;			// INS system time
	CAligni0 aligni0;	// Alignment in inertial frame

	void reInit();
};


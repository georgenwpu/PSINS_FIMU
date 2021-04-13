
// PSINS_FIMUDlg.h: 头文件
//
#include "NavChart.h"
#include "SerialPort.h"
#include "PSINSCore/KFApp.h"
#include "DigiStatic_src/Digistatic.h"
#include "NaviThread.h"
#include "NaviDataStore.h"
#include "InputInitInfoDlg.h"
#pragma once

#define EARTH_RE (6300000.0)		// 地球半径

typedef struct
{
	int counter;
	char CS;
	int Gx;
	int Gy;
	int Gz;
	int Ax;
	int Ay;
	int Az;
	int Wx;
	int Wy;
	int Wz;
	int Fx;
	int Fy;
	int Fz;
	double wx;
	double wy;
	double wz;
	double fx;
	double fy;
	double fz;
	double wxts;
	double wyts;
	double wzts;
	double fxts;
	double fyts;
	double fzts;
} IMUData;

typedef struct
{
	float time;		// UTC时间
	double Lat;		// 纬度
	char NS;		// 南北半球
	double Lng;		// 经度
	char WE;		// 东西半球
	short state;	// GPS解类型
	short SatNum;	// 卫星数
	float HDOP;		// HDOP
	float Alt;		// 海拔高度
	float Herr;		// 大地水准面高度异常值
	float xx;		// 差分数据期限
	short BaseNum;	// 参考站标号
	short chk;		// 校验
} GPGGAPKG;

typedef struct
{
	char header[256];
	char SOL_STATUS[64];
	UCHAR SS;
	char VEL_TYPE[64];
	UCHAR VT;
	float latency;
	float age;
	double vel_h;
	double heading;
	double vel_v;
	float xx;
	char CRC[16];
}BESTVELPKG;



// CPSINSFIMUDlg 对话框
class CPSINSFIMUDlg : public CDialogEx
{
// 构造
public:
	CPSINSFIMUDlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PSINS_FIMU_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg LRESULT OnI3DlgMsg(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnAVPUdt(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnSerialMsg(WPARAM wParam, LPARAM lParam);
	DECLARE_MESSAGE_MAP()
public:

	// 参数设置相关变量与事件（IMU，GNSS，对准）
	afx_msg void OnCbnSelchangeComboImucom();
	afx_msg void OnCbnSelchangeComboImubdr();
	afx_msg void OnCbnSelchangeComboGpscom();
	afx_msg void OnCbnSelchangeComboGpsbdr();
	afx_msg void OnCbnSelchangeComboAlignmethod();
	afx_msg void OnCbnSelchangeComboAligntime();
	afx_msg void OnCbnSelchangeComboFiletype();
	afx_msg void OnCbnSelchangeComboInitsource();
	afx_msg void OnCbnSelchangeComboNavmethod();
	CComboBox combo_imucom;
	CComboBox combo_imubdr;
	CComboBox combo_gpscom;
	CComboBox combo_gpsbdr;
	CComboBox combo_align;
	CComboBox combo_aligntime;
	CComboBox combo_filetype;
	CComboBox combo_initsource, combo_navimethod;
	CDigiStatic NavAx, NavAy, NavAz;
	CDigiStatic NavVx, NavVy, NavVz;
	CDigiStatic NavPx, NavPy, NavPz;
	CDigiStatic IMUWx, IMUWy, IMUWz;
	CDigiStatic IMUFx, IMUFy, IMUFz;
	CButton Btn_open;
	CButton Btn_start;
	CButton Btn_exit;
	void imuDataUpdate();
	void navDataUpdate();
	CFont titleFont;
	CStatic bigTitle;

	// 画图控件
	NavChart ChartAttx, ChartAtty, ChartAttz;
	NavChart ChartVelx, ChartVely, ChartVelz;
	NavChart ChartPos, ChartAlt;
	NavChart ChartGyro, ChartAcce;
	void imuChartUpdate();
	void navChartUpdate();

	afx_msg void OnBnClickedButtonOpen();
	afx_msg void OnBnClickedButtonStart();
	afx_msg void OnBnClickedButtonExit();

	// 串口相关
	SerialPort serialport;
	IMUData imudata = { 0 };			// 当前imu输出
	GPGGAPKG gpgga;
	BESTVELPKG bestvel;
	BOOL GPGGAOK, BESTVELOK;
	HANDLE TP_CommThread, GPS_CommThread;
	unsigned char pkgbuffer[64];	// 数据包存储
	char pkgbufferG0[512];	// 数据包存储
	char pkgbufferG1[512];	// 数据包存储
	UINT msg_errlost = 101;
	UINT msg_errchkfaile = 102;

	// 导航解算相关
	char alignState;			// AlignMethod：0->static base; 1->moving  base; 2->GNSS assistant
	char insState;			// NavMethod：0->Inertial Navigation; 1->INS/GNSS Integrated Navigation
	char initSource;			// Initalization information source: 0-> input;  1-> GNSS init
	char fileState;			// FileType: 0->text type; 1->binary type;
	UINT alignTime;			// Align TIme
	BOOL isNaviRunning;
	void posDataUpdate();
	double dlgAVPt[9] = { 0.0 };	// avp vector used to refresh dialog
	double dlgAVP0[9] = { 0.0 };
	NaviThread* navithread;
	WPARAM threadpara1;
	LPARAM threadpara2;
	InputInitInfoDlg I3Dlg;
	
	// 文件
	NaviDataStore navidata;
	BOOL isRunningMsg;
	BOOL IMURcvChk();

	/*------------线程相关--------------------*/
	DWORD exitCode;		// exitCode读取成功标志
	static LPVOID threadPara;
	static BOOL isRunningIMU;
	static BOOL isRunningGPS;
	static DWORD WINAPI SerialThread(LPVOID lpara);
	static DWORD WINAPI SerialThreadGPS(LPVOID lpara);

	/*------------时间相关--------------------*/
	SYSTEMTIME st = { 0 };		// 系统时间
	CString time_tmp;
	int secCounter = 0;
	int rcvCounter;
	unsigned int SYSCNT = 0;
	CDigiStatic Timer_day;
	CDigiStatic Timer_hour;
	CDigiStatic Timer_minute;
	CDigiStatic Timer_month;
	CDigiStatic Timer_second;
	CDigiStatic Timer_year;
	CDigiStatic Counter;
	CButton BtnPause;
	CButton BtnReset;
	BOOL counterPause = FALSE;
	void SetLocalTime();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg void OnClickedBtnPause();
	afx_msg void OnClickedBtnSetzero();
};

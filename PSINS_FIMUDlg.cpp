
// PSINS_FIMUDlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "PSINS_FIMU.h"
#include "PSINS_FIMUDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

LPVOID CPSINSFIMUDlg::threadPara = NULL;
int CPSINSFIMUDlg::isRunningIMU = 0;
int CPSINSFIMUDlg::isRunningGPS = 0;
NaviParaS nps;

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CPSINSFIMUDlg 对话框



CPSINSFIMUDlg::CPSINSFIMUDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_PSINS_FIMU_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPSINSFIMUDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_IMUCOM, combo_imucom);
	DDX_Control(pDX, IDC_COMBO_IMUBDR, combo_imubdr);
	DDX_Control(pDX, IDC_COMBO_GPSCOM, combo_gpscom);
	DDX_Control(pDX, IDC_COMBO_GPSBDR, combo_gpsbdr);
	DDX_Control(pDX, IDC_COMBO_ALIGNMETHOD, combo_align);
	DDX_Control(pDX, IDC_COMBO_ALIGNTIME, combo_aligntime);
	DDX_Control(pDX, IDC_COMBO_FILETYPE, combo_filetype);
	DDX_Control(pDX, IDC_COMBO_INITSOURCE, combo_initsource);
	DDX_Control(pDX, IDC_COMBO_NAVMETHOD, combo_navimethod);
	DDX_Control(pDX, IDC_EDIT_NAVRES_AX, NavAx);
	DDX_Control(pDX, IDC_EDIT_NAVRES_AY, NavAy);
	DDX_Control(pDX, IDC_EDIT_NAVRES_AZ, NavAz);
	DDX_Control(pDX, IDC_EDIT_NAVRES_VX, NavVy);
	DDX_Control(pDX, IDC_EDIT_NAVRES_VY, NavVx);
	DDX_Control(pDX, IDC_EDIT_NAVRES_VZ, NavVz);
	DDX_Control(pDX, IDC_EDIT_NAVRES_PX, NavPx);
	DDX_Control(pDX, IDC_EDIT_NAVRES_PY, NavPy);
	DDX_Control(pDX, IDC_EDIT_NAVRES_PZ, NavPz);
	DDX_Control(pDX, IDC_EDIT_IMUWX, IMUWx);
	DDX_Control(pDX, IDC_EDIT_IMUWY, IMUWy);
	DDX_Control(pDX, IDC_EDIT_IMUWZ, IMUWz);
	DDX_Control(pDX, IDC_EDIT_IMUFX, IMUFx);
	DDX_Control(pDX, IDC_EDIT_IMUFY, IMUFy);
	DDX_Control(pDX, IDC_EDIT_IMUFZ, IMUFz);
	DDX_Control(pDX, IDC_BUTTON_OPEN, Btn_open);
	DDX_Control(pDX, IDC_BUTTON_START, Btn_start);
	DDX_Control(pDX, IDC_BUTTON_EXIT, Btn_exit);
	DDX_Control(pDX, IDC_CHART_ATTX, ChartAttx);
	DDX_Control(pDX, IDC_CHART_ATTY, ChartAtty);
	DDX_Control(pDX, IDC_CHART_ATTZ, ChartAttz);
	DDX_Control(pDX, IDC_CHART_VELX, ChartVelx);
	DDX_Control(pDX, IDC_CHART_VELY, ChartVely);
	DDX_Control(pDX, IDC_CHART_VELZ, ChartVelz);
	DDX_Control(pDX, IDC_CHART_POS, ChartPos);
	DDX_Control(pDX, IDC_CHART_ALT, ChartAlt);
	DDX_Control(pDX, IDC_CHART_GYRO, ChartGyro);
	DDX_Control(pDX, IDC_CHART_ACCE, ChartAcce);
	DDX_Control(pDX, IDC_STATIC_TYEAR, Timer_year);
	DDX_Control(pDX, IDC_STATIC_TMON, Timer_month);
	DDX_Control(pDX, IDC_STATIC_TDAY, Timer_day);
	DDX_Control(pDX, IDC_STATIC_THOUR, Timer_hour);
	DDX_Control(pDX, IDC_STATIC_TMIN, Timer_minute);
	DDX_Control(pDX, IDC_STATIC_TSEC, Timer_second);
	DDX_Control(pDX, IDC_STATIC_TCOUNTER, Counter);
	DDX_Control(pDX, IDC_BUTTON_TIMERPAUSE, BtnPause);
	DDX_Control(pDX, IDC_BUTTON_TIMERRESET, BtnReset);
	DDX_Control(pDX, IDC_STATIC_TITLE, bigTitle);
}

BEGIN_MESSAGE_MAP(CPSINSFIMUDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_MESSAGE(WM_SERIALMSG, &CPSINSFIMUDlg::OnSerialMsg)		// 串口对象消息
	ON_MESSAGE(NAV_INITVP, &CPSINSFIMUDlg::OnI3DlgMsg)
	ON_MESSAGE(NT_AVPUDT, &CPSINSFIMUDlg::OnAVPUdt)
	ON_CBN_SELCHANGE(IDC_COMBO_IMUCOM, &CPSINSFIMUDlg::OnCbnSelchangeComboImucom)
	ON_CBN_SELCHANGE(IDC_COMBO_IMUBDR, &CPSINSFIMUDlg::OnCbnSelchangeComboImubdr)
	ON_CBN_SELCHANGE(IDC_COMBO_GPSCOM, &CPSINSFIMUDlg::OnCbnSelchangeComboGpscom)
	ON_CBN_SELCHANGE(IDC_COMBO_GPSBDR, &CPSINSFIMUDlg::OnCbnSelchangeComboGpsbdr)
	ON_CBN_SELCHANGE(IDC_COMBO_ALIGNMETHOD, &CPSINSFIMUDlg::OnCbnSelchangeComboAlignmethod)
	ON_CBN_SELCHANGE(IDC_COMBO_ALIGNTIME, &CPSINSFIMUDlg::OnCbnSelchangeComboAligntime)
	ON_BN_CLICKED(IDC_BUTTON_OPEN, &CPSINSFIMUDlg::OnBnClickedButtonOpen)
	ON_BN_CLICKED(IDC_BUTTON_START, &CPSINSFIMUDlg::OnBnClickedButtonStart)
	ON_BN_CLICKED(IDC_BUTTON_EXIT, &CPSINSFIMUDlg::OnBnClickedButtonExit)
	ON_BN_CLICKED(IDC_BUTTON_TIMERPAUSE, &CPSINSFIMUDlg::OnClickedBtnPause)
	ON_BN_CLICKED(IDC_BUTTON_TIMERRESET, &CPSINSFIMUDlg::OnClickedBtnSetzero)
	ON_CBN_SELCHANGE(IDC_COMBO_FILETYPE, &CPSINSFIMUDlg::OnCbnSelchangeComboFiletype)
	ON_CBN_SELCHANGE(IDC_COMBO_INITSOURCE, &CPSINSFIMUDlg::OnCbnSelchangeComboInitsource)
	ON_CBN_SELCHANGE(IDC_COMBO_NAVMETHOD, &CPSINSFIMUDlg::OnCbnSelchangeComboNavmethod)
END_MESSAGE_MAP()


// CPSINSFIMUDlg 消息处理程序

BOOL CPSINSFIMUDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	//============ 控 件 初 始 化 =================//
	combo_imucom.SetCurSel(0);			// IMU-->COM1
	combo_imubdr.SetCurSel(6);			// IMUBDR-->460400
	combo_gpscom.SetCurSel(1);			// GPS-->COM2
	combo_gpsbdr.SetCurSel(4);			// GPSBDR-->115200
	combo_align.SetCurSel(1);			// Align-->Moving Base
	combo_aligntime.SetCurSel(1);		// AlignTime = 180s
	combo_initsource.SetCurSel(0);		// Initalize navigation information with inputing
	combo_navimethod.SetCurSel(0);		// Default Navigation: inertial navigation
	Btn_start.EnableWindow(FALSE);
	imuDataUpdate();
	navDataUpdate();
	titleFont.CreateFont(32,0,0,0,FW_DONTCARE, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PALETTE, L"system");
	bigTitle.SetFont(&titleFont);
	//===========================================//

	//============ 串 口 初 始 化 =================//
	serialport = SerialPort();
	serialport.checkb = 0;				// 设置串口为1位停止位+无校验
	serialport.checkbG = 0;
	serialport.stopb = 0;
	serialport.stopbG = 0;
	OnCbnSelchangeComboImucom();
	OnCbnSelchangeComboImubdr();
	OnCbnSelchangeComboGpscom();
	OnCbnSelchangeComboGpsbdr();
	rcvCounter = 1;
	serialport.hwndOfComm = (AfxGetMainWnd())->m_hWnd;
	//===========================================//

	//============ 导 航 初 始 化 =================//
	alignState = combo_align.GetCurSel();		// Must be executed after combobox initialization
	insState = combo_navimethod.GetCurSel();
	initSource = combo_initsource.GetCurSel();
	fileState = combo_filetype.GetCurSel();
	navithread = (NaviThread*)AfxBeginThread(RUNTIME_CLASS(NaviThread));
	OnCbnSelchangeComboAlignmethod();
	OnCbnSelchangeComboAligntime();
	OnCbnSelchangeComboFiletype();
	OnCbnSelchangeComboInitsource();
	OnCbnSelchangeComboNavmethod();
	//===========================================//

	//============ 画 图 初 始 化 =================//
	ChartAttx.chartInit(_T("俯仰角"), 10, 1);
	ChartAtty.chartInit(_T("横滚角"), 10, 1);
	ChartAttz.chartInit(_T("航向角"), 10, 1);
	ChartVelx.chartInit(_T("东向速度"), 10, 1);
	ChartVely.chartInit(_T("北向速度"), 10, 1);
	ChartVelz.chartInit(_T("天向速度"), 10, 1);
	ChartPos.chartInit(_T("水平位置"), 10);
	ChartAlt.chartInit(_T("高度"), 10, 1);
	ChartGyro.chartInit(_T("角速度"), 10, 3);
	ChartAcce.chartInit(_T("比力"), 10, 3);
	//===========================================//

	//==============  定 时 器  =================//
	SetLocalTime();
	SetTimer(1, 1000, 0);
	SetTimer(2, 1000, 0);
	Counter.SetText(_T("0000"));
	rcvCounter = 1;			// 计数重置
	//===========================================//

	//============ 文 件 初 始 化 ================//
	navidata = NaviDataStore();
	//===========================================//

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CPSINSFIMUDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CPSINSFIMUDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CPSINSFIMUDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

/************************************   Digi-Panel  **************************************/
/*    Referesh Digi-Panel  (avp+imu+timer)                                               */
/*****************************************************************************************/
void CPSINSFIMUDlg::imuDataUpdate()
{
	CString navTmp;
	navTmp.Format(_T("%6.3f"), imudata.wx);
	IMUWx.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), imudata.wy);
	IMUWy.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), imudata.wz);
	IMUWz.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), imudata.fx);
	IMUFx.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), imudata.fy);
	IMUFy.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), imudata.fz);
	IMUFz.SetText(navTmp);
}

void CPSINSFIMUDlg::navDataUpdate()
{
	CString navTmp;
	navTmp.Format(_T("%6.3f"), dlgAVPt[0]);
	NavAx.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[1]);
	NavAy.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[2]);
	NavAz.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[3]);
	NavVx.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[4]);
	NavVy.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[5]);
	NavVz.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[6]);
	NavPx.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[7]);
	NavPy.SetText(navTmp);
	navTmp.Format(_T("%6.3f"), dlgAVPt[8]);
	NavPz.SetText(navTmp);
}

void CPSINSFIMUDlg::imuChartUpdate()
{
	double tk = rcvCounter * TS;
	double data1[3] = { imudata.wx, imudata.wy, imudata.wz };
	double data2[3] = { imudata.fx, imudata.fy, imudata.fz };
	ChartGyro.chartUpdate(tk, data1);
	ChartAcce.chartUpdate(tk, data2);
}

void CPSINSFIMUDlg::navChartUpdate()
{
	double tk = rcvCounter * TS;
	double dpE, dpN;
	dpE = (dlgAVPt[7] - dlgAVP0[7]) * DEG * cos(dlgAVPt[6] * DEG) * RE;
	dpN = (dlgAVPt[6] - dlgAVP0[6]) * DEG * RE;
	ChartAttx.chartUpdate(tk, dlgAVPt+0);
	ChartAtty.chartUpdate(tk, dlgAVPt+1);
	ChartAttz.chartUpdate(tk, dlgAVPt+2);
	ChartVelx.chartUpdate(tk, dlgAVPt+3);
	ChartVely.chartUpdate(tk, dlgAVPt+4);
	ChartVelz.chartUpdate(tk, dlgAVPt+5);
	ChartPos.chartUpdate(dpE, dpN);
	ChartAlt.chartUpdate(tk, dlgAVPt + 8);
}
/********************************   END DIGI-PANEL  **************************************/

/**************************************   START  *****************************************/
/*    Start Serial Communication & Close Dlg                                             */
/*****************************************************************************************/
void CPSINSFIMUDlg::OnBnClickedButtonOpen()
{
	// “打开串口”按钮响应程序
	int openRes;
	char fileName[64] = "";
	exitCode = 0;
	if (!serialport.isOpen)
	{
		// ============================= IMU 串口打开 ======================================= //
		//============ 串 口 初 始 化 =================//
		serialport.checkb = 0;				// 设置串口为1位停止位+无校验
		serialport.checkbG = 0;
		serialport.stopb = 0;
		serialport.stopbG = 0;
		OnCbnSelchangeComboImucom();
		OnCbnSelchangeComboImubdr();
		OnCbnSelchangeComboGpscom();
		OnCbnSelchangeComboGpsbdr();
		serialport.hwndOfComm = (AfxGetMainWnd())->m_hWnd;
		//============ 画 图 初 始 化 =================//
		ChartGyro.chartReinit();		
		ChartAcce.chartReinit();
		//===========================================//
		openRes = serialport.OpenCOM();
		//SYSCNT = 0;			// 系统计数重新清零
		if (openRes == 0)
		{
			// Initial File Work
			Btn_open.SetWindowText(_T("关闭串口"));
			navidata.createDirectory(&st);
			navidata.createFile(&st, navidata.COM);
			navidata.createFile(&st, navidata.IMU);
			navidata.createFile(&st, navidata.RES);
			serialport.isIMUOpen = TRUE;
			// Create Thread
			threadPara = (LPVOID)&serialport;
			isRunningIMU = 1;
			TP_CommThread = CreateThread(NULL, 0, SerialThread, threadPara, THREAD_PRIORITY_NORMAL, 0);
			combo_imucom.EnableWindow(FALSE);		// 设置串口参数不可更改
			combo_imubdr.EnableWindow(FALSE);
			Btn_start.EnableWindow(TRUE);
		}
		else
		{
			MessageBox(_T("IMU串口打开失败！"), _T("Warrning"), MB_OK);
		}
		// ============================= GPS 串口打开 ======================================= //
		if (insState == 1)			// If choose inertial navigation, GPS serial port don't need use
		{
			openRes = serialport.OpenGPSCOM();
			if (openRes == 0)
			{
				// Initial File Work
				Btn_open.SetWindowText(_T("关闭串口"));
				serialport.isOpen = TRUE;
				serialport.isGPSOpen = TRUE;
				navidata.createFile(&st, navidata.GPS);
				// Create Thread
				threadPara = (LPVOID)&serialport;
				isRunningGPS = 1;
				GPS_CommThread = CreateThread(NULL, 0, SerialThreadGPS, threadPara, THREAD_PRIORITY_NORMAL, 0);
				combo_gpscom.EnableWindow(FALSE);
				combo_gpsbdr.EnableWindow(FALSE);
			}
			else
			{
				MessageBox(_T("GPS串口打开失败！"), _T("Warrning"), MB_OK);
				if (alignState == 2)
				{
					combo_align.SetCurSel(2);
					MessageBox(_T("GPS未能打开"), _T("Warrning"), MB_OK);
				}
			}
			serialport.isOpen = serialport.isIMUOpen || serialport.isGPSOpen;
			isRunningMsg = serialport.isIMUOpen || serialport.isGPSOpen;
		}
		else
		{
			serialport.isOpen = serialport.isIMUOpen;
			isRunningMsg = serialport.isIMUOpen;
		}
	}
	else
	{
		int rtn = MessageBox(_T("数据采集中，确定要关闭串口？"), _T("工作中"), MB_OKCANCEL);
		if (rtn == IDOK)
		{
			BOOL isCloseThreadIMU, isCloseThreadGPS;
			isRunningMsg = FALSE;
			if (serialport.isGPSOpen == TRUE)
			{
				serialport.isGPSOpen = FALSE;
				isRunningGPS = 0;
				isCloseThreadGPS = CloseHandle(GPS_CommThread);
				if (!isCloseThreadGPS)
				{
					int rtn = MessageBox(_T("GPS串口线程结束异常，是否强制关闭？"), _T("串口关闭异常"), MB_OKCANCEL);
					if (rtn == IDOK)
					{
						CloseHandle(GPS_CommThread);
					}
				}
				serialport.CloseGPSCOM();				// 关闭串口
				serialport.isGPSOpen = FALSE;
				navidata.closeFile(navidata.GPS);
				combo_gpscom.EnableWindow(TRUE);
				combo_gpsbdr.EnableWindow(TRUE);
			}

			if (serialport.isIMUOpen == TRUE)
			{
				serialport.isIMUOpen = FALSE;
				isNaviRunning = FALSE;
				Btn_start.EnableWindow(FALSE);
				isRunningIMU = 0;
				isCloseThreadIMU = CloseHandle(TP_CommThread);
				if (!isCloseThreadIMU)
				{
					int rtn = MessageBox(_T("IMU串口线程结束异常，是否强制关闭？"), _T("串口关闭异常"), MB_OKCANCEL);
					if (rtn == IDOK)
					{
						CloseHandle(TP_CommThread);
					}
				}
				serialport.CloseCOM();				// 关闭串口
				navidata.closeFile(navidata.COM);
				navidata.closeFile(navidata.IMU);
				navidata.closeFile(navidata.RES);
				combo_imucom.EnableWindow(TRUE);		// 设置串口参数可更改
				combo_imubdr.EnableWindow(TRUE);

			}
			Btn_open.SetWindowText(_T("打开串口"));
			serialport.isOpen = FALSE;
		}
	}
}

void CPSINSFIMUDlg::OnBnClickedButtonStart()
{
	//============ 导 航 初 始 化 =================//
	alignState = combo_align.GetCurSel();		// Must be executed after combobox initialization
	insState = combo_navimethod.GetCurSel();
	initSource = combo_initsource.GetCurSel();
	fileState = combo_filetype.GetCurSel();
	OnCbnSelchangeComboAlignmethod();
	OnCbnSelchangeComboAligntime();
	OnCbnSelchangeComboFiletype();
	OnCbnSelchangeComboInitsource();
	OnCbnSelchangeComboNavmethod();
	navithread->reInit();			// 重置标志位
	//============ 画 图 初 始 化 =================//
	ChartAttx.chartReinit();
	ChartAtty.chartReinit();
	ChartAttz.chartReinit();
	ChartVelx.chartReinit();
	ChartVely.chartReinit();
	ChartVelz.chartReinit();
	ChartPos.chartReinit();
	ChartAlt.chartReinit();
	//===========================================//
	if (initSource == 0)
	{
		I3Dlg.DoModal();
	}
	Counter.SetText(_T("0000"));
	rcvCounter = 1;			// 计数重置
	isNaviRunning = TRUE;
	Btn_start.EnableWindow(FALSE);
}

void CPSINSFIMUDlg::OnBnClickedButtonExit()
{
	// “退出程序”按钮响应程序
	if (serialport.isOpen == TRUE)
	{
		MessageBox(_T("串口处于打开状态，是否继续退出程序？"), _T("Warrning"), MB_OKCANCEL);
	}
	else
	{
		navithread->Delete();
		EndDialog(0);
	}
}
/************************************   END START  ***************************************/


/************************************ InputDlg Msg ***************************************/
/* Transport the initialization information                                              */
/*****************************************************************************************/
LRESULT CPSINSFIMUDlg::OnI3DlgMsg(WPARAM wParam, LPARAM lParam)
{
	int i = (int)lParam;
	if (i < 9)
	{
		dlgAVP0[i] = *(double*)wParam;
		dlgAVPt[i] = *(double*)wParam;
	}
	else
	{
		threadpara1 = (WPARAM)(&dlgAVP0);
		navithread->PostThreadMessage(NT_ALIGNINIT, threadpara1, threadpara2);
	}
	return LRESULT();
}

LRESULT CPSINSFIMUDlg::OnAVPUdt(WPARAM wParam, LPARAM lParam)
{
	for (size_t i = 0; i < 9; i++)
	{
		dlgAVPt[i] = *((double*)wParam + i);
	}
	return LRESULT();
}
/************************************   END START  ***************************************/


/************************************** Serial Msg ***************************************/
/* Two threads used to get IMU and GPS data                                              */
/*****************************************************************************************/
LRESULT CPSINSFIMUDlg::OnSerialMsg(WPARAM wParam, LPARAM lParam)
{
	if (isRunningMsg)
	{
		serialport.isResolving = TRUE;		// 正在解析数据
		unsigned int msg_code = *(unsigned int*)wParam;
		if (msg_code == serialport.msg_getpkg)
		{
			unsigned char CHK;
			memcpy(pkgbuffer, serialport.buffer[serialport.buftagR], serialport.pkgLen);		// 主线获取数据
			SYSCNT++;			// 以IMU采样为基础的计数时间戳
			CHK = (unsigned char)IMURcvChk();
			memcpy(&imudata.counter, &pkgbuffer[0], sizeof(unsigned char));
			memcpy(&imudata.CS, &pkgbuffer[1], sizeof(char));
			memcpy(&imudata.Ax, &pkgbuffer[2], sizeof(int));
			memcpy(&imudata.Ay, &pkgbuffer[6], sizeof(int));
			memcpy(&imudata.Az, &pkgbuffer[10], sizeof(int));
			memcpy(&imudata.Gx, &pkgbuffer[14], sizeof(int));
			memcpy(&imudata.Gy, &pkgbuffer[18], sizeof(int));
			memcpy(&imudata.Gz, &pkgbuffer[22], sizeof(int));
			memcpy(&imudata.Fx, &pkgbuffer[26], sizeof(int));
			memcpy(&imudata.Fy, &pkgbuffer[30], sizeof(int));
			memcpy(&imudata.Fz, &pkgbuffer[34], sizeof(int));
			memcpy(&imudata.Wx, &pkgbuffer[38], sizeof(int));
			memcpy(&imudata.Wy, &pkgbuffer[42], sizeof(int));
			memcpy(&imudata.Wz, &pkgbuffer[46], sizeof(int));
			imudata.wx = imudata.Wx / (double)100000.0;
			imudata.wy = imudata.Wy / (double)100000.0;
			imudata.wz = imudata.Wz / (double)100000.0;
			imudata.fx = imudata.Fx / (double)100000.0;
			imudata.fy = imudata.Fy / (double)100000.0;
			imudata.fz = imudata.Fz / (double)100000.0;
			imudata.wxts = imudata.wx * DEG * TS;
			imudata.wyts = imudata.wy * DEG * TS;
			imudata.wzts = imudata.wz * DEG * TS;
			imudata.fxts = imudata.fx * TS;
			imudata.fyts = imudata.fy * TS;
			imudata.fzts = imudata.fz * TS;

			navithread->wmi[rcvCounter % Samples] = &imudata.wxts;		// rad
			navithread->vmi[rcvCounter % Samples] = &imudata.fxts;		// m/s

			navidata.saveData(navidata.IMU, &imudata.wx);
			rcvCounter++;
			
			if (rcvCounter % Samples == 0)
			{
				if (isNaviRunning == TRUE)
				{
					if (!navithread->AlignFished)				// 对准初始化未完成
					{
						if (navithread->AlignInitFinish==FALSE && alignState<=1)		// 对准初始化未完成
						{
							threadpara1 = (WPARAM)&rcvCounter;
							navithread->PostThreadMessage(NT_ALIGNINIT, threadpara1, threadpara2);
						}
						else									// 对准初始化完成
						{
							navithread->PostThreadMessage(NT_ALIGNUPDATE, threadpara1, threadpara2);
							if (rcvCounter * TS >= alignTime)
							{
								navithread->AlignFished = TRUE;
								threadpara1 = (WPARAM)&rcvCounter;
								navithread->PostThreadMessage(NT_NaviINIT, threadpara1, threadpara2);
							}
						}
					}
					else										// 对准完成
					{
						if (!navithread->NaviInitFinished)	// 导航初始化未完成
						{
							threadpara1 = (WPARAM)&rcvCounter;
							navithread->PostThreadMessage(NT_NaviINIT, threadpara1, threadpara2);
						}
						else									// 导航初始化完成
						{
							if (rcvCounter * TS <= (alignTime * 2))
							{
								threadpara1 = (WPARAM)&rcvCounter;
								navithread->PostThreadMessage(NT_KFTUPDATE, threadpara1, threadpara2);
								if (rcvCounter % 400 == 0)
								{
									threadpara1 = (WPARAM)&rcvCounter;
									threadpara2 = (LPARAM)0;
									navithread->PostThreadMessage(NT_KFMUPDATE, threadpara1, threadpara2);
								}
							}
							else
							{
								threadpara1 = (WPARAM)&rcvCounter;
								navithread->PostThreadMessage(NT_NAVIUPDATE, threadpara1, threadpara2);
							}
						}
						
					}
				}
			}

			// 将数据加入折线图并刷新(0.1s)
			if (rcvCounter % 100 == 0)
			{
				//float imu1pkg[6] = {imudata.wx, imudata.wy, imudata.wz, imudata.fx, imudata.fy, imudata.fz };
				imuDataUpdate();
				imuChartUpdate();
				navDataUpdate();
				navChartUpdate();
			}

			if (CHK == FALSE)
			{
				SendMessage(WM_DATAERRMSG, (WPARAM)&msg_errchkfaile, (LPARAM)&SYSCNT);
			}

			// Write Data 2 IMUFile

			if (GPGGAOK && BESTVELOK)
			{
				if (navithread->AlignInitFinish == FALSE && initSource == 1 && alignState == 2)	// 未进行对准初始化并且选择GNSS数据初始化
				{
					navithread->PostThreadMessage(NT_ALIGNINIT, threadpara1, threadpara2);
					navithread->AlignInitFinish = TRUE;
				}
				else
				{
					if (insState == 1 && rcvCounter * TS > (alignTime * 2))
					{
						navithread->PostThreadMessage(NT_KFMUPDATE, threadpara1, threadpara2);
					}
				}

				GPGGAOK = FALSE;
				BESTVELOK = FALSE;
			}
			else
			{

			}


			// 每1s插入一次数据
			if ((rcvCounter) % 400 == 0)
			{
				
			}
		}
		else if (msg_code == serialport.msg_getpkgfail)
		{
			MessageBox(_T("串口数据读取异常"), _T("通信异常"), MB_ICONWARNING);
		}
		else if (msg_code == serialport.msg_getGPGGA)	// GPS数据处理函数
		{
			int pkglen = lParam;		// 从消息中获取数据长度

			memcpy(pkgbufferG0, serialport.gpsbuffer0, pkglen);		// 主线获取数据
			sscanf_s(pkgbufferG0, ",%f,%lf,%c,%lf,%c,%hd,%hd,%f,%f,M,%f,M,%f,%hd*%hd\r",
				&gpgga.time, &gpgga.Lat, &gpgga.NS, 1, &gpgga.Lng, &gpgga.WE, 1, &gpgga.state, &gpgga.SatNum, &gpgga.HDOP,
				&gpgga.Alt, &gpgga.Herr, &gpgga.xx, &gpgga.BaseNum, &gpgga.chk);
			navidata.saveData(navidata.GPS, pkgbufferG0);
			GPGGAOK = TRUE;
		}
		else if (msg_code == serialport.msg_getBESTVEL)
		{
			int pkglen = lParam;		// 从消息中获取数据长度

			memcpy(pkgbufferG1, serialport.gpsbuffer1, pkglen);		// 主线获取数据
			sscanf_s(pkgbufferG1, ",%[^;];%[^,],%[^,],%f,%f,%lf,%lf,%lf,%f*%s\r",
				bestvel.header, 128, bestvel.SOL_STATUS, 32, bestvel.VEL_TYPE, 32, &bestvel.latency, &bestvel.age, &bestvel.vel_h,
				&bestvel.heading, &bestvel.vel_v, &bestvel.xx, bestvel.CRC, 16);
			navidata.saveData(navidata.GPS, pkgbufferG1);

			if (!strcmp(bestvel.SOL_STATUS, "SOL_COMPUTED"))
				bestvel.SS = 0;
			else if (!strcmp(bestvel.SOL_STATUS, "INSUFFICIENT_OBS"))
				bestvel.SS = 1;
			else if (!strcmp(bestvel.SOL_STATUS, "NO_CONVERGENCE"))
				bestvel.SS = 2;
			else if (!strcmp(bestvel.SOL_STATUS, "COV_TRACE"))
				bestvel.SS = 3;

			BESTVELOK = TRUE;
		}
		serialport.isResolving = FALSE;		// 一包数据解析完毕
	}
	return LRESULT();
}
/*************************************   END MSG  ****************************************/


/**************************************** Therad *****************************************/
/* Two threads used to get IMU and GPS data                                              */
/*****************************************************************************************/
DWORD WINAPI CPSINSFIMUDlg::SerialThread(LPVOID lpara)
{
	static SerialPort* serialPort = (SerialPort*)lpara;
	static BOOL isGetPkgHead = FALSE;
	while (isRunningIMU)
	{
		isGetPkgHead = serialPort->findHead();
		if (isGetPkgHead)
		{
			serialPort->readPkg();
		}
	}
	isRunningIMU = 2;
	return 0;
}

DWORD WINAPI CPSINSFIMUDlg::SerialThreadGPS(LPVOID lpara)
{
	static SerialPort* serialPort = (SerialPort*)lpara;
	static BOOL isGetPkgHead = FALSE;
	while (isRunningGPS)
	{
		isGetPkgHead = serialPort->findGPSHead();
		if (isGetPkgHead)
		{
			serialPort->readGPSPkg();
		}
	}
	isRunningGPS = 2;
	return 0;
}
/************************************   END THREAD  **************************************/


/**************************************** TIMER ******************************************/
/* Update System Time & TIMER                                                            */
/*****************************************************************************************/
// 定时处理程序
void CPSINSFIMUDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值
	switch (nIDEvent)
	{
	case 1:
		SetLocalTime();
		break;
	case 2:
		if ((!counterPause) && isNaviRunning)
		{
			time_tmp.Format(_T("%4d"), ++secCounter);
			Counter.SetText(time_tmp);
		}
		break;
	}
	CDialogEx::OnTimer(nIDEvent);
}

void CPSINSFIMUDlg::SetLocalTime()
{
	GetLocalTime(&st);
	time_tmp.Format(_T("%4d"), st.wYear);
	Timer_year.SetText(time_tmp);
	time_tmp.Format(_T("%2d"), st.wMonth);
	Timer_month.SetText(time_tmp);
	time_tmp.Format(_T("%2d"), st.wDay);
	Timer_day.SetText(time_tmp);
	time_tmp.Format(_T("%2d"), st.wHour);
	Timer_hour.SetText(time_tmp);
	time_tmp.Format(_T("%2d"), st.wMinute);
	Timer_minute.SetText(time_tmp);
	time_tmp.Format(_T("%2d"), st.wSecond);
	Timer_second.SetText(time_tmp);
}

void CPSINSFIMUDlg::OnClickedBtnPause()
{
	// 定时器暂停与恢复
	counterPause = !counterPause;
	if (counterPause) {
		BtnPause.SetWindowText(_T("计时恢复"));
	}
	else {
		BtnPause.SetWindowText(_T("计时暂停"));
	}
}


void CPSINSFIMUDlg::OnClickedBtnSetzero()
{
	// 将定时器清零
	secCounter = 0;
	Counter.SetText(_T("0000"));
}
/************************************   END TIMER  ***************************************/


BOOL CPSINSFIMUDlg::IMURcvChk()
{
	unsigned char i, chkref, chksum = 0;
	for (i = 0; i < serialport.pkgLen; i++)
	{
		chksum += pkgbuffer[i];
	}
	chkref = pkgbuffer[1];
	chksum = chksum - chkref + 255;		// 减去校验和加上帧头数据
	if (chksum == chkref)
		return TRUE;
	else
		return FALSE;
}

/*************************************** Settings ****************************************/
/* Set the COM number, BDR and alignment method                                          */
/*****************************************************************************************/
void CPSINSFIMUDlg::OnCbnSelchangeComboImucom()
{
	int idx = combo_imucom.GetCurSel();
	CString lab, lab10("\\\\.\\");
	combo_imucom.GetLBText(idx, lab);
	if (idx >= 9)
	{
		lab = lab10 + lab;
	}
	serialport.ComNum = lab;
}


void CPSINSFIMUDlg::OnCbnSelchangeComboImubdr()
{
	int idx = combo_imubdr.GetCurSel();
	CString lab;
	combo_imubdr.GetLBText(idx, lab);
	serialport.bdr = _ttoi(lab);
}


void CPSINSFIMUDlg::OnCbnSelchangeComboGpscom()
{
	int idx = combo_gpscom.GetCurSel();
	CString lab, lab10("\\\\.\\");
	combo_gpscom.GetLBText(idx, lab);
	if (idx >= 9)
	{
		lab = lab10 + lab;
	}
	serialport.ComNumG = lab;
}


void CPSINSFIMUDlg::OnCbnSelchangeComboGpsbdr()
{
	int idx = combo_gpsbdr.GetCurSel();
	CString lab;
	combo_gpsbdr.GetLBText(idx, lab);
	serialport.bdrG = _ttoi(lab);
}


void CPSINSFIMUDlg::OnCbnSelchangeComboAlignmethod()
{
	alignState = combo_align.GetCurSel();
}


void CPSINSFIMUDlg::OnCbnSelchangeComboAligntime()
{
	int idx = combo_aligntime.GetCurSel();
	CString lab;
	combo_aligntime.GetLBText(idx, lab);
	alignTime = (UINT)_ttoi(lab);
}

void CPSINSFIMUDlg::OnCbnSelchangeComboFiletype()
{
	fileState = combo_filetype.GetCurSel();
}

void CPSINSFIMUDlg::OnCbnSelchangeComboInitsource()
{
	initSource = combo_initsource.GetCurSel();
}


void CPSINSFIMUDlg::OnCbnSelchangeComboNavmethod()
{
	insState = combo_navimethod.GetCurSel();
}


/*********************************** SETTING END *****************************************/








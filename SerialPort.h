#pragma once

#define WM_SERIALMSG WM_USER+100		// SerialPort Msg
#define WM_DATAERRMSG WM_USER+101		// 数据处理异常处理

class SerialPort
{
public:
	CString ComNum;
	int bdr;
	int stopb;
	int checkb;
	DCB COMPara;
	CString ComNumG;
	int bdrG;
	int stopbG;
	int checkbG;
	DCB COMParaG;
	HANDLE hCOM, hCOMG;
	UCHAR pkgHead[2] = { 0xAA, 0x55 };		// FIMU帧头
	CHAR  pkgHeadPOS[5] = { 'G','N','G','G','A' }; // GPGGA
	CHAR  pkgHeadVEL[8] = { 'B','E','S','T','V','E','L','A' }; // BESTVEL
	BOOL GPSPOSVEL[2] = { FALSE, FALSE };
	BOOL isOpen = FALSE;
	BOOL isIMUOpen = FALSE;
	BOOL isGPSOpen = FALSE;
	BOOL isResolving = FALSE;

	unsigned int pkgLen = 50;		//数据包有效长度（除去帧头）
	UCHAR rByte1;			// FIMU十六进制数据用
	CHAR  rByte2;			// GPS字符数据用
	UCHAR buffer[2][64];		//数据包双缓冲区
	CHAR gpsbuffer0[512];		// GPS数据缓冲区(GPGGA)
	CHAR gpsbuffer1[512];		// GPS数据缓冲区(BESTVEL)
	char buftagR = 0;
	char buftagW = 0;
	BOOL ReadPkg = FALSE;
	
	HWND hwndOfComm;
	UINT msg_getpkg = 101;
	UINT msg_getpkgfail = 102;
	UINT msg_getGPGGA = 201;
	UINT msg_getBESTVEL = 301;

	int OpenCOM();
	BOOL CloseCOM();
	BOOL findHead();
	void readPkg();

	int OpenGPSCOM();
	BOOL CloseGPSCOM();
	BOOL findGPSHead();
	void readGPSPkg();
};


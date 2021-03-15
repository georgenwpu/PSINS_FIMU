#include "pch.h"
#include "SerialPort.h"

int SerialPort::OpenCOM()
{
	hCOM = CreateFile(ComNum, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hCOM == INVALID_HANDLE_VALUE)
	{
		return -1;
	}
	else
	{
		COMPara.BaudRate = bdr;
		COMPara.ByteSize = 8;
		COMPara.StopBits = stopb;
		COMPara.Parity = checkb;
		SetCommState(hCOM, &COMPara);
		SetupComm(hCOM, 1024, 1024);				//开辟10M的缓存空间				
		PurgeComm(hCOM, PURGE_TXCLEAR | PURGE_RXCLEAR);
		return 0;
	}
}

BOOL SerialPort::CloseCOM()
{
	BOOL closeComRes;
	closeComRes = CloseHandle(hCOM);
	return closeComRes;
}

BOOL SerialPort::findHead()
{
	DWORD wCount;
	BOOL bReadState;
	bReadState = ReadFile(hCOM, &rByte1, 1, &wCount, NULL);
	if (bReadState && rByte1 == pkgHead[0])	// AA 2E
	{
		bReadState = ReadFile(hCOM, &rByte1, 1, &wCount, NULL);
		if (bReadState && rByte1 == pkgHead[1])
			return TRUE;
	}
	return FALSE;
}

void SerialPort::readPkg()
{
	BOOL pReadState;
	DWORD wCount;
	HWND hwnd = hwndOfComm;
	pReadState = ReadFile(hCOM, buffer[buftagW], pkgLen, &wCount, NULL);
	if (pReadState)
	{
		buftagR = buftagW;
		buftagW = 1 - buftagW;		//缓冲区切换
		SendMessage(hwnd, WM_SERIALMSG, (WPARAM)&msg_getpkg, NULL);
	}
	else
	{
		SendMessage(hwnd, WM_SERIALMSG, (WPARAM)&msg_getpkgfail, NULL);
	}
}

int SerialPort::OpenGPSCOM()
{
	hCOMG = CreateFile(ComNumG, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hCOMG == INVALID_HANDLE_VALUE)
	{
		return -1;
	}
	else
	{
		COMParaG.BaudRate = bdrG;
		COMParaG.ByteSize = 8;
		COMParaG.StopBits = stopbG;
		COMParaG.Parity = checkbG;
		SetCommState(hCOMG, &COMParaG);
		SetupComm(hCOMG, 1024, 1024);				//开辟10M的缓存空间				
		PurgeComm(hCOMG, PURGE_TXCLEAR | PURGE_RXCLEAR);
		return 0;
	}
}

BOOL SerialPort::CloseGPSCOM()
{
	BOOL closeComRes;
	closeComRes = CloseHandle(hCOMG);
	return closeComRes;
}

BOOL SerialPort::findGPSHead()
{
	DWORD wCount;
	BOOL bReadState;
	GPSPOSVEL[0] = TRUE;
	GPSPOSVEL[1] = TRUE;
	bReadState = ReadFile(hCOMG, &rByte2, 1, &wCount, NULL);
	if (bReadState && ((rByte2=='$')||(rByte2 == '#')))
	{
		for (size_t i = 0; i < 8; i++)
		{
			bReadState = ReadFile(hCOMG, &rByte2, 1, &wCount, NULL);
			if (bReadState && rByte2 != pkgHeadPOS[i])	// GPGGA
				GPSPOSVEL[0] = FALSE;
			if (bReadState && rByte2 != pkgHeadVEL[i])	// BESTVEL
				GPSPOSVEL[1] = FALSE;
			if (GPSPOSVEL[0] == TRUE && i == 4)		// iIf get GPGGA, break the loop when i==6
				break;
		}
		if ((GPSPOSVEL[0] || GPSPOSVEL[1]) == FALSE)
			return FALSE;
		else
			return TRUE;
	}
	return FALSE;
}

void SerialPort::readGPSPkg()
{
	BOOL pReadState;
	DWORD wCount;
	char ch1='0', ch2='0';
	HWND hwnd = hwndOfComm;
	int i = 0;		// buf写指针
	LPARAM LenGpgga, LenBestvel;

	if ((GPSPOSVEL[0] == TRUE) && (GPSPOSVEL[1] == FALSE))		// 解析位置数据
	{
		unsigned char buflen = (UCHAR)511;
		while (buflen--)
		{
			pReadState = ReadFile(hCOMG, &ch2, 1, &wCount, NULL);
			if (pReadState)
			{
				gpsbuffer0[i++] = ch2;
				if (ch1 == '\r' && ch2 == '\n')
				{
					gpsbuffer0[i++] = '\0';		//字符串结束标志
					LenGpgga = i;
					SendMessage(hwnd, WM_SERIALMSG, (WPARAM)&msg_getGPGGA, LenGpgga);
					break;
				}
				ch1 = ch2;
			}
			else
			{
				SendMessage(hwnd, WM_SERIALMSG, (WPARAM)&msg_getpkgfail, NULL);
			}
		}
	}
	else if ((GPSPOSVEL[0] == FALSE) && (GPSPOSVEL[1] == TRUE))	// 解析速度数据
	{
		unsigned char buflen = (UCHAR)511;
		while (buflen--)
		{
			pReadState = ReadFile(hCOMG, &ch2, 1, &wCount, NULL);
			if (pReadState)
			{
				gpsbuffer1[i++] = ch2;
				if (ch1 == '\r' && ch2 == '\n')
				{
					gpsbuffer1[i++] = '\0';		//字符串结束标志
					LenBestvel = i;
					SendMessage(hwnd, WM_SERIALMSG, (WPARAM)&msg_getBESTVEL, LenBestvel);
					break;
				}
				ch1 = ch2;
			}
			else
			{
				SendMessage(hwnd, WM_SERIALMSG, (WPARAM)&msg_getpkgfail, NULL);
			}
		}
	}
}





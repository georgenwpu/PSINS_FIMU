#include "pch.h"
#include "NaviDataStore.h"

NaviDataStore::NaviDataStore()
{
}

//NaviDataStore::~NaviDataStore()
//{
//	if (wfpS != NULL)
//		fclose(wfpS);
//	if (wfpN != NULL)
//		fclose(wfpN);
//	if (wfpG != NULL)
//		fclose(wfpG);
//	if (wfpR != NULL)
//		fclose(wfpR);
//}

void NaviDataStore::createDirectory(SYSTEMTIME* st)
{
	sprintf_s(datetime, "%4d%02d%02d%02d%02d%02d", st->wYear, st->wMonth, st->wDay, st->wHour, st->wMinute, st->wSecond);
	datapath += datetime;
	if (!PathIsDirectory(datapath))
	{
		CreateDirectory(datapath,0);
	}
}

void NaviDataStore::closeFile(DataType dt)
{
	if (dt == COM)
	{
		if (wfpS != NULL)
			fclose(wfpS);
	}
	else if (dt == IMU)
	{
		if (wfpN != NULL)
			fclose(wfpN);
	}
	else if (dt == GPS)
	{
		if (wfpG != NULL)
			fclose(wfpG);
	}
	else if (dt == RES)
	{
		if (wfpG != NULL)
			fclose(wfpR);
	}
}

void NaviDataStore::createFile(SYSTEMTIME* st, DataType dt)
{
	if (dt == COM)
	{
		fopen_s(&wfpS, "COM_Data.bin", "wb");		// bin Serial
	}
	else if (dt == IMU)
	{
		fopen_s(&wfpN, "IMU_Data.bin", "wb");		// bin imu+gps
	}
	else if (dt == GPS)
	{
		fopen_s(&wfpG, "GNSS_Data.bin", "w");		// GPSÎÄ±¾¡¡¡¡GPGGA BESTVELA
	}
	else if (dt == RES)
	{
		fopen_s(&wfpR, "RESS_DATA.bin", "w");
	}
}

void NaviDataStore::saveData(DataType dt, void* data)
{
	if (dt == COM)
	{
		fwrite((double*)data, sizeof(double), 15, wfpS);
	}
	else if (dt == IMU)
	{
		fwrite((double*)data, sizeof(double), 15, wfpN);
	}
	else if (dt == GPS)
	{
		fprintf_s(wfpG, (char*)data);
	}
	else if (dt == RES)
	{
		fwrite((double*)data, sizeof(double), 9, wfpR);
	}
}


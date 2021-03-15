#pragma once

class NaviDataStore
{
public:
	NaviDataStore();


public:
	enum DataType
	{
		COM, IMU, GPS, RES
	} datatype;
	FILE* wfpS, * wfpN, * wfpG, * wfpR;
	BOOL saveasbin;
	char datetime[16];
	CString datapath = TEXT("D:\\FINS3X_DataDir\\");
	void createDirectory(SYSTEMTIME* st);
	void createFile(SYSTEMTIME* st, DataType dt );
	void closeFile(DataType dt);

	void saveData(DataType dt, void* dataa);
};


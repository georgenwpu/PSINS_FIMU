#pragma once
#include "pch.h"
#include "ChartCtrl_source/ChartLineSerie.h"
#include "ChartCtrl_source/ChartCtrl.h"

class NavChart : public CChartCtrl
{
public:
	NavChart();

public:
	TChartString title;
	UINT lineLen;
	UCHAR LINEN;
	INT FS;		// Update Frequency
	double t[1000] = { 0 };		// time tag
	double data[3][2000];	// data
	CChartLineSerie* pLS[3];
	COLORREF LineRGB[3] = {RGB(255,0,0), RGB(0,255,0), RGB(0,0,255)};

	void chartInit(TChartString chartTitle, int fs, UCHAR lineN);	// 图像对象初始化
	void chartUpdate(double tk, double* data);
	void chartReinit();
	void chartInit(TChartString chartTitle, int fs);
	void chartUpdate(double x, double y);
};


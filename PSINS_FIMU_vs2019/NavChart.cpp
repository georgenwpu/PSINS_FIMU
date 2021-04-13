#include "pch.h"
#include "NavChart.h"

NavChart::NavChart() :CChartCtrl()
{ }

void NavChart::chartInit(TChartString chartTitle, int fs, UCHAR lineN)
{
	title = chartTitle;
	FS = fs;
	lineLen = 20 * FS;
	LINEN = lineN;
	CChartAxis* pAxis = NULL;
	pAxis = CreateStandardAxis(CChartCtrl::BottomAxis);		// 坐标系初始化（不可缺少）
	pAxis->SetAutomatic(true);
	pAxis = CreateStandardAxis(CChartCtrl::LeftAxis);
	pAxis->SetAutomatic(true);

	GetTitle()->AddString(title);

	for (int i = 0; i < lineLen; i++)
	{
		t[i] = -20 + i * 0.1;
		for (int l=0; l< LINEN; l++)
			data[l][i] = 0;
	}
	RemoveAllSeries();	//先清空

	for (size_t i = 0; i < LINEN; i++)
	{
		pLS[i] = CreateLineSerie();
		pLS[i]->SetPoints(t, data[i], lineLen);
		pLS[i]->SetColor(LineRGB[i]);
	}
}

void NavChart::chartUpdate(double tk,  double* input)
{
	size_t i;
	for (i = 0; i < lineLen-1; i++)
	{
		t[i] = t[i + 1];
		for (size_t j = 0; j < LINEN; j++)
		{
			data[j][i] = data[j][i + 1];
			t[i] = t[i + 1];
		}
	}
	t[i] = tk;
	for (size_t j = 0; j < LINEN; j++)
	{
		data[j][i] = input[j];
		pLS[j]->SetPoints(t, data[j], lineLen);
	}
}

void NavChart::chartReinit()
{
	int lineLen = 20 * FS;
	for (int i = 0; i < lineLen; i++)
	{
		t[i] = -20 + i * 0.1;
		for (int l = 0; l < LINEN; l++)
			data[l][i] = 0;
	}
	for (size_t j = 0; j < LINEN; j++)
	{
		pLS[j]->SetPoints(t, data[j], lineLen);
	}
}

void NavChart::chartInit(TChartString chartTitle, int fs)
{
	title = chartTitle;
	FS = fs;
	lineLen = 2000;
	CChartAxis* pAxis = NULL;
	pAxis = CreateStandardAxis(CChartCtrl::BottomAxis);		// 坐标系初始化（不可缺少）
	pAxis->SetAutomatic(true);
	pAxis = CreateStandardAxis(CChartCtrl::LeftAxis);
	pAxis->SetAutomatic(true);

	GetTitle()->AddString(title);

	for (UINT  i = 0; i < lineLen; i++)
	{
		for (int l = 0; l < 2; l++)
			data[l][i] = 0;
	}
	RemoveAllSeries();	//先清空

	pLS[0] = CreateLineSerie();
	pLS[0]->SetPoints(data[0], data[1], lineLen);
	pLS[0]->SetColor(LineRGB[0]);
}

void NavChart::chartUpdate(double x, double y)
{
	size_t i;
	for (i = 0; i < lineLen - 1; i++)
	{
		data[0][i] = data[0][i + 1];
		data[1][i] = data[1][i + 1];
	}
	data[0][i] = x;
	data[1][i] = y;
	pLS[0]->SetPoints(data[0], data[1], lineLen);
}

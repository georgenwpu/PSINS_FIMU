#pragma once
#include "PSINSCore/PSINS.h"

// InputInitInfoDlg 对话框
#define NAV_INITVP	WM_USER+200

class InputInitInfoDlg : public CDialogEx
{
	DECLARE_DYNAMIC(InputInitInfoDlg)

public:
	InputInitInfoDlg(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~InputInitInfoDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_INPUT };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持
	virtual BOOL OnInitDialog();

	DECLARE_MESSAGE_MAP();

public:
	HWND MainDlg;
	double v0[3], p0[3];
	CVect3 vel0;		// initialization vectors
	CVect3 pos0;
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	CEdit editVe;
	CEdit editVn;
	CEdit editVu;
	CEdit editLat;
	CEdit editLng;
	CEdit editAlt;
};

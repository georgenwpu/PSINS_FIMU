// InputInitInfoDlg.cpp: 实现文件
//

#include "pch.h"
#include "PSINS_FIMU.h"
#include "InputInitInfoDlg.h"
#include "afxdialogex.h"

// InputInitInfoDlg 对话框

IMPLEMENT_DYNAMIC(InputInitInfoDlg, CDialogEx)

InputInitInfoDlg::InputInitInfoDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_DIALOG_INPUT, pParent)
{
	
}

InputInitInfoDlg::~InputInitInfoDlg()
{
}

BOOL InputInitInfoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	editVe.SetWindowText(L"0.0");
	editVn.SetWindowText(L"0.0");
	editVu.SetWindowText(L"0.0");
	editLat.SetWindowText(L"34.02856913972845");
	editLng.SetWindowText(L"108.76883280563354");
	editAlt.SetWindowText(L"400.0");

	return TRUE;
}

void InputInitInfoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_VE, editVe);
	DDX_Control(pDX, IDC_EDIT_VN, editVn);
	DDX_Control(pDX, IDC_EDIT_VU, editVu);
	DDX_Control(pDX, IDC_EDIT_LAT, editLat);
	DDX_Control(pDX, IDC_EDIT_LNG, editLng);
	DDX_Control(pDX, IDC_EDIT_ALT, editAlt);
}


BEGIN_MESSAGE_MAP(InputInitInfoDlg, CDialogEx)
	ON_BN_CLICKED(IDOK, &InputInitInfoDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &InputInitInfoDlg::OnBnClickedCancel)
END_MESSAGE_MAP()


// InputInitInfoDlg 消息处理程序


void InputInitInfoDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	MainDlg = ::GetParent(m_hWnd);
	CString vp;
	editVe.GetWindowText(vp);
	v0[0] = _ttof(vp);
	editVn.GetWindowText(vp);
	v0[1] = _ttof(vp);
	editVu.GetWindowText(vp);
	v0[2] = _ttof(vp);
	editLat.GetWindowText(vp);
	p0[0] = _ttof(vp);
	editLng.GetWindowText(vp);
	p0[1] = _ttof(vp);
	editAlt.GetWindowText(vp);
	p0[2] = _ttof(vp);
	::SendMessage(MainDlg, NAV_INITVP, (WPARAM)(v0 + 0), (LPARAM)(3));
	::SendMessage(MainDlg, NAV_INITVP, (WPARAM)(v0 + 1), (LPARAM)(4));
	::SendMessage(MainDlg, NAV_INITVP, (WPARAM)(v0 + 2), (LPARAM)(5));
	::SendMessage(MainDlg, NAV_INITVP, (WPARAM)(p0 + 0), (LPARAM)(6));
	::SendMessage(MainDlg, NAV_INITVP, (WPARAM)(p0 + 1), (LPARAM)(7));
	::SendMessage(MainDlg, NAV_INITVP, (WPARAM)(p0 + 2), (LPARAM)(8));
	::SendMessage(MainDlg, NAV_INITVP, NULL, (WPARAM)(10));
	CDialogEx::OnOK();
}


void InputInitInfoDlg::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
}

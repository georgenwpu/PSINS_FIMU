#if !defined(AFX_DIGISTATIC_H__F77484C2_745F_11D3_A718_87712333104C__INCLUDED_)
#define AFX_DIGISTATIC_H__F77484C2_745F_11D3_A718_87712333104C__INCLUDED_

#if _MSC_VER >= 1000
#pragma once
#endif // _MSC_VER >= 1000
// Digistatic.h : header file
//
// Copyright (C) 2000 by Michel Wassink
// All rights reserved
//
// This is free software.
// This code may be used in compiled form in any way you desire. This  
// file may be redistributed unmodified by any means PROVIDING it is   
// not sold for profit without the authors written consent, and   
// providing that this notice and the authors name and all copyright   
// notices remains intact. If the source code in this file is used in   
// any  commercial application then a statement along the lines of   
// "Portions Copyright � 2000 Michel Wassink" must be included in   
// the startup banner, "About" box or printed documentation. An email   
// letting me know that you are using it would be nice as well. That's   
// not much to ask considering the amount of work that went into this.  
//
// No warrantee of any kind, expressed or implied, is included with this
// software; use at your own risk, responsibility for damages (if any) to
// anyone resulting from the use of this software rests entirely with the
// user.
//
// Version: 1.0
// Release: 1 (februari 2000 to www.codeguru.com and www.codeproject.com)
// Release: 2 (not released yet)
// -----------------------------------------------------------------------
// Notes to changes for release 1 (V1.0):
//  -	First release.
// Notes to changes for version 1.1:
//  -	Use Windowtext as defaut text.
//  -   changed SetBackColor to SetBkColor
//  -   changed from stl to Afx templates
// Notes to changes for version 1.2
//  -   added option to draw immediately
//  -   Changed CDigiString to CDigiStatic
//  -   released at codeproject and codeguru
// Notes to changes for version 1.3
//  -   Added option to draw transparent, thank to Daniel DeGeest for his help
//  -   5 New characters added:   ()/\'
//  -   bleeding edge can be turned off
//  -   released at codeproject and codeguru(release 3)
// Notes to changes for future release 4: (V2.0)
//  -   nicer symmetrical rounding vertically.
//  -   second release with enhanced speed
// Send bug reports, bug fixes, enhancements, requests, flames, etc., and
// I'll try to keep a version up to date.  I can be reached as follows:
//    micways@hotmail.com				  (private site)
// An email letting me know that you are using it would be nice.
/////////////////////////////////////////////////////////////////////////////
#include <afxtempl.h>
#include "RGBCOLOR.H"

#pragma once

// CRect class with double precision for accurate drawing.
class CDoubleRect
{
public:
	void SetRect(double x1, double y1, double x2, double y2)
	{ left = x1; top = y1; right = x2; bottom = y2;}
	double Width() const{return right - left;}
	double Height() const{return bottom - top;}
	void SetRectEmpty(){left = top = right = bottom = 0.0;}
public:
	double left, top, right, bottom; 
};

class CDSegment
{
public:
	CDSegment();
	CDSegment(const CDSegment& Segment);
	~CDSegment();
	void DefPoints(const POINT* lpPoints, const BYTE* lpTypes, int nCount);
	void Draw(CDC *pDC,const CDoubleRect &DrawPlace, int iWidth) const;
	void FreeSegment();
	CDSegment operator=(const CDSegment &Segment);

// Implementation
public:
	CPoint	  *	m_paPoints;			// array with point
	BYTE	  *	m_paTypes;			// array with connecting info for point
	int			m_nCount;			// number of points
};

typedef CArray< CDSegment, CDSegment> DSegmentArray;

class CDigiChar
{
// Construction
public:
	CDigiChar();
	CDigiChar(const CDigiChar& DigiChar);

//Attributes:
public:
	virtual void SetElementData(WORD wSegmData, int iDispStyle);
	void	Draw(CDC *pDC, const CDoubleRect &DrawPlace, CPen *pOffPen, CPen *pOnPen,
				CBrush *pOffBrush, CBrush *pOnBrush, CRgn* region = NULL);
	void SetColor(COLORREF OffColor, COLORREF OnColor);
	int GetNormWidth() const;
	CDigiChar operator=(const CDigiChar &DigiChar);
	COLORREF GetColor(void) const { return m_OnColor; }

protected:
	int				m_Width;		// Width of character
	WORD			m_wSegmData;	// segments to highlight (binairy encoded)
	DSegmentArray	m_SegmentArray;	// Character is array of segments
	int				m_NSegments;	// Number of segments
	COLORREF		m_OffColor;		// Color of segment when off
	COLORREF		m_OnColor;		// Color of segment when on
	BOOL			m_bNoOff;		// Do not draw segments that are off

};

typedef CArray<CDigiChar, CDigiChar> DigiCharArray;

/////////////////////////////////////////////////////////////////////////////
// CDigiStatic class

class CDigiStatic : public CStatic
{
// Construction
public:
	CDigiStatic();

// Attributes
public:
	enum {
		DS_SMOOTH	=  1,	// Pioneer kind of characters
		DS_STYLE14	=  2,	// use allways 14 segment display.
		DS_SZ_PROP	=  4,	// size proportional(default)
		DS_NO_OFF	=  8,	// Don't draw the off segments
		DS_SOFT		= 16	// ambient bleeding to background
	};

// Operations
public:

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CDigiStatic)
	//}}AFX_VIRTUAL

// Implementation
public:
	virtual		~CDigiStatic();
	void		SetText(LPCTSTR lpszText);
	void		Format(LPCTSTR lpszFormat, ...);
	void		Format(UINT nFormatID, ...);
	void		SetColor(COLORREF OffColor, COLORREF OnColor);
	COLORREF	GetOnColor() const {return m_OnColor;}
	COLORREF	GetOffColor() const {return m_OffColor;}
	void		SetDrawImmediately(BOOL Enable = TRUE);
	COLORREF	SetBkColor(COLORREF BackColor = BLACK);
	void		SetTransparent(BOOL bSet = TRUE);
	BOOL		ModifyDigiStyle(DWORD dwRemove, DWORD dwAdd);

	// Generated message map functions
protected:
	CDigiChar * DefineChar(TCHAR cChar);
	void		BuildString();

	CString			m_strText;			// Current text
	BOOL			m_Modified;			// text is dirty
	DigiCharArray	m_CharArray;		// digistatic is an array of characters
	COLORREF		m_OffColor;			// Color of segment when off
	COLORREF		m_OnColor;			// Color of segment when on
	COLORREF		m_BackColor;		// Background color
	DWORD			m_DispStyle;		// DS_... may be bitwise OR-red
	BOOL			m_bImmediateUpdate;	// Enable/Disable immediate repaint
	BOOL			m_bTransparent;		// Enable/Disable transparent background

	//{{AFX_MSG(CDigiStatic)
	afx_msg void OnPaint();
	afx_msg BOOL OnEraseBkgnd(CDC* pDC);
	//}}AFX_MSG

	DECLARE_MESSAGE_MAP()

	void DoInvalidate();
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Developer Studio will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_DIGISTATIC_H__F77484C2_745F_11D3_A718_87712333104C__INCLUDED_)

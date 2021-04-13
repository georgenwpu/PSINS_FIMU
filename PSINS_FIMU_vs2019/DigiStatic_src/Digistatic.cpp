// Digistatic.cpp : implementation file
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
// "Portions Copyright2002 Michel Wassink" must be included in   
// the startup banner, "About" box or printed documentation. An email   
// letting me know that you are using it would be nice as well. That's   
// not much to ask considering the amount of work that went into this.  
//  
// No warrantee of any kind, expressed or implied, is included with this
// software; use at your own risk, responsibility for damages (if any) to
// anyone resulting from the use of this software rests entirely with the
// user.
//
// Send bug reports, bug fixes, enhancements, requests, flames, etc., and
// I'll try to keep a version up to date.  I can be reached as follows:
//    micways@hotmail.com				  (private site)
// An email letting me know that you are using it would be nice.
/////////////////////////////////////////////////////////////////////////////

#include "pch.h"
#include "MemDCEx.h"
#include "Digistatic.h"
#include "Curvefit.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

// Segment numbering:
// -----          13		 -----          0 
//|\ | /|      8  0  12		|     |      1     2  
//| \|/ |        1 2		|     |            
// -- --   ==    6 7	     -----   ==     3     
//| /|\ |        3 4	    |     |               
//|/ | \|      9  5  11		|     |      4     5  
// -----          10	     -----          6    

#define MAXSEGCHAR7		12		// Number of supported 7 segment characters
#define MAXSEGCHAR14	45		// Number of supported 14 segment characters
#define MAXSEGSEMCOL	 2		// Number of supported 3 segment characters
#define NORM_DIGIHEIGHT	83		// All characters must have this height

// please uncomment the following line if you want nicer scrolling 
// and all characters get same width
//#define _WIDE_SEMICOLON	

////////////////////////////////////////////////////////////////////////////////
// For 14 segments display...
//									   SP		0		1		2		3		4		5		6	
WORD CHAR_SEGM14[MAXSEGCHAR14]  = {0x0000, 0x3F00, 0x1800, 0x36C0, 0x3CC0, 0x19C0, 0x2DC0, 0x2FC0, 
//		7		8		9		-		A		B		C		D		E		F		G		H
   0x3800, 0x3FC0, 0x3DC0, 0x00C0, 0x3BC0, 0x3CA1, 0x2700, 0x3C21, 0x27C0, 0x23C0, 0x2F80, 0x1BC0,
//		I		J		K		L		M		N		O		P		Q		R		S		T
   0x2421, 0x1E00, 0x0354, 0x0700, 0x1B06, 0x1B12, 0x3F00, 0x33C0, 0x3F10, 0x33D0, 0x2DC0, 0x2021,
//		U		V		W		X		Y		Z		*		+		(		)	Back/		/
   0x1F00, 0x030C, 0x1B18, 0x001E, 0x11E0, 0x240C, 0x00FF, 0x00E1, 0x0014, 0x000A, 0x0012, 0x000C,
//		'
   0x0001 };
// straight style
CPoint PtSeg14N0[5]	 = {CPoint(20,13), CPoint(20,36), CPoint(24,40), CPoint(28,36), CPoint(28,13)};
CPoint PtSeg14N1[4]	 = {CPoint( 5, 5), CPoint(15,35), CPoint(20,37), CPoint(18,25)};
CPoint PtSeg14N6[6]	 = {CPoint( 6,41), CPoint(14,45), CPoint(18,45), CPoint(22,41), CPoint(18,37), 
						CPoint(14,37)};
CPoint PtSeg14N8[4]	 = {CPoint( 4, 7), CPoint( 4,40), CPoint(11,36), CPoint(11,26)};
CPoint PtSeg14N13[4] = {CPoint( 6, 4), CPoint(11,11), CPoint(37,11), CPoint(42, 4)};
BYTE   TpSeg14N0[5]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO,		PT_LINETO};
BYTE   TpSeg14N1[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};
BYTE   TpSeg14N6[6]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO,		PT_LINETO, 
						PT_LINETO};
BYTE   TpSeg14N8[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};
BYTE   TpSeg14N13[4] = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};

// smooth style PT_BEZIERTO
CPoint PtSeg14N0S[13] ={CPoint(20,12), CPoint(20,25), CPoint(22,36), CPoint(23,39), CPoint(24,40), 
						CPoint(25,39), CPoint(26,36), CPoint(28,25), CPoint(28,12), CPoint(26,10),
						CPoint(24, 9), CPoint(22,10), CPoint(20,12)};
CPoint PtSeg14N1S[10] ={CPoint(10,10), CPoint(10,13), CPoint(11,20), CPoint(13,28), CPoint(21,38), 
						CPoint(21,37), CPoint(19,26), CPoint(15,16), CPoint(11,10), CPoint(10,10)};
CPoint PtSeg14N6S[6] = {CPoint( 8,41), CPoint(12,45), CPoint(16,45), CPoint(23,41), CPoint(16,37),
						CPoint(12,37)};
CPoint PtSeg14N8S[10]= {CPoint( 4, 7), CPoint( 4,39), CPoint( 5,40), CPoint( 6,40), CPoint( 9,37),
						CPoint(11,33), CPoint(11,25), CPoint( 9,14), CPoint( 5, 6), CPoint( 4, 7)};
CPoint PtSeg14N13S[17]={CPoint( 8, 4), CPoint( 7, 5), CPoint( 7, 6), CPoint( 9, 8), CPoint(12, 9),
						CPoint(14,11), CPoint(19,11), CPoint(21, 9), CPoint(24, 7), CPoint(27, 9),
						CPoint(29,11), CPoint(34,11), CPoint(36, 9), CPoint(39, 8), CPoint(41, 6),
						CPoint(41, 5), CPoint(40, 4)};
BYTE   TpSeg14N0S[13]= {PT_MOVETO,	   PT_LINETO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_BEZIERTO,
						PT_BEZIERTO,   PT_BEZIERTO,	  PT_LINETO,	 PT_LINETO,		PT_BEZIERTO,
						PT_BEZIERTO,   PT_BEZIERTO,	  PT_LINETO};
BYTE   TpSeg14N1S[10] ={PT_MOVETO,	   PT_BEZIERTO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_LINETO,
						PT_LINETO,	   PT_BEZIERTO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_LINETO};
BYTE   TpSeg14N6S[6] = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO,		PT_LINETO, 
						PT_LINETO};
BYTE   TpSeg14N8S[10] ={PT_MOVETO,	   PT_LINETO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_BEZIERTO,
						PT_BEZIERTO,   PT_LINETO,	  PT_BEZIERTO,   PT_BEZIERTO,   PT_LINETO};
BYTE   TpSeg14N13S[17]={PT_MOVETO,	   PT_BEZIERTO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_BEZIERTO, 
						PT_LINETO,	   PT_LINETO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_BEZIERTO,
						PT_LINETO,	   PT_LINETO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_BEZIERTO,
						PT_BEZIERTO,   PT_LINETO};
///////////////////////////////////////////////////////////////////////////////
// For 7 segments display...
//									 SP		0	  1		2	  3		4	  5
BYTE CHAR_SEGM7[MAXSEGCHAR7]    = {0x00, 0x77, 0x24, 0x5D, 0x6D, 0x2E, 0x6B, 
// 	  6		7	  8		9	  -
   0x7B, 0x25, 0x7F, 0x6F, 0x08};
// straight style
CPoint PtSeg7N0[4]	 = {CPoint( 5, 4), CPoint(12,11), CPoint(36,11), CPoint(43, 4)};
CPoint PtSeg7N1[4]	 = {CPoint( 4, 6), CPoint( 4,40), CPoint(11,36), CPoint(11,13)};
CPoint PtSeg7N3[6]	 = {CPoint( 6,41), CPoint(14,45), CPoint(34,45), CPoint(42,41), CPoint(34,37),
						CPoint(14,37)}; // 3
BYTE   TpSeg7N0[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};
BYTE   TpSeg7N1[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};
BYTE   TpSeg7N3[6]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO,		PT_LINETO,
						PT_LINETO};

// smooth style PT_BEZIERTO
CPoint PtSeg7N0S[7]	 = {CPoint( 6, 4), CPoint( 5, 5), CPoint( 5, 6), CPoint( 8, 9), CPoint(12,11),
						CPoint(36,11), CPoint(39, 4)};
CPoint PtSeg7N1S[7]	 = {CPoint( 4, 9), CPoint( 4,39), CPoint( 6,40), CPoint( 7,40), CPoint( 9,38),
						CPoint(11,36), CPoint(11,12)};
CPoint PtSeg7N2S[10] = {CPoint(37,36), CPoint(39,38), CPoint(41,40), CPoint(42,40), CPoint(44,39),
						CPoint(44, 6), CPoint(42, 4), CPoint(41, 4), CPoint(39, 8), CPoint(37,12)};
CPoint PtSeg7N3S[6]	 = {CPoint( 8,41), CPoint(12,45), CPoint(36,45), CPoint(40,41), CPoint(36,37),
						CPoint(12,37)};
BYTE   TpSeg7N0S[7]	 = {PT_MOVETO,	   PT_BEZIERTO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_LINETO,
						PT_LINETO,	   PT_LINETO};
BYTE   TpSeg7N1S[7]	 = {PT_MOVETO,	   PT_LINETO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_BEZIERTO,
						PT_LINETO,	   PT_LINETO};
BYTE   TpSeg7N2S[10] = {PT_MOVETO,	   PT_BEZIERTO,	  PT_BEZIERTO,	 PT_LINETO,		PT_LINETO,
						PT_LINETO,	   PT_BEZIERTO,	  PT_BEZIERTO,	 PT_BEZIERTO,	PT_LINETO};
BYTE   TpSeg7N3S[6]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO,		PT_LINETO,
						PT_LINETO};

// For 3 segments semicoloncombi display...
//									  .		:
BYTE CHAR_SEMCOL[MAXSEGSEMCOL]  = {0x04, 0x03};

#ifdef _WIDE_SEMICOLON
	// for wide semicolon character space
	CPoint PtSegScN0[4]	 = {CPoint( 19,23), CPoint( 19,32), CPoint(32,32), CPoint(32,23)};
	CPoint PtSegScN1[4]	 = {CPoint( 19,50), CPoint( 19,59), CPoint(32,59), CPoint(32,50)};
	CPoint PtSegScN2[4]	 = {CPoint( 19,68), CPoint( 19,77), CPoint(32,77), CPoint(32,68)};
#else
	// small semicolon character space
	CPoint PtSegScN0[4]	 = {CPoint( 4,23), CPoint( 4,32), CPoint(13,32), CPoint(13,23)};
	CPoint PtSegScN1[4]	 = {CPoint( 4,50), CPoint( 4,59), CPoint(13,59), CPoint(13,50)};
	CPoint PtSegScN2[4]	 = {CPoint( 4,68), CPoint( 4,77), CPoint(13,77), CPoint(13,68)};
#endif
BYTE   TpSegScN0[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};
BYTE   TpSegScN1[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};
BYTE   TpSegScN2[4]	 = {PT_MOVETO,	   PT_LINETO,	  PT_LINETO,	 PT_LINETO};


// Functions for mirroring points...
CPoint PointMirror(const CPoint &P, const CPoint &M)
{
	return CPoint(P.x + 2*(M.x - P.x), P.y + 2*(M.y - P.y));
}

CPoint LineMirrorX(const CPoint &P, int X)
{
	return CPoint(P.x + 2*(X - P.x), P.y);
}

CPoint LineMirrorY(const CPoint &P, int Y)
{
	return CPoint(P.x, P.y + 2*(Y - P.y));
}

// character classes.
class CDigiColonDotChar : public CDigiChar
{
public:
	CDigiColonDotChar();
	void SetElementData(WORD wSegmData, int iDispStyle);
};

class CDigi7Segment : public CDigiChar
{
public:
	CDigi7Segment();
	void SetElementData(WORD wSegmData, int iDispStyle);
};

class CDigi14Segment : public CDigiChar
{
public:
	CDigi14Segment();
	void SetElementData(WORD wSegmData, int iDispStyle);
};

/////////////////////////////////////////////////////////////////////////////
// CDSegment
CDSegment::CDSegment()
{
	m_paPoints	= NULL;
	m_paTypes	= NULL;
}

CDSegment::CDSegment(const CDSegment& Segment)
{
	ASSERT(Segment.m_paPoints != NULL); // Do not copy an unitialized segment

	m_nCount = Segment.m_nCount;

	m_paPoints = new CPoint[m_nCount];
	m_paTypes = new BYTE[m_nCount];
	if (m_paPoints != NULL && m_paTypes != NULL)
	{
		memcpy(m_paPoints, Segment.m_paPoints, m_nCount * sizeof(CPoint));
		memcpy(m_paTypes, Segment.m_paTypes, m_nCount * sizeof(BYTE));
	}
}

// -----------------------------------------------------------------------------

CDSegment::~CDSegment()
{
	FreeSegment();
}

void CDSegment::DefPoints(const POINT* lpaPoints, const BYTE* lpaTypes, int nCount)
{
	FreeSegment();

	m_paPoints = new CPoint[nCount];
	m_paTypes = new BYTE[nCount];
	m_nCount = nCount;
	if (m_paPoints != NULL && m_paTypes != NULL)
	{
		memcpy(m_paPoints, lpaPoints, m_nCount * sizeof(CPoint));
		memcpy(m_paTypes, lpaTypes, m_nCount * sizeof(BYTE));
	}
}

// -----------------------------------------------------------------------------

CDSegment CDSegment::operator=(const CDSegment &Segment)
{
	CPoint *pNewPoints;
	BYTE * pNewTypes;

	m_nCount = Segment.m_nCount;

	pNewPoints = new CPoint[m_nCount];
	pNewTypes = new BYTE[m_nCount];
	memcpy(pNewPoints, Segment.m_paPoints, m_nCount * sizeof(CPoint));
	memcpy(pNewTypes, Segment.m_paTypes, m_nCount * sizeof(BYTE));
	FreeSegment();			// Get rid of old stuff
	m_paPoints = pNewPoints;
	m_paTypes = pNewTypes;

	return *this;
}

void CDSegment::FreeSegment()
{
	delete[] m_paPoints;
	delete[] m_paTypes;
}

void CDSegment::Draw(CDC *pDC, const CDoubleRect &DrawPlace, int iWidth) const
{
	int i, nBez,b;
	CPoint * paPoints;
	double daContr[4];
	double *pBezPts;
	double dRelWidth, dRelHeight;

	paPoints = new CPoint[m_nCount];
	if (paPoints == NULL) return;

	dRelWidth = DrawPlace.Width() / iWidth;
	dRelHeight = DrawPlace.Height() / NORM_DIGIHEIGHT;
	for (i = 0; i < m_nCount; i++)
	{
		if (m_paTypes[i] != PT_BEZIERTO)
		{
			paPoints[i] = CPoint(int(DrawPlace.left + dRelWidth	 * m_paPoints[i].x + 0.5),
								 int(DrawPlace.top  + dRelHeight * m_paPoints[i].y + 0.5));
		}
	}

	for (i = 0; i < m_nCount; i++)
	{
		if (m_paTypes[i] == PT_MOVETO)
		{
			pDC->MoveTo(paPoints[i]);
		}
		else if (m_paTypes[i] == PT_LINETO)
		{
			VERIFY(pDC->LineTo(paPoints[i]));
		}
		else if (m_paTypes[i] == PT_BEZIERTO)
		{
			// Look for the first non-bezier point(This is the EndPoint)...
			nBez = 0;
			do
			{
				nBez++;
			} while (m_paTypes[i+nBez] == PT_BEZIERTO);

			pBezPts = new double[2*(nBez+2)];
			for (b = 0; b < (nBez+2)*2; b += 2)
			{
				pBezPts[b  ] = DrawPlace.left + dRelWidth	* m_paPoints[i-1+b/2].x; 
				pBezPts[b+1] = DrawPlace.top  + dRelHeight	* m_paPoints[i-1+b/2].y;
			}
			CalcBezier(pBezPts, 2*(nBez+2), daContr);
			delete[] pBezPts;

			paPoints[i  ].x	= int(daContr[0] + 0.5);
			paPoints[i  ].y	= int(daContr[1] + 0.5);
			paPoints[i+1].x	= int(daContr[2] + 0.5);
			paPoints[i+1].y	= int(daContr[3] + 0.5);
			paPoints[i+2]	= paPoints[i+nBez];

			VERIFY(pDC->PolyBezierTo(&paPoints[i], 3));
			i += nBez;
		}
	} // for

	delete[] paPoints;
}

/////////////////////////////////////////////////////////////////////////////
// CDigiChar

CDigiChar::CDigiChar()
{
	m_Width		= 49;
	m_wSegmData = 0x0000; // All segments in offcolor
	m_bNoOff = FALSE;
}

CDigiChar::CDigiChar(const CDigiChar& DigiChar)
{
	m_NSegments		  = DigiChar.m_NSegments;
	m_OffColor		  = DigiChar.m_OffColor;
	m_OnColor		  = DigiChar.m_OnColor;
	m_SegmentArray.Copy(DigiChar.m_SegmentArray);
	m_Width			  = DigiChar.m_Width;
	m_wSegmData		  = DigiChar.m_wSegmData;
	m_bNoOff	      = DigiChar.m_bNoOff;
}

void CDigiChar::Draw(CDC *pDC, const CDoubleRect &DrawPlace, CPen *pOffPen, CPen *pOnPen,
					 CBrush *pOffBrush, CBrush *pOnBrush, CRgn* region) 
{
	WORD SegMask;
	int iSeg;

	SegMask = 1;
	for (iSeg = 0; iSeg < m_SegmentArray.GetSize(); iSeg++)
	{
		if (m_wSegmData & SegMask)
		{
			pDC->SelectObject(pOnBrush);
			pDC->SelectObject(pOnPen);
		}
		else
		{
			pDC->SelectObject(pOffBrush);
			pDC->SelectObject(pOffPen);
		}

		if (!m_bNoOff || (m_wSegmData & SegMask))
		{
			pDC->BeginPath();
			m_SegmentArray[iSeg].Draw(pDC, DrawPlace, m_Width);
			pDC->EndPath();
			pDC->StrokeAndFillPath();
			if (region != NULL)
			{
				if (region->m_hObject == NULL)
					region->CreateFromPath(pDC);
				else
				{
					CRgn segmentRgn;
					segmentRgn.CreateFromPath(pDC);
					region->CombineRgn(region,&segmentRgn,RGN_OR);
				}
			}
		}
		SegMask <<= 1;
	}
}

void CDigiChar::SetElementData(WORD wSegmData, int iDispStyle)
{
	m_wSegmData = wSegmData;
	m_bNoOff = (iDispStyle & CDigiStatic::DS_NO_OFF) > 0;
}

int CDigiChar::GetNormWidth() const
{
	return m_Width;
}

// -----------------------------------------------------------------------------

CDigiChar CDigiChar::operator=(const CDigiChar &DigiChar)
{
	m_NSegments		  = DigiChar.m_NSegments;
	m_OffColor		  = DigiChar.m_OffColor;
	m_OnColor		  = DigiChar.m_OnColor;
	m_SegmentArray.Copy(DigiChar.m_SegmentArray);
	m_Width			  = DigiChar.m_Width;
	m_wSegmData		  = DigiChar.m_wSegmData;
	m_bNoOff		  = DigiChar.m_bNoOff;
	return *this;
}

/////////////////////////////////////////////////////////////////////////////
// CDigi7Segment

CDigi7Segment::CDigi7Segment()
{
	m_Width		= 49;
	m_NSegments = 7;
}

void CDigi7Segment::SetElementData(WORD wSegmData, int iDispStyle)
{
	int i, p;
	CDSegment TmpSegm;
	LPPOINT lpTmpSegPoints = NULL;
	LPPOINT lpSegPoints	= NULL;
	LPBYTE lpType		= NULL;
	int nCount	=0;

	m_SegmentArray.RemoveAll();
	for (i = 0; i < m_NSegments; i++)
	{
		// Find data for segment in correct style...
		switch(i)
		{
			case 0:
			case 6: if (iDispStyle & CDigiStatic::DS_SMOOTH)
					{
						lpSegPoints = PtSeg7N0S;
						lpType = TpSeg7N0S; 
						nCount = 7; 
					}
					else
					{
						lpSegPoints = PtSeg7N0;
						lpType = TpSeg7N0; 
						nCount = 4; 
					}
					break;
			case 1:
			case 4:if (iDispStyle & CDigiStatic::DS_SMOOTH)
					{
						lpSegPoints = PtSeg7N1S;
						lpType = TpSeg7N1S; 
						nCount = 7; 
					}
					else
					{
						lpSegPoints = PtSeg7N1;
						lpType = TpSeg7N1; 
						nCount = 4; 
					}
					break;
			case 2:
			case 5: if (iDispStyle & CDigiStatic::DS_SMOOTH)
					{
						lpSegPoints = PtSeg7N2S;
						lpType = TpSeg7N2S; 
						nCount = 10; 
					}
					else
					{
						lpSegPoints = PtSeg7N1;
						lpType = TpSeg7N1; 
						nCount = 4; 
					}
					break;
			case 3: if (iDispStyle & CDigiStatic::DS_SMOOTH)
					{
						lpSegPoints = PtSeg7N3S;
						lpType = TpSeg7N3S; 
						nCount = 6; 
					}
					else
					{
						lpSegPoints = PtSeg7N3;
						lpType = TpSeg7N3; 
						nCount = 6; 
					}
					break;
		}
		// For nondefined segments use mirroring...
		switch(i)
		{
		case 6: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);break;
		case 2: if (!(iDispStyle & CDigiStatic::DS_SMOOTH))
				{
					lpTmpSegPoints = new CPoint[nCount]; 
					for (p = 0; p < nCount; p++)
						lpTmpSegPoints[p] = LineMirrorX(lpSegPoints[p], (m_Width-1)/2);
				}
				break;
		case 4: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);break;
		case 5: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				if (iDispStyle & CDigiStatic::DS_SMOOTH)
					lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);
				else
					lpTmpSegPoints[p] = PointMirror(lpSegPoints[p], CPoint((m_Width-1)/2, 41));
				break;
		}

		// Copy data to segment array...
		if (lpTmpSegPoints == NULL)
			// point is an original.
			TmpSegm.DefPoints(lpSegPoints, lpType, nCount);
		else
		{
			// point is mirrored.
			TmpSegm.DefPoints(lpTmpSegPoints, lpType, nCount);
			delete[] lpTmpSegPoints;
			lpTmpSegPoints = NULL;
		}
		m_SegmentArray.Add(TmpSegm);
	}
	CDigiChar::SetElementData(wSegmData, iDispStyle);
}

/////////////////////////////////////////////////////////////////////////////
// CDigi14Segment

CDigi14Segment::CDigi14Segment()
{
	m_Width		= 49;
	m_NSegments = 14;
}

void CDigi14Segment::SetElementData(WORD wSegmData, int iDispStyle)
{
	int i, p;
	CDSegment TmpSegm;
	LPPOINT lpTmpSegPoints = NULL;
	LPPOINT lpSegPoints		= NULL;
	LPBYTE lpType			= NULL;
	int nCount				= 0;

	m_SegmentArray.RemoveAll();
	for (i = 0; i < m_NSegments; i++)
	{
		// Find data for segment in correct style...
		switch(i)
		{
			case 0:
			case 5:	if (iDispStyle & CDigiStatic::DS_SMOOTH)
					{
						lpSegPoints = PtSeg14N0S;
						lpType = TpSeg14N0S; 
						nCount = 13;
					}
					else
					{
						lpSegPoints = PtSeg14N0;
						lpType = TpSeg14N0; 
						nCount = 5;
					}
				break;
			case 1:
			case 2:
			case 3:
			case 4:
				if (iDispStyle & CDigiStatic::DS_SMOOTH)
				{
					lpSegPoints = PtSeg14N1S;
					lpType = TpSeg14N1S; 
					nCount = 10;
				}
				else
				{
					lpSegPoints = PtSeg14N1;
					lpType = TpSeg14N1; 
					nCount = 5;
				}
				break;
			case 6:
			case 7:
				if (iDispStyle & CDigiStatic::DS_SMOOTH)
				{
					lpSegPoints = PtSeg14N6S;
					lpType = TpSeg14N6S; 
					nCount = 6;
				}
				else
				{
					lpSegPoints = PtSeg14N6;
					lpType = TpSeg14N6; 
					nCount = 6;
				}
				break;
			case 8:
			case 9:
			case 11:
			case 12:
				if (iDispStyle & CDigiStatic::DS_SMOOTH)
				{
					lpSegPoints = PtSeg14N8S;
					lpType = TpSeg14N8S; 
					nCount = 10;
				}
				else
				{
					lpSegPoints = PtSeg14N8;
					lpType = TpSeg14N8; 
					nCount = 4;
				}
				break;
			case 13:
			case 10:
				if (iDispStyle & CDigiStatic::DS_SMOOTH)
				{
					lpSegPoints = PtSeg14N13S;
					lpType = TpSeg14N13S; 
					nCount = 17;
				}
				else
				{
					lpSegPoints = PtSeg14N13;
					lpType = TpSeg14N13; 
					nCount = 4;
				}
				break;
		}
		// For nondefined segments use mirroring...
		switch(i)
		{
		case 5: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);break;
		case 2: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorX(lpSegPoints[p], (m_Width-1)/2);break;
		case 3: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);break;
		case 4: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = PointMirror(lpSegPoints[p], CPoint((m_Width-1)/2, 41));break;
		case 7: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorX(lpSegPoints[p], (m_Width-1)/2);break;
		case 9: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);break;
		case 11: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = PointMirror(lpSegPoints[p], CPoint((m_Width-1)/2, 41));break;
		case 12: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorX(lpSegPoints[p], (m_Width-1)/2);break;
		case 10: lpTmpSegPoints = new CPoint[nCount]; 
			for (p = 0; p < nCount; p++)
				lpTmpSegPoints[p] = LineMirrorY(lpSegPoints[p], 41);break;
		}

		// Copy data to segment array...
		if (lpTmpSegPoints == NULL)
			// point is an original.
			TmpSegm.DefPoints(lpSegPoints, lpType, nCount);
		else
		{
			// point is mirrored.
			TmpSegm.DefPoints(lpTmpSegPoints, lpType, nCount);
			delete[] lpTmpSegPoints;
			lpTmpSegPoints = NULL;
		}
		m_SegmentArray.Add(TmpSegm);
	}
	CDigiChar::SetElementData(wSegmData, iDispStyle);
}

/////////////////////////////////////////////////////////////////////////////
// CDigiSemiColonCombi

CDigiColonDotChar::CDigiColonDotChar()
{
#ifdef _WIDE_SEMICOLON
	m_Width		= 49;
#else
	m_Width		= 18;
#endif
	m_NSegments = 3;
}

void CDigiColonDotChar::SetElementData(WORD wSegmData, int iDispStyle)
{
	int i;
	CDSegment DSegment;
	LPPOINT lpSegPoints	= NULL;
	LPBYTE lpType	= NULL;
	int nCount = 0; 

	m_SegmentArray.RemoveAll();

	for (i = 0; i < m_NSegments; i++)
	{
		// Find data for segment in correct style...
		switch(i)
		{
			case 0:lpSegPoints = PtSegScN0;
				lpType = TpSegScN0; 
				nCount = 4; break;
			case 1:
				lpSegPoints = PtSegScN1;
				lpType = TpSegScN1; 
				nCount = 4; break;
			case 2:
				lpSegPoints = PtSegScN2;
				lpType = TpSegScN2; 
				nCount = 4; break;
		}
		// Copy data to segment array...
		DSegment.DefPoints(lpSegPoints, lpType, nCount);
		m_SegmentArray.Add(DSegment);
	}

	CDigiChar::SetElementData(wSegmData, iDispStyle);
}

/////////////////////////////////////////////////////////////////////////////
// CDigiStatic

CDigiStatic::CDigiStatic()
	: m_strText(_T("?"))
{
	m_DispStyle			= DS_SZ_PROP | DS_SOFT;
	m_BackColor			= BLACK;
	m_Modified			= TRUE;
	m_OffColor			= DARKGREEN;
	m_OnColor			= LIGHTGREEN;
	m_bImmediateUpdate	= FALSE;
	m_bTransparent		= FALSE;
}

CDigiStatic::~CDigiStatic()
{
	m_CharArray.RemoveAll();
}

void CDigiStatic::SetText(LPCTSTR lpszText)
{
	if (m_strText != lpszText)
	{
		m_strText = lpszText;
		m_Modified = TRUE;

		DoInvalidate();
	}
}

void CDigiStatic::Format(LPCTSTR lpszFormat, ...)
{
	CString str;

	// format and write the data we were given
	va_list argList;
	va_start(argList, lpszFormat);
	str.FormatV(lpszFormat, argList);
	va_end(argList);

	SetText(str);
}

void CDigiStatic::Format(UINT nFormatID, ...)
{
	CString str;
	CString strFormat;
	VERIFY(strFormat.LoadString(nFormatID) != 0);

	va_list argList;
	va_start(argList, nFormatID);
	str.FormatV(strFormat, argList);
	va_end(argList);

	SetText(str);
}


void CDigiStatic::SetColor(COLORREF OffColor, COLORREF OnColor)
{
	if (m_OnColor == OnColor && m_OffColor == OffColor)
		return;
	m_OnColor = OnColor;
	m_OffColor = OffColor;
	DoInvalidate();
}

COLORREF CDigiStatic::SetBkColor(COLORREF BackColor /* = BLACK */)
{
	COLORREF PrevBkColor;
	if (m_BackColor == BackColor)
		return m_BackColor;
	PrevBkColor = m_BackColor;
	m_BackColor = BackColor;
	DoInvalidate();

	return PrevBkColor;
}

void CDigiStatic::SetTransparent(BOOL bSet /* = TRUE */)
{
	m_bTransparent = bSet;
	if (m_bTransparent)
		ModifyStyleEx(0,WS_EX_TRANSPARENT); // Add transparency
	else
		ModifyStyleEx(WS_EX_TRANSPARENT, 0); // Remove transparency
}


void CDigiStatic::SetDrawImmediately(BOOL Enable /* = TRUE*/)
{
	m_bImmediateUpdate = Enable;
}

BOOL CDigiStatic::ModifyDigiStyle(DWORD dwRemove, DWORD dwAdd)
{
	ASSERT(!(dwRemove & dwAdd));
	if (dwRemove & dwAdd)
		return FALSE;

	m_DispStyle |= dwAdd;
	m_DispStyle &= ~dwRemove;

	m_Modified = TRUE;
	DoInvalidate();

	return TRUE;
}

CDigiChar * CDigiStatic::DefineChar(TCHAR cChar)
{
	int iIndex;
	CDigiChar * pDChar = NULL;

	if (cChar >= _T('0') && cChar <= _T('9') 
		|| cChar == _T(' ') || cChar == _T('-'))
	{
		// these characters can be 7 or 14 segments...
		if (cChar == _T(' '))
		{
			iIndex = 0;
		}
		else if (cChar == _T('-'))
		{
			iIndex = 11;
		}
		else
		{
			iIndex = cChar - _T('0') + 1;
		}

		if (m_DispStyle & DS_STYLE14)
		{
			pDChar = new CDigi14Segment;
			pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
		}
		else
		{
			pDChar = new CDigi7Segment;
			pDChar->SetElementData(CHAR_SEGM7[iIndex], m_DispStyle);
		}
	}
	else if (cChar >= _T('A') && cChar <= _T('Z'))
	{
		// 14 segment only...
		iIndex = cChar - _T('A') + 12;
			pDChar = new CDigi14Segment;
			pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
	}
	else
	{
		// Spcial characters...
		iIndex = 0;
		switch(cChar)
		{
			case _T(':'): iIndex++; 
			case _T('.'): pDChar = new CDigiColonDotChar; 
					pDChar->SetElementData(CHAR_SEMCOL[iIndex], m_DispStyle);
					break;
			case _T('*'): iIndex = MAXSEGCHAR14 - 7; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			case _T('+'): iIndex = MAXSEGCHAR14 - 6; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			case _T('('): iIndex = MAXSEGCHAR14 - 5; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			case _T(')'): iIndex = MAXSEGCHAR14 - 4; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			case _T('\\'): iIndex = MAXSEGCHAR14 - 3; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			case _T('/'): iIndex = MAXSEGCHAR14 - 2; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			case _T('\''): iIndex = MAXSEGCHAR14 - 1; pDChar = new CDigi14Segment; 
					pDChar->SetElementData(CHAR_SEGM14[iIndex], m_DispStyle);
				break;
			//default : ASSERT(FALSE);
		}
	}

	return pDChar;
}

void CDigiStatic::BuildString()
{
	CDigiChar * pDChar;
	if (!m_Modified) return;
	m_CharArray.RemoveAll();
	
	if (m_strText == _T("?"))
		GetWindowText(m_strText);
	m_strText.MakeUpper();
	for (int i = 0; i < m_strText.GetLength(); i++)
	{
		if ((pDChar = DefineChar(m_strText[i])) != NULL)
		{
			m_CharArray.Add(*pDChar);
			delete pDChar;
		}
	}

	m_Modified = FALSE;
}

BEGIN_MESSAGE_MAP(CDigiStatic, CStatic)
	//{{AFX_MSG_MAP(CDigiStatic)
	ON_WM_PAINT()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CDigiStatic message handlers

void CDigiStatic::OnPaint() 
{
	CRect rect;
	CDoubleRect CharRect;
	int r, g, b;
	GetClientRect(&rect);
	COLORREF	PenColor;

	CPaintDC dc(this); // device context for painting
	dc.SetBkColor(m_BackColor);
	CMemDCEx MemDC(&dc, &rect, m_bTransparent);

	CBrush hBrushOff, hBrushOn;
	hBrushOff.CreateSolidBrush(m_OffColor);
	hBrushOn.CreateSolidBrush(m_OnColor);
	CBrush *pOldBrush = MemDC.SelectObject(&hBrushOn);

	if (m_DispStyle & DS_SOFT)
	{
		r = int(GetRValue(m_OffColor) * 0.75 + GetRValue(m_BackColor) * 0.25);
		g = int(GetGValue(m_OffColor) * 0.75 + GetGValue(m_BackColor) * 0.25);
		b = int(GetBValue(m_OffColor) * 0.75 + GetBValue(m_BackColor) * 0.25);
		PenColor = RGB(r,g,b);
	}
	else
		PenColor = m_OffColor;
	CPen OffPen(PS_SOLID | PS_ENDCAP_ROUND, 1, PenColor);

	if (m_DispStyle & DS_SOFT)
	{
		r = int(GetRValue(m_OnColor) * 0.75 + GetRValue(m_BackColor) * 0.25);
		g = int(GetGValue(m_OnColor) * 0.75 + GetGValue(m_BackColor) * 0.25);
		b = int(GetBValue(m_OnColor) * 0.75 + GetBValue(m_BackColor) * 0.25);
		PenColor = RGB(r,g,b);
	}
	else
		PenColor = m_OnColor;

	CPen OnPen(PS_SOLID | PS_ENDCAP_ROUND, 1, PenColor);
	CPen *pOldPen = MemDC.SelectObject(&OffPen);

	int iTotWidth = 0;
	double dRelWidth, dRelHeight;

	// Calculate resizing factors...
	BuildString();
	for (int iChar = 0; iChar < m_CharArray.GetSize(); iChar++)
	{
		iTotWidth += m_CharArray[iChar].GetNormWidth();
	}
	dRelWidth = double(rect.Width()) / iTotWidth;
	dRelHeight = double(rect.Height()) / NORM_DIGIHEIGHT;

	// If proportional make offset for centered text
	if (m_DispStyle & DS_SZ_PROP)
	{
		if (dRelWidth < dRelHeight)
			dRelHeight = dRelWidth;
		else
			dRelWidth = dRelHeight;

		CharRect.left = (rect.Width() - dRelWidth * iTotWidth) / 2;
		CharRect.top = (rect.Height() - dRelHeight * NORM_DIGIHEIGHT) / 2; 
	}
	else
		CharRect.SetRectEmpty();
	if (m_strText == "MAGIC")
		TRACE("%d %d\n", rect.top, rect.bottom);

	CRgn* region = m_bTransparent ? new CRgn : NULL;
	// Draw all characters...
	for (int iChar = 0; iChar < m_CharArray.GetSize(); iChar++)
	{

		CharRect.SetRect(CharRect.left, CharRect.top,
						 CharRect.left + dRelWidth * m_CharArray[iChar].GetNormWidth(), 
						 CharRect.top  + dRelHeight * NORM_DIGIHEIGHT);

		m_CharArray[iChar].Draw(&MemDC, CharRect, &OffPen, &OnPen, &hBrushOff, &hBrushOn, region);

		CharRect.left += dRelWidth * m_CharArray[iChar].GetNormWidth();
	}

	if (region)
	{
		SetWindowRgn(*region,FALSE);
		delete region;
	}
	// Mama says: Clean up your mess!
	MemDC.SelectObject(pOldPen);
	MemDC.SelectObject(pOldBrush);
	OffPen.DeleteObject();
	OnPen.DeleteObject();
	hBrushOff.DeleteObject();
	hBrushOn.DeleteObject();
}

BOOL CDigiStatic::OnEraseBkgnd(CDC* /*pDC*/) 
{
	// Don't erase the background to avoid flickering
	// Background is painted in CMemDC::CMemDC(); with FillSolidRect();
	return FALSE;
}

void CDigiStatic::DoInvalidate()
{
	if (m_bTransparent)
	{
		SetWindowRgn(NULL,TRUE);
	}

	if (m_bImmediateUpdate)
		RedrawWindow(NULL, NULL,  RDW_INVALIDATE | RDW_UPDATENOW | RDW_NOERASE);
	else
		Invalidate();
}

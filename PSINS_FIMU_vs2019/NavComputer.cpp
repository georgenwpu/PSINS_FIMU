#include "pch.h"
#include "NavComputer.h"

CKFApp kf(TS);
int state = 0;		// �����״̬��־��0-��ʼ����1-��׼��2-����׼��3-�����ԣ�4-��ϣ�

DWORD __stdcall NavigationThread(LPVOID lpara)
{
	NaviParaS* NPS = (NaviParaS*)lpara;

	/*-------------- Defination of Navigation Parameters -------------*/
	if (NPS->alignType == 0)
	{
		aligni0 = CAligni0();
	}
	else if (NPS->alignType == 1) 
	{

	}

	

	while (NPS->ComputerONOFF)
	{
		if (PS_INS)
		{
			PS_INS = FALSE;
			tk += TS;		// update INS time
			if (tk <= NPS->alignTimeC)
			{
				
				if (tk == NPS->alignTimeC)		// State exchaange
				{

				}
			}
			else if (tk <= NPS->alignTImeF)
			{

			}
			else
			{

			}
		}
		
		if (PS_GNSS)
		{
			PS_GNSS = FALSE;
		}

	}
	return 0;
}

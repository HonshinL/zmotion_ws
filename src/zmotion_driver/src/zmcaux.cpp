
/********************************** ZMCϵ�п�����  ************************************************
**--------------�ļ���Ϣ--------------------------------------------------------------------------------
**�ļ���: zmcaux.c
**������: zxy
**ʱ��: 20130621
**����: ZMCDLL ��������

�����ṩ��EXCUTE����������ִ�к����ķ�װ�����Կ�����������ִ�г�������޸Ĳ���.
  
	
	  
**------------�޶���ʷ��¼----------------------------------------------------------------------------
		
** �޸���: zxy
** ��  ��: 1.1
** �ա���: 2014.5.11
** �衡��: ZMC_ExecuteNoAck �滻Ϊ ZMC_Execute
		  
			
** �޸���: zxy
** ��  ��: 1.3
** �ա���: 2014.7.21
** �衡��: ZMC_Execute ZMC_DirectCommand �滻ΪZAux_Execute ZAux_DirectCommand
			  
���� ZAux_SetParam  ZAux_GetParam  ZAux_Direct_SetParam  ZAux_Direct_GetParam
				
���� ZAux_WriteUFile  ZAux_ReadUFile
				  
** �޸���: wy
** ��  ��: 1.5
** �ա���: 2016.6.6
** �衡��: ������BASICָ����з�װ������ZMC�⵽AUX��


  ** �޸���: wy
** ��  ��: 2.1
** �ա���: 2018.8.24
** ��  ��������PCI���Ӻ���
**		   ������BASICָ���˶�ָ����з�װ����װ���б�������
**		   ���Ӳ�������ָ��
**		   ���Ӳ���MOVE_PARAָ��
		   ����λ�ñȽ����ָ��
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

//#include "stdafx.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include "stdarg.h"
#include "ctype.h"
#include "malloc.h"
#include "stdlib.h"



#include "zmotion_driver/zmotion.h"
#include "zmotion_driver/zmcaux.h"

#ifdef Z_DEBUG
#undef THIS_FILE
static const char THIS_FILE[] = "zmcaux";
#endif

int g_ZMC_MaxExcuteWaitms = 1000;
int g_ZMC_bIfDebugtoFile = false;
char g_ZMC_aDebugFileName[2048] = "zauxcmd.txt";

#if 0
//�����������
#endif

void  ZAux_TraceOut(const char * cText, ...)
{
    char ErrorText[2048];

	
    va_list valist;
    
    // Build variable text buffer
    va_start(valist, cText);
    vsprintf(ErrorText, cText, valist);
    va_end(valist);
	
	
    //OutputDebugString( ErrorText );
	
	
#ifdef ZAUX_DEBUG
	FILE * DebugFileId;
	DebugFileId = fopen(ZAUX_DEBUG_FILE,"a");
	if(NULL == DebugFileId)
	{
		::MessageBox(NULL, "Can not open file", _T("Error"), MB_OK | MB_ICONEXCLAMATION);
	}
	else
	{
		fputs(ErrorText, DebugFileId);
		fclose(DebugFileId);
	}
	
#endif    
	
}

/*************************************************************
Description:    //��������������ӣ� ���ڷ�ʽ.
Input:          //���ں�COMId 
Output:         //������phandle
Return:         //������
*************************************************************/
int32  ZAux_OpenCom(uint32 comid, ZMC_HANDLE * phandle)
{
	int32 iresult;
	iresult = ZMC_OpenCom(comid, phandle);
	
	return iresult;
}

/*************************************************************
Description:    //���ٿ�������������
Input:          //��С���ں�uimincomidfind
Input:          //��󴮿ں�uimaxcomidfind
Input:          //����ʱ��uims
Output:         //��ЧCOM pcomid
Output:         //������handle
Return:         //������
*************************************************************/
int32  ZAux_SearchAndOpenCom(uint32 uimincomidfind, uint32 uimaxcomidfind,uint* pcomid, uint32 uims, ZMC_HANDLE * phandle)
{
	int32 iresult;
	iresult =ZMC_SearchAndOpenCom(uimincomidfind, uimaxcomidfind, pcomid,uims, phandle);
	
	return iresult;
}

/*************************************************************
Description:    //�����޸�ȱʡ�Ĳ����ʵ�����
Input:          //dwBaudRate ������
				dwByteSize   ����λ 
				dwParity = NOPARITY,У��λ 
				dwStopBits = ONESTOPBITֹͣλ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_SetComDefaultBaud(uint32 dwBaudRate, uint32 dwByteSize, uint32 dwParity, uint32 dwStopBits)
{
	int32 iresult;
	iresult = ZMC_SetComDefaultBaud(dwBaudRate,dwByteSize,dwParity,dwStopBits);
	
	return iresult;
}


/*************************************************************
Description:    //�޸Ŀ�����IP��ַ
Input:          //������handle 
Input:          //ipaddress IP��ַ  
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_SetIp(ZMC_HANDLE handle, char * ipaddress)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "IP_ADDRESS=%s", ipaddress);
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return  ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //���������������
Input:          //IP��ַ���ַ����ķ�ʽ����
Output:         //������handle
Return:         //������
*************************************************************/
int32  ZAux_OpenEth(char *ipaddr, ZMC_HANDLE * phandle)
{
	int32 iresult;
	iresult = ZMC_OpenEth(ipaddr, phandle);
	
	return iresult;
}

/*************************************************************
Description:    //���ټ���IP�б�
Input:          //uims ��Ӧʱ��
Input:          //addrbufflength		��󳤶�
output:			//ipaddrlist		��ǰ����IP�б�
Return:         //������, ERR_OK��ʾ��������.
*************************************************************/
int32  ZAux_SearchEthlist(char *ipaddrlist, uint32 addrbufflength, uint32 uims)
{
    //�Զ�����IP��ַ
	int32 iresult;
    iresult = ZMC_SearchEth(ipaddrlist, addrbufflength, uims);
	return iresult;
}

/*************************************************************
Description:    //���ټ���������
Input:          //ipaddress ������IP��ַ
Input:          //uims ��Ӧʱ��
Output:         //
Return:         //������, ERR_OK��ʾ��������.
*************************************************************/
int32  ZAux_SearchEth(const char *ipaddress,  uint32 uims)
{
    //�Զ�����IP��ַ
    char buffer[10240];
	int32 iresult;
    
    //
    iresult = ZMC_SearchEth(buffer, 10230, uims);
    if(ERR_OK != iresult)
    {
		return 20010; //����
    }
	
    //���ַ���ת������
    int ipos =0;
    const char * pstring;
    pstring = buffer;
	
    for(int j= 0; j< 100;j++) //���100��IP�б�
    {
        char buffer2[256];
        buffer2[0] = '\0';
		
        //�����ո�
        while(' ' == pstring[0])
        {
            pstring++;
        }
        
        ipos = sscanf(pstring , "%s", &buffer2);
        if(EOF  == ipos)
        {
            break;
        }
		
        //�����ַ�
        while((' ' != pstring[0]) && ('\t' != pstring[0]) && ('\0' != pstring[0]))
        {
            pstring++;
        }
        
		if(0 == strcmp(buffer2, ipaddress))
		{
			return ERR_OK;
		}
		
    }
    
	return 20010; //����
}


/*************************************************************
Description:    //�رտ���������
Input:          //������handle
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Close(ZMC_HANDLE  handle)
{
	int32 iresult;
	iresult = ZMC_Close(handle);
	
	return iresult;	
}

/*************************************************************
Description:    //��ͣ��������BAS��Ŀ
Input:          //������handle
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Resume(ZMC_HANDLE handle)
{
	int32 iresult;
	iresult = ZMC_Resume(handle);
	
	return iresult;	
}

/*************************************************************
Description:    //��ͣ��������BAS����
Input:          //������handle
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Pause(ZMC_HANDLE handle)
{
	int32 iresult;
	iresult = ZMC_Pause(handle);
	
	return iresult;	
}


/*************************************************************
Description:    //����BAS�ļ�����ZAR�������ص�����������
Input:          //������handle
Input:          //Filename BAS�ļ�·��
Input:          //���ص�RAM-ROM  0-RAM  1-ROM
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BasDown(ZMC_HANDLE handle,const char *Filename,uint32 run_mode )
{
	int32 	iresult;
    char    atemp[4096];
    char    adir[4096];
	
    iresult = ZMC_MakeOneFileZpj(atemp, adir, Filename);
    if(ERR_OK != iresult)
    {
        return iresult;
    }
    
	if (run_mode == 0)
	{
		iresult = ZMC_MakeZarAndRamRun2(handle, atemp, adir, NULL, 0);
    
	}
	else
	{
		iresult = ZMC_MakeZarAndDown2(handle, atemp, adir, NULL, 0);
	}

	if(ERR_OK != iresult)
    {
		return iresult;
    }        
	
    iresult = ZMC_Resume(handle);
    if(ERR_OK != iresult)
    {
		return iresult;
    }  
	
	return ERR_OK;
}



/*************************************************************
Description:    //��װ Excute ����, �Ա���մ���
Input:          //������			handle
Input:          //�ַ�������		pszCommand
Input:			//���ص��ַ�����	uiResponseLength 
Output:         //���ص��ַ���		psResponse
Return:         //������
*************************************************************/
int32  ZAux_Execute(ZMC_HANDLE handle, const char* pszCommand, char* psResponse, uint32 uiResponseLength)
{
	int32 iresult;
	iresult = ZMC_Execute(handle, pszCommand, g_ZMC_MaxExcuteWaitms, psResponse, uiResponseLength);
	if(ERR_OK != iresult)
	{
		ZAUX_ERROR2("ZMC_Execute:%s error:%d.",  pszCommand, iresult);
	}

	//������д���ļ�
	if(g_ZMC_bIfDebugtoFile)
	{
		FILE * DebugFileId;
		char  Backbuff[2048];
		char time_str[32];
		time_t SysTime = time(0); 
		DebugFileId = fopen(g_ZMC_aDebugFileName,"a");
		if(NULL == DebugFileId)
		{
			//::MessageBox(NULL, "Can not open file", _T("Error"), MB_OK | MB_ICONEXCLAMATION);
		}
		else
		{

			if(g_ZMC_bIfDebugtoFile == 1)		//�������
			{
				if(ERR_OK != iresult) 
				{
					strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&SysTime));
					sprintf(Backbuff,"%s\t%s\tError:%d\r",time_str,pszCommand,iresult);
					fputs(Backbuff, DebugFileId);
				}

			}
			else if( g_ZMC_bIfDebugtoFile == 2)  //�Ƕ�ȡ����
			{
				//
				if(0 == uiResponseLength) 
				{
					strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&SysTime));

					if(iresult != 0 )
					{
						sprintf(Backbuff,"%s\t%s\tError:%d\r",time_str,pszCommand,iresult);
					}
					else
					{
						sprintf(Backbuff,"%s\t%s\r",time_str,pszCommand);				
					}
					fputs(Backbuff, DebugFileId);
				}
			}
			else										
			{
				strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&SysTime));

				if(iresult != 0 )
				{
					sprintf(Backbuff,"%s\t%s\tError:%d\r",time_str,pszCommand,iresult);

				}
				else if(0 != uiResponseLength)
				{
					sprintf(Backbuff,"%s\t%s\tRe:%s\r",time_str,pszCommand,psResponse);
				}
				else
				{
					sprintf(Backbuff,"%s\t%s\r",time_str,pszCommand);			
				}
				fputs(Backbuff, DebugFileId);
			}

			fclose(DebugFileId);
		}
	}

	return iresult;
}


/*************************************************************
Description:    //��װ DirectCommand ����, �Ա���մ���
Input:          //������			handle
Input:          //�ַ�������		pszCommand
Input:			//���ص��ַ�����	uiResponseLength 
Output:         //���ص��ַ���		psResponse
Return:         //������
*************************************************************/
int32  ZAux_DirectCommand(ZMC_HANDLE handle, const char* pszCommand, char* psResponse, uint32 uiResponseLength)
{
	int32 iresult;
	iresult = ZMC_DirectCommand(handle, pszCommand, psResponse, uiResponseLength);
	if(ERR_OK != iresult)
	{
		ZAUX_ERROR2("ZMC_DirectCommand:%s error:%d.", pszCommand, iresult);
	}

	//������д���ļ�
	if(g_ZMC_bIfDebugtoFile)
	{
		FILE * DebugFileId;
		char  Backbuff[2048];
		char time_str[32];
		time_t SysTime = time(0); 
		DebugFileId = fopen(g_ZMC_aDebugFileName,"a");
		if(NULL == DebugFileId)
		{
			//::MessageBox(NULL, "Can not open file", _T("Error"), MB_OK | MB_ICONEXCLAMATION);
		}
		else
		{

			if(g_ZMC_bIfDebugtoFile == 1)		//�������
			{
				if(ERR_OK != iresult) 
				{
					strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&SysTime));
					sprintf(Backbuff,"%s\t%s\tError:%d\r",time_str,pszCommand,iresult);
					fputs(Backbuff, DebugFileId);
				}

			}
			else if( g_ZMC_bIfDebugtoFile == 2)  //�Ƕ�ȡ����
			{
				//
				if(0 == uiResponseLength) 
				{
					strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&SysTime));

					if(iresult != 0 )
					{
						sprintf(Backbuff,"%s\t%s\tError:%d\r",time_str,pszCommand,iresult);
					}
					else
					{
						sprintf(Backbuff,"%s\t%s\r",time_str,pszCommand);				
					}
					fputs(Backbuff, DebugFileId);
				}
			}
			else										
			{
				strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&SysTime));

				if(iresult != 0 )
				{
					sprintf(Backbuff,"%s\t%s\tError:%d\r",time_str,pszCommand,iresult);

				}
				else if(0 != uiResponseLength)
				{
					sprintf(Backbuff,"%s\t%s\tRe:%s\r",time_str,pszCommand,psResponse);
				}
				else
				{
					sprintf(Backbuff,"%s\t%s\r",time_str,pszCommand);			
				}
				fputs(Backbuff, DebugFileId);
			}

			fclose(DebugFileId);
		}
	}
	return iresult;
}


/*************************************************************
Description:    //�����������.
Input:          //������handle 
bifTofile		0 �ر�  1-ֻ�����������  2-ֻ����˶�����������  3���ȫ������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_SetTraceFile(int bifTofile, const char *pFilePathName)
{
	g_ZMC_bIfDebugtoFile = bifTofile;
	strcpy(g_ZMC_aDebugFileName, pFilePathName);
	
	return ERR_OK;
}


#if 0
//****************************************************IOָ��**************************
// ����ʹ�� ZMC_GetIn ZMC_GetOutput ��
#endif

/*************************************************************
Description:    //��ȡ�����ź�
Input:          //������handle
				ionum IN���
Output:         //piValue �����״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetIn(ZMC_HANDLE handle, int ionum , uint32 *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?IN(%d)", ionum);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //������ź�
Input:          //������handle 
				ionum ����ڱ��
				iValue	�����״̬
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetOp(ZMC_HANDLE handle, int ionum, uint32 iValue)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "op(%d,%d)", ionum, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�����״̬
Input:          //������handle  
				ionum ����ڱ��
Output:         //piValue �����״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetOp(ZMC_HANDLE handle, int ionum, uint32 *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?OP(%d)", ionum);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡģ���������ź�
Input:          //������handle 
				ionum AIN�ڱ��			
Output:         //pfValue ���ص�ģ����ֵ 4ϵ������0-4095
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAD(ZMC_HANDLE handle, int ionum , float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?AIN(%d)", ionum);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ģ��������ź�
Input:          //������handle  
				ionum DA����ڱ��
				fValue �趨��ģ����ֵ4ϵ������0-4095
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetDA(ZMC_HANDLE handle, int ionum, float fValue)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "AOUT(%d) = %f", ionum, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡģ�������״̬
Input:          //������handle  
				ionum ģ��������ڱ��
Output:         //pfValue ��ȡ�ĵ�ģ����ֵ 4ϵ������0-4095
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetDA(ZMC_HANDLE handle, int ionum, float *pfValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?AOUT(%d)", ionum);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��������ڷ�ת
Input:          //������handle  
				 ionum ����ڱ��
				 bifInvert ��ת״̬ 0/1
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetInvertIn(ZMC_HANDLE handle, int ionum, int bifInvert)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "INVERT_IN(%d,%d)", ionum, bifInvert);
	
	//��������ִ�к���
	return  ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����ڷ�ת״̬
Input:          //������handle  
				ionum ����ڱ��
Output:         //piValue ��ת״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetInvertIn(ZMC_HANDLE handle, int ionum, int *piValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?INVERT_IN(%d)", ionum);
	
	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //����pwmƵ��
Input:          //������handle  
				ionum PWM��ſ�
				fValue Ƶ�� Ӳ��PWM1M ��PWM 2K
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetPwmFreq(ZMC_HANDLE handle, int ionum, float fValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "PWM_FREQ(%d) = %f", ionum, fValue);
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
}

/*************************************************************
Description:    //��ȡpwmƵ��
Input:          //������handle  
				ionum PWM�ڱ��
Output:         //pfValue ���ص�Ƶ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetPwmFreq(ZMC_HANDLE handle, int ionum, float *pfValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
   	if(NULL == pfValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?PWM_FREQ(%d)", ionum);
	
	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����pwmռ�ձ�
Input:          //������handle  
				ionum PWM�ڱ��
				fValue ռ�ձ�	0-1  0��ʾ�ر�PWM��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetPwmDuty(ZMC_HANDLE handle, int ionum, float fValue)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "PWM_DUTY(%d) = %f", ionum, fValue);
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
}

/*************************************************************
Description:    //��ȡpwmռ�ձ�
Input:          //������handle  
				ionum PWM�ڱ��
Output:         //pfValue ��ȡ��ռ�ձ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetPwmDuty(ZMC_HANDLE handle, int ionum, float *pfValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
   	if(NULL == pfValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?PWM_DUTY(%d)", ionum);
	
	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}



#if 0
//ͨ��modbus���ٶ�ȡ����Ĵ���
#endif

/*************************************************************
Description:    //���� ���ٶ�ȡ�������
Input:          //������handle  
				ionumfirst IN��ʼ���
				ionumend	IN�������
Output:         //pValueList λ״̬ ��λ�洢
Return:         //������
*************************************************************/
int32  ZAux_GetModbusIn(ZMC_HANDLE handle, int ionumfirst, int ionumend, uint8 *pValueList)
{
    if(ionumend < ionumfirst)
    {
        return ERR_AUX_PARAERR;
    }
    
    return ZMC_Modbus_Get0x(handle, 10000+ionumfirst, ionumend-ionumfirst +1, pValueList);
}

/*************************************************************
Description:    //���� ���ٶ�ȡ�����ǰ�����״̬
Input:          //������handle  
				ionumfirst IN��ʼ���
				ionumend	IN�������
Output:         //pValueList λ״̬ ��λ�洢
Return:         //������
*************************************************************/
int32  ZAux_GetModbusOut(ZMC_HANDLE handle, int ionumfirst, int ionumend, uint8 *pValueList)
{
    if(ionumend < ionumfirst)
    {
        return ERR_AUX_PARAERR;
    }
    
    return ZMC_Modbus_Get0x(handle, 20000+ionumfirst, ionumend-ionumfirst +1, pValueList);
}

/*************************************************************
Description:    //���� ���ٶ�ȡ�����ǰ��DPOS
Input:          //������handle  
				imaxaxises ������
Output:         //pValueList ��ȡ������ֵ ����0��ʼ
Return:         //������
*************************************************************/
int32  ZAux_GetModbusDpos(ZMC_HANDLE handle, int imaxaxises, float *pValueList)
{
    return ZMC_Modbus_Get4x(handle, 10000, imaxaxises*2, (uint16 *)pValueList);
}

/*************************************************************
Description:    //���� ���ٶ�ȡ�����ǰ��MPOS
Input:          //������handle 
				imaxaxises ������
Output:         //pValueList ��ȡ�ķ�������ֵ ����0��ʼ
Return:         //������
*************************************************************/
int32  ZAux_GetModbusMpos(ZMC_HANDLE handle, int imaxaxises, float *pValueList)
{
    return ZMC_Modbus_Get4x(handle, 11000, imaxaxises*2, (uint16 *)pValueList);
}

/*************************************************************
Description:    //���� ���ٶ�ȡ�����ǰ���ٶ�
Input:          //������handle  
				imaxaxises ������
Output:         //pValueList ��ȡ�ĵ�ǰ�ٶ� ����0��ʼ
Return:         //������
*************************************************************/
int32  ZAux_GetModbusCurSpeed(ZMC_HANDLE handle, int imaxaxises, float *pValueList)
{
    return ZMC_Modbus_Get4x(handle, 12000, imaxaxises*2, (uint16 *)pValueList);
}

#if 0
//����ZAux_DirectCommand �����ٻ�ȡһЩ״̬, ZAux_DirectCommand��ִ�б�ZMC_ExecuteҪ��
// ֻ�в���������������Ԫ�ص���ʹ��ZAux_DirectCommand
// 20130901�Ժ�İ汾��һЩ�˶�����Ҳ���Ե���ZAux_DirectCommand�����˶������������ʱ�򣬻����̷���ʧ�ܡ�
// ZAux_DirectCommand�����˶�����ʱ�����������Ǿ������ֵ�������Ǳ�������ʽ��
#endif

#if 0
//**************************************���������***************************************
#endif


/*************************************************************
Description:    //ͨ�õĲ����޸ĺ��� sParam: ��д��������
Input:          //������handle 
				sParam ��������� "DPOS" ...
				iaxis ���
				fset �趨ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetParam(ZMC_HANDLE handle,const char *sParam,int iaxis, float fset)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
    //��������
	sprintf(cmdbuff, "%s(%d)=%f", sParam,iaxis, fset);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���� ͨ�õĲ�����ȡ����, sParam:��д��������
Input:          //������handle  
				sParam ��������� "DPOS" ...
				iaxis ���
Output:         //pfValue  ��ȡ�ķ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetParam(ZMC_HANDLE handle,const char *sParam, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?%s(%d)", sParam,iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ü��ٶ�
Input:          //������handle  
				iaxis ���
				fValue  �趨ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetAccel(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "ACCEL(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ٶȷ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAccel(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ACCEL(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ������
Input:          //������handle  
				iaxis ���
Output:         //��ȡ����������
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAddax(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ADDAX_AXIS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //������澯�ź�
Input:          //������handle  
				iaxis ���
				iValue �����ź�����ڱ�ţ�ȡ��ʱ�趨-1
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetAlmIn(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "ALM_IN(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�澯�ź�
Input:          //������handle  
				iaxis ���
Output:         //piValue �����ź�����ڷ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAlmIn(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ALM_IN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����������
Input:          //������handle 
				iaxis ���
				iValue ������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetAtype(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "ATYPE(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ������
Input:          //������handle 
				iaxis ���
Output:         //iValue �����ͷ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAtype(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ATYPE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��״̬
Input:          //������handle  
				iaxis ���
Output:         //��״̬����ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAxisStatus(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?AXISSTATUS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�������ַ
Input:          //������handle  
				iaxis ���
				iValue ���ַ�趨ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetAxisAddress(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "AXIS_ADDRESS(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ַ
Input:          //������handle  
				iaxis ��� 
Output:         //piValue ���ַ����ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAxisAddress(ZMC_HANDLE handle, int iaxis, int *piValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?AXIS_ADDRESS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //������ʹ�� ��ֻ������߿�������ʹ����Ч��
Input:          //������handle  
				iaxis ���
				iValue ״̬ 0-�ر� 1- ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetAxisEnable(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "AXIS_ENABLE(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ��ʹ��״̬
Input:          //������handle 
				iaxis ���
Output:         //piValue ���ص�ʹ��״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAxisEnable(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?AXIS_ENABLE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //������������
Input:          //������handle 
				iaxis ���
				fValue ͬ����������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetClutchRate(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "CLUTCH_RATE(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ��������
Input:          //������handle  
				iaxis ���
Output:         //pfValue �������ʷ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetClutchRate(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?CLUTCH_RATE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�������津���Ľ������귶Χ��
Input:          //������handle  
				iaxis ���
				fValue �趨�ķ�Χֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetCloseWin(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "CLOSE_WIN(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���津���Ľ������귶Χ��
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صķ�Χֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetCloseWin(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?CLOSE_WIN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ùսǼ���
Input:          //������handle 
				iaxis ���
				iValue �սǼ���ģʽ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetCornerMode(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "CORNER_MODE(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�սǼ���
Input:          //������handle  
				iaxis ���
Output:         //piValue ���صĹս�ģʽ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetCornerMode(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?CORNER_MODE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���û��������ٶ�
Input:          //������handle  
				iaxis ���
				fValue���õ��ٶ�ֵ
Output:         //	
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetCreep(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "CREEP(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���������ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص������ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetCreep(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?CREEP(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����ԭ���ź�   �趨-1Ϊȡ��ԭ������
Input:          //������handle 
				iaxis ���
				iValue ���õ�ԭ���ź�����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetDatumIn(ZMC_HANDLE handle, int iaxis, int iValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "DATUM_IN(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡԭ���ź�
Input:          //������handle  
				iaxis 
Output:         //piValue ����ԭ������ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetDatumIn(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?DATUM_IN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ü��ٶ�
Input:          //������handle
				iaxis ���
				fValue ���õļ��ٶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetDecel(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "DECEL(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ٶ�
Input:          //������handle 
				iaxis ���
Output:         //pfValue �趨�ļ��ٶȷ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetDecel(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?DECEL(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ùսǼ��ٽǶȣ���ʼ���ٽǶȣ���λΪ����
Input:          //������handle  
				iaxis ���
				fValue ���õĹսǼ��ٽǶ�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetDecelAngle(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "DECEL_ANGLE(%d)=%.3f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�սǿ�ʼ���ٽǶȣ���λΪ����
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صĹսǼ��ٽǶ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetDecelAngle(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?DECEL_ANGLE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //������λ��
Input:          //������handle
				iaxis ���
				fValue ���õ�����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetDpos(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "DPOS(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ��λ��
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص�����λ������
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetDpos(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?DPOS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�ڲ�������ֵ  �����߾���ֵ�ŷ�ʱΪ����ֵλ�ã�
Input:          //������handle  
				iaxis ���		
Output:         //pfValue ���ص��ڲ�������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetEncoder(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ENCODER(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�˶�������λ��
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص�����λ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetEndMove(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ENDMOVE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�ͻ������˶�������λ�ã�����������Ծ���ת��
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص�����λ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetEndMoveBuffer(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ENDMOVE_BUFFER(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����SP�˶��Ľ����ٶ�
Input:          //������handle  
				iaxis ���
Output:         //fValue �趨���ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetEndMoveSpeed(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "ENDMOVE_SPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡSP�˶��Ľ����ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص��ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetEndMoveSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ENDMOVE_SPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ô����ǣ���AXISSTATUS����������������Щ������Ҫ�ر�WDOG��
Input:          //������handle 
				iaxis ���
				iValue ����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetErrormask(ZMC_HANDLE handle, int iaxis, int iValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "ERRORMASK(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�����ǣ���AXISSTATUS����������������Щ������Ҫ�ر�WDOG��
Input:          //������handle 
				iaxis ���
Output:         //piValue ���صı��ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetErrormask(ZMC_HANDLE handle, int iaxis , int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?ERRORMASK(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ÿ���JOG����
Input:          //������handle  
				iaxis ���
				iValue ����JOG����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFastJog(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FAST_JOG(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����JOG����
Input:          //������handle  
				iaxis ���
Output:         //���ص�JOG����ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFastJog(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FAST_JOG(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ÿ��ټ��ٶ�
Input:          //������handle  
				iaxis ���
				fValue �趨�Ŀ��ټ��ٶ�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFastDec(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FASTDEC(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ټ��ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صĿ��ټ��ٶ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFastDec(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FASTDEC(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�涯���
Input:          //������handle 
				iaxis ���
Output:         //pfValue ���ص��涯���
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFe(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��������������涯���ֵ
Input:          //������handle  
				iaxis ���
				fValue ���õ�������ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFeLimit(ZMC_HANDLE handle, int iaxis, float fValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FE_LIMIT(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����������涯���ֵ
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص�����������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFeLimit(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FE_LIMIT(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ñ���ʱ�涯���ֵ
Input:          //������handle  
				iaxis ���
				fValue ���õ����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFRange(ZMC_HANDLE handle, int iaxis, float fValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FE_RANGE(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����ʱ���涯���ֵ
Input:          //������handle
				iaxis ���
Output:         //pfValue ���صı������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFeRange(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FE_RANGE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //���ñ�������
Input:          //������handle  
				iaxis ���
				iValue ���õ�����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFholdIn(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FHOLD_IN(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ��������
Input:          //������handle 
				iaxis ���
Output:         //piValue ��������HOLDIN����ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFholdIn(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FHOLD_IN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //�����ᱣ���ٶ�
Input:          //������handle  
				iaxis ���
				fValue ���õ��ٶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFhspeed(ZMC_HANDLE handle, int iaxis, float fValue)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FHSPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�ᱣ���ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صı����ٶ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFhspeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FHSPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����SP�˶��������ٶ�
Input:          //������handle  
				iaxis ���
				fValue ���õ��ٶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetForceSpeed(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FORCE_SPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡSP�˶��������ٶ�
Input:          //������handle
				iaxis ���
Output:         //pfValue ����SP�˶��ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetForceSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FORCE_SPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //������������λ		ȡ��ʱ����һ���ϴ�ֵ����
Input:          //������handle  
				iaxis ���
				fValue �趨����λֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFsLimit(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FS_LIMIT(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ��������λ
Input:          //������handle
				iaxis ���
Output:         //pfValue ���ص�������λ����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFsLimit(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FS_LIMIT(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����СԲ������С�뾶
Input:          //������handle  
				iaxis ���
				fValue ���õ���С�뾶
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFullSpRadius(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FULL_SP_RADIUS(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡСԲ������С�뾶
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص����ٰ뾶
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFullSpRadius(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FULL_SP_RADIUS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��������Ӳ��λ����  ���ó�-1ʱ��ʾ��������λ
Input:          //������handle  
				iaxis ���
				iValue ���õ���λ����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFwdIn(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FWD_IN(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����Ӳ��λ����
Input:          //������handle  
				iaxis ���
Output:         //piValue ����������λ����ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFwdIn(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FWD_IN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��������JOG����
Input:          //������handle  
				iaxis ���
				iValue ���õ�JOG����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetFwdJog(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "FWD_JOG(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����JOG����
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص�JOG����ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetFwdJog(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?FWD_JOG(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ���Ƿ��˶�����
Input:          //������handle  
				iaxis ���
Output:         //piValue ��������״̬ 0-�˶��� -1 ֹͣ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetIfIdle(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?IDLE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�����������ģʽ
Input:          //������handle  
				iaxis ���
				iValue �趨���������ģʽ ����+����/˫����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetInvertStep(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "INVERT_STEP(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�������ģʽ
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص�����ģʽ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetInvertStep(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?INVERT_STEP(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ò岹ʱ���Ƿ�����ٶȼ��㣬ȱʡ���루1�����˲���ֻ��ֱ�ߺ������ĵ�������������
Input:          //������handle  
				iaxis	���
				iValue ģʽ 0-������ 1-����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetInterpFactor(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "INTERP_FACTOR(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�岹ʱ���Ƿ�����ٶȼ��㣬ȱʡ���루1�����˲���ֻ��ֱ�ߺ������ĵ�������������
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص��ٶȼ���ģʽ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetInterpFactor(ZMC_HANDLE handle, int iaxis, int *piValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?INTERP_FACTOR(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //����JOGʱ�ٶ�
Input:          //������handle  
				iaxis ���
Output:         //fValue �趨���ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetJogSpeed(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "JOGSPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡJOGʱ�ٶ�
Input:          //������handle  
				iaxis	���
Output:         //pfValue ���ص�JOG�ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetJogSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?JOGSPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�����˶��Ĳο����
Input:          //������handle  
				iaxis ���
Output:         //piValue �������ӵĲο����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetLinkax(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?LINK_AXIS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ���˵�ǰ�˶��Ƿ��л��� 
Input:          //������handle
				iaxis ���
Output:         //piValue ����״ֵ̬  -1 û��ʣ�ຯ�� 0-����ʣ���˶�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetLoaded(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?LOADED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��������ʼ�ٶ�
Input:          //������handle  
				iaxis ���
				fValue ���õ��ٶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetLspeed(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "LSPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����ʼ�ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص���ʼ�ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetLspeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?LSPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���û��㷴�ҵȴ�ʱ��
Input:          //������handle 
				iaxis ���
				iValue ���㷴�ҵȴ�ʱ�� MS
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetHomeWait(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "HOMEWAIT(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���㷴�ҵȴ�ʱ��
Input:          //������handle  
				iaxis ���
Output:         //piValue ���صķ��ҵȴ�ʱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetHomeWait(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?HOMEWAIT(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ����������ʾ�̷���״̬
Input:          //������handle  
				iaxis ���
Output:         //piValue  ���ص����津��״̬ -1-���津�� 0-δ����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMark(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MARK(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ����������b����״̬
Input:          //������handle  
				iaxis ���
Output:         //piValue  ���ص����津��״̬ -1-���津�� 0-δ����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMarkB(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MARKB(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��������������Ƶ��
Input:          //������handle  
				iaxis ���
				iValue ���õ��������Ƶ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetMaxSpeed(ZMC_HANDLE handle, int iaxis, int iValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "MAX_SPEED(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����������Ƶ��
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص�����Ƶ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMaxSpeed(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MAX_SPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���������岹
Input:          //������handle  
				iaxis ���
				iValue	�����岹���� 0-�ر������岹 1-�������岹
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetMerge(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "MERGE(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�����岹״̬
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص������岹����״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMerge(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MERGE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�������������˶�����
Input:          //������handle  
				iaxis ����
Output:         //piValue �����˶���
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMovesBuffered(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MOVES_BUFFERED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�����˶�ָ���MOVE_MARK���
Input:          //������handle  
				iaxis ���
Output:         //piValue ��ǰMARK���
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMoveCurmark(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MOVE_CURMARK(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�����˶�ָ���MOVE_MARK��� ÿ�����˶��������˶�����ʱMARK�Զ�+1
Input:          //������handle  
				iaxis ���
				iValue �趨��MARKֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetMovemark(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "MOVE_MARK(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //���÷���λ��
Input:          //������handle  
				iaxis ���
				fValue ���õķ���λ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetMpos(ZMC_HANDLE handle, int iaxis, float fValue)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "MPOS(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����λ��
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص��ᷴ��λ������
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMpos(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MPOS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�����ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صı����������ٶ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMspeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MSPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�����˶�ָ������
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص�ǰ���˶�����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetMtype(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?MTYPE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ���ڽ��е��˶�ָ�����ĵ�һ��ָ�����ͣ����岹����ʱ���Դ������Ƿ���������˶�ָ������
Input:          //������handle  
				iaxis  ���
Output:         //piValue ������һ��ָ����˶�����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetNtype(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?NTYPE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�����޸�ƫ��λ��
Input:          //������handle 
				iaxis ���
				fValue ���õ�ƫ��ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetOffpos(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "OFFPOS(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�޸�ƫ��λ��
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص�ƫ������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetOffpos(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?OFFPOS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�������津���Ľ������귶Χ�㡣
Input:          //������handle  
				iaxis ���
				fValue ���õ�����ֵ
Output:         // 
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetOpenWin(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "OPEN_WIN(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���津���Ľ������귶Χ�㡣
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صĽ�������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetOpenWin(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?OPEN_WIN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��������Ĳ�������λ��(MPOS)
Input:          //������handle 
				iaxis ���
Output:         //pfValue ���������λ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRegPos(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REG_POS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��������Ĳ�������λ��(MPOS)
Input:          //������handle 
iaxis ���
Output:         //pfValue ���������λ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRegPosB(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REG_POSB(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}	

/*************************************************************
Description:    //��ȡ�����ᵱǰ�˶���δ��ɵľ���
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص�ʣ�����
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRemain(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REMAIN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}	

/*************************************************************
Description:    //����  ��ʣ��Ļ���, ��ֱ�߶�������
REMAIN_BUFFERΪΨһһ�����Լ�AXIS����ZAux_DirectCommand��ȡ��.
Input:          //������handle  
				iaxis ���
Output:         //piValue ʣ���ֱ�߻�������
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRemain_LineBuffer(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REMAIN_BUFFER(1) AXIS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //����  ��ʣ��Ļ���, ����ӵĿռ�Բ��������
REMAIN_BUFFERΪΨһһ�����Լ�AXIS����ZAux_DirectCommand��ȡ��.
Input:          //������handle  
				iaxis ���
Output:         //piValue ʣ��Ļ�������
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRemain_Buffer(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REMAIN_BUFFER() AXIS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //���ø���REP_OPTION�������Զ�ѭ����DPOS��MPOS���ꡣ
Input:          //������handle  
				iaxis ���
				fValue	���õ�����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetRepDist(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "REP_DIST(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����REP_OPTION�������Զ�ѭ����DPOS��MPOS���ꡣ
Input:          //������handle 
				iaxis	���
Output:         //pfValue ���ص�ѭ������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRepDist(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REP_DIST(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���������ظ�����
Input:          //������handle  
				iaxis ���
				iValue ģʽ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetRepOption(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "REP_OPTION(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�����ظ�����
Input:          //������handle
				iaxis ���
Output:         //piValue ���ص�ģʽ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRepOption(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REP_OPTION(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ø���Ӳ����λ���ض�Ӧ��������ţ�-1��Ч��
Input:          //������handle  
				iaxis  ���
				iValue ���õ�����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetRevIn(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "Rev_In(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����Ӳ����λ���ض�Ӧ��������ţ�-1��Ч��
Input:          //������handle  
				iaxis ���
Output:         //piValue ���صĸ�����λ����ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRevIn(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REV_IN(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ø���JOG�����Ӧ��������ţ�-1��Ч��
Input:          //������handle  
				iaxis ���
				iValue ���õ�����ڱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetRevJog(ZMC_HANDLE handle, int iaxis, int iValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "REV_JOG(%d)=%d", iaxis, iValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����JOG�����Ӧ��������ţ�-1��Ч��
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص�����ڱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRevJog(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?REV_JOG(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���ø�������λλ�á�  ����һ���ϴ��ֵʱ��Ϊȡ����λ
Input:          //������handle  
				iaxis	���
				fValue  ������λֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetRsLimit(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "RS_LIMIT(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ��������λλ�á�
Input:          //������handle  
				iaxis	���
Output:         //pfValue �趨����λֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetRsLimit(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?RS_LIMIT(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�������ٶȣ���λΪunits/s���������˶�ʱ����Ϊ�岹�˶����ٶ�
Input:          //������handle  
				iaxis ���
				fValue ���õ��ٶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetSpeed(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "SPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ٶȣ���λΪunits/s���������˶�ʱ����Ϊ�岹�˶����ٶ�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���ص��ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?Speed(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���� S�������á� 0-���μӼ���
Input:          //������handle  
				iaxis ���
				fValue S����ƽ��ʱ��MS
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetSramp(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "SRAMP(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ S�������á�
Input:          //������handle  
				iaxis ���
Output:         //pfValue ƽ��ʱ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetSramp(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?SRAMP(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���� �Զ����ٶȵ�SP�˶�����ʼ�ٶ�
Input:          //������handle 
				iaxis ���
				fValue ���õ��ٶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetStartMoveSpeed(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "STARTMOVE_SPEED(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ�Զ����ٶȵ�SP�˶�����ʼ�ٶ�
Input:          //������handle  
				iaxis	���
Output:         //pfValue ���ص�SP�˶���ʼ�ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetStartMoveSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?STARTMOVE_SPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //���� ���ٵ���͵���С�ս� ������
Input:          //������handle  
				iaxis ���
				fValue ���õĽǶ�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetStopAngle(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "STOP_ANGLE(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ٵ���͵���С�ս� ������
Input:          //������handle  
				iaxis ���
Output:         //pfValue ���صĹս�ֹͣ�Ƕ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetStopAngle(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?STOP_ANGLE(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //���� ���ٵ��ǰ뾶
Input:          //������handle  
				iaxis	���
				fValue	���ǰ뾶
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetZsmooth(ZMC_HANDLE handle, int iaxis, float fValue)
{


	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "ZSMOOTH(%d)=%f", iaxis, fValue);

	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���ǰ뾶
Input:          //������handle  
				iaxis	���
Output:         //pfValue	���صĵ��ǰ뾶ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetZsmooth(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?ZSMOOTH(%d)", iaxis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%f",pfValue);

	return ERR_OK;
}

/*************************************************************
Description:    //���� ���嵱��
Input:          //������handle  
				iaxis ���
				fValue ���õĵ���ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetUnits(ZMC_HANDLE handle, int iaxis, float fValue)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "UNITS(%d)=%f", iaxis, fValue);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ���嵱��
Input:          //������handle
				iaxis	���
Output:         //pfValue ���ص����嵱��
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetUnits(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?UNITS(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //��ȡ�����ᵱǰ��ǰ�˶��ͻ����˶���δ��ɵľ���
Input:          //������handle
				iaxis ���
Output:         //pfValue ���ص��������
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetVectorBuffered(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?VECTOR_BUFFERED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ�����е������ٶ�
Input:          //������handle  
				iaxis	���
Output:         //pfValue	���صĵ�ǰ�ٶ�ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetVpSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?VP_SPEED(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}


/*************************************************************
Description:    //ȫ�ֱ�����ȡ, Ҳ�����ǲ����ȵ�
Input:          //������handle  
				pname ȫ�ֱ�������/����ָ����ŵ����������DPOS(0)
Output:         //pfValue ����ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetVariablef(ZMC_HANDLE handle, const char *pname, float *pfValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?%s", pname);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%f",pfValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //ȫ�ֱ�����ȡ, Ҳ�����ǲ����ȵ�
Input:          //������handle  
				pname ȫ�ֱ�������/����ָ����ŵ����������DPOS(0)
Output:         //piValue ����ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetVariableInt(ZMC_HANDLE handle, const char *pname, int *piValue)
{

	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?%s", pname);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}


///////////////////////  ֻ��������˶�����֧��ֱ�ӵ��ã����������е�ָ�֧��
///////////////////////  ���� 20130901 �Ժ�Ŀ������汾֧��

/*************************************************************
Description:    //BASEָ�����
�����޸����������BASE�б������Կ����������������BASE�����޸�.
�޸ĺ󣬺���������MOVE��ָ��������BASEΪ����  
Input:          //������handle
				imaxaxises ��������
				piAxislist	���б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Base(ZMC_HANDLE handle, int imaxaxises, int *piAxislist)
{
	int i;
	char  cmdbuffAck[2048];	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	
	//
	
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}




/*************************************************************
Description:    //����DPOS,������ʹ�ã�����ֱ�ӵ���SETDPOS�ﵽͬ��Ч��
Input:          //������handle
				iaxis	���
				pfDpos ���õ�����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Defpos(ZMC_HANDLE handle, int iaxis, float pfDpos)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	//
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	sprintf(cmdbuff, "DEFPOS(%f) AXIS(%d)",pfDpos,iaxis);

	//��������ִ�к���
	return ZAux_Execute(handle, cmdbuff,  cmdbuffAck,2048);
	//return   ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //�������ֱ�߲岹  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Move(ZMC_HANDLE handle, int imaxaxises, int *piAxislist,float *pfDisancelist)
{
	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	//
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
    //��������
	strcat(cmdbuff, "MOVE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%f,",pfDisancelist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%f)",pfDisancelist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��Զ���ֱ�߲岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist)
{
	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];		
	//
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
    //��������
	strcat(cmdbuff, "MOVESP(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%f,",pfDisancelist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%f)",pfDisancelist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //���Զ���ֱ�߲岹  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist)
{
	int i;

	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];		
	//
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
    //��������
	strcat(cmdbuff, "MOVEABS(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%f,",pfDisancelist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%f)",pfDisancelist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���Զ���ֱ�߲岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist)
{
	int i;

	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];		
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
		
    //��������
	strcat(cmdbuff, "MOVEABSSP(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%f,",pfDisancelist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%f)",pfDisancelist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
//Description:    //�˶����޸Ľ���λ��  20130901 �Ժ�Ŀ������汾֧��
//Input:          //������handle
					��� iaxis
					���Ծ��� pfDisance
//Output:         //
//Return:         //������
/*************************************************************/
int32  ZAux_Direct_MoveModify(ZMC_HANDLE handle, int iaxis, float pfDisance)
{

	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];		
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

    //��������
	
	sprintf(cmdbuff, "MOVEMODIFY(%f) AXIS(%d)",pfDisance,iaxis);

	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���Բ�Ķ�Բ���岹�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶�����
				end2              �ڶ������˶�����
				centre1    ��һ�����˶�Բ�ģ��������ʼ�㡣
				centre2    �ڶ������˶�Բ�ģ��������ʼ�㡣
				direction  0-��ʱ�룬1-˳ʱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCirc(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection)
{
	int i;
	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
		
    //��������
	sprintf(tempbuff, "MOVECIRC(%f,%f,%f,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���Բ�Ķ�Բ�� �岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶�����
				end2              �ڶ������˶�����
				centre1    ��һ�����˶�Բ�ģ��������ʼ�㡣
				centre2    �ڶ������˶�Բ�ģ��������ʼ�㡣
				direction  0-��ʱ�룬1-˳ʱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCircSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection)
{

	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
			
    //��������
	sprintf(tempbuff, "MOVECIRCSP(%f,%f,%f,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //����Բ��Բ���岹�˶�  20130901 �Ժ�Ŀ������汾֧��  �޷�����Բ
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶����꣬����λ��
				end2              �ڶ������˶����꣬����λ��
				centre1    ��һ�����˶�Բ�ģ�����λ��
				centre2    �ڶ������˶�Բ�ģ�����λ��
				direction  0-��ʱ�룬1-˳ʱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCircAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection)
{

	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
    //��������
	sprintf(tempbuff, "MOVECIRCABS(%f,%f,%f,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����Բ��Բ���岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��  �޷�����Բ
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶����꣬����λ��
				end2              �ڶ������˶����꣬����λ��
				centre1    ��һ�����˶�Բ�ģ�����λ��
				centre2    �ڶ������˶�Բ�ģ�����λ��
				direction  0-��ʱ�룬1-˳ʱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCircAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection)
{
	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
    //��������
	sprintf(tempbuff, "MOVECIRCABSSP(%f,%f,%f,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}



/*************************************************************
Description:    //���3�㶨Բ���岹�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				mid1       ��һ�����м�㣬�����ʼ�����
				mid2       �ڶ������м�㣬�����ʼ�����
				end1              ��һ��������㣬�����ʼ�����
				end2              �ڶ���������㣬�����ʼ�����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCirc2(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2)
{

	int i ;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
		
    //��������
	sprintf(tempbuff, "MOVECIRC2(%f,%f,%f,%f)", fmid1,fmid2,fend1,fend2);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //����3�㶨Բ���岹�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				mid1       ��һ�����м�㣬����λ��
				mid2       �ڶ������м�㣬����λ��
				end1              ��һ��������㣬����λ��
				end2              �ڶ���������㣬����λ�� 
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCirc2Abs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2)
{

	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
			
    //��������
	sprintf(tempbuff, "MOVECIRC2ABS(%f,%f,%f,%f)", fmid1,fmid2,fend1,fend2);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���3�㶨Բ���岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				mid1       ��һ�����м�㣬�����ʼ�����
				mid2       �ڶ������м�㣬�����ʼ�����
				end1              ��һ��������㣬�����ʼ�����
				end2              �ڶ���������㣬�����ʼ�����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCirc2Sp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MOVECIRC2SP(%f,%f,%f,%f)", fmid1,fmid2,fend1,fend2);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //����3�㶨Բ���岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				mid1       ��һ�����м�㣬����λ��
				mid2       �ڶ������м�㣬����λ��
				end1              ��һ��������㣬����λ��
				end2              �ڶ���������㣬����λ�� 
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveCirc2AbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
			
    //��������
	sprintf(tempbuff, "MOVECIRC2ABSSP(%f,%f,%f,%f)", fmid1,fmid2,fend1,fend2);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}



/*************************************************************
Description:    //���3��Բ�������岹�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶�����
				end2              �ڶ������˶�����
				centre1    ��һ�����˶�Բ�ģ��������ʼ��
				centre2    �ڶ������˶�Բ�ģ��������ʼ��		
				direction  0-��ʱ�룬1-˳ʱ��
				distance3���������˶����롣
				mode      ��������ٶȼ���:0(ȱʡ)����������ٶȼ��㡣1�����᲻�����ٶȼ��㡣
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelical(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MHELICAL(%f,%f,%f,%f,%d,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection, fDistance3, imode);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����3��Բ�������岹�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶�����
				end2              �ڶ������˶�����
				centre1    ��һ�����˶�Բ������
				centre2    �ڶ������˶�Բ������
				direction  0-��ʱ�룬1-˳ʱ��
				distance3���������˶����롣
				mode      ��������ٶȼ���:0(ȱʡ) ����������ٶȼ��㡣1�����᲻�����ٶȼ��㡣
Output:         //
 Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelicalAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
		
    //��������
	sprintf(tempbuff, "MHELICALABS(%f,%f,%f,%f,%d,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection, fDistance3, imode);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	//return ZMC_ExecuteNoAck(handle, cmdbuff, g_ZMC_MaxExcuteWaitms);
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���3��Բ�������岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶�����
				end2              �ڶ������˶�����
				centre1    ��һ�����˶�Բ�ģ��������ʼ��	  
				centre2    �ڶ������˶�Բ�ģ��������ʼ��		
				direction  0-��ʱ�룬1-˳ʱ��  
				distance3���������˶����롣
			  mode      ��������ٶȼ���: 0(ȱʡ)����������ٶȼ��㡣 1�����᲻�����ٶȼ��㡣
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelicalSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
			
    //��������
	sprintf(tempbuff, "MHELICALSP(%f,%f,%f,%f,%d,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection, fDistance3, imode);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����3��Բ�������岹SP�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��һ�����˶�����
				end2              �ڶ������˶�����
				centre1    ��һ�����˶�Բ������
				centre2    �ڶ������˶�Բ������
				direction  0-��ʱ�룬1-˳ʱ��
				distance3���������˶����롣
				mode      ��������ٶȼ���:0(ȱʡ) ����������ٶȼ��㡣1�����᲻�����ٶȼ��㡣
Output:         //
 Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelicalAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MHELICALABSSP(%f,%f,%f,%f,%d,%f,%d)", fend1,fend2,fcenter1,fcenter2,idirection, fDistance3, imode);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //���3�� 3�㻭�����岹�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				mid1       ��һ�����м��  
				mid2       �ڶ������м��	
				end1              ��һ���������	  
				end2              �ڶ����������		
				distance3���������˶�����		  
				mode      ��������ٶȼ���:			
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelical2(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
					
    //��������
	sprintf(tempbuff, "MHELICAL2(%f,%f,%f,%f,%f,%d)", fmid1,fmid2,fend1,fend2, fDistance3, imode);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����3�� 3�㻭�����岹�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
			imaxaxises			�����˶�������
			piAxislist			����б�
			mid1       ��һ�����м��
			mid2       �ڶ������м��
			end1              ��һ���������
			end2              �ڶ����������
			distance3   ���������˶�������
			mode      ��������ٶȼ���:
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelical2Abs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MHELICAL2ABS(%f,%f,%f,%f,%f,%d)", fmid1,fmid2,fend1,fend2, fDistance3, imode);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���3�� 3�㻭�����岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				mid1       ��һ�����м��  
				mid2       �ڶ������м��	
				end1              ��һ���������	  
				end2              �ڶ����������		
				distance3���������˶�����		  
				mode      ��������ٶȼ���:			
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelical2Sp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
					
    //��������
	sprintf(tempbuff, "MHELICAL2SP(%f,%f,%f,%f,%f,%d)", fmid1,fmid2,fend1,fend2, fDistance3, imode);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����3�� 3�㻭�����岹SP�˶�  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
			imaxaxises			�����˶�������
			piAxislist			����б�
			mid1       ��һ�����м��
			mid2       �ڶ������м��
			end1              ��һ���������
			end2              �ڶ����������
			distance3   ���������˶�������
			mode      ��������ٶȼ���:
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MHelical2AbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
						
    //��������
	sprintf(tempbuff, "MHELICAL2ABSSP(%f,%f,%f,%f,%f,%d)", fmid1,fmid2,fend1,fend2, fDistance3, imode);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}



/*************************************************************
Description:    //�����Բ�岹 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
			imaxaxises			�����˶�������
			piAxislist			����б�
			fend1              �յ��һ�����˶����꣬�������ʼ�㡣
			fend2              �յ�ڶ������˶����꣬�������ʼ�㡣
			fcenter1    ���ĵ�һ�����˶����꣬�������ʼ�㡣
			fcenter2    ���ĵڶ������˶����꣬�������ʼ�㡣
			idirection  0-��ʱ�룬1-˳ʱ��
			fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
			fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipse(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
						
    //��������
	sprintf(tempbuff, "MECLIPSE(%f,%f,%f,%f,%d,%f,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //������Բ�岹 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				fend1              �յ��һ�����˶�����
				fend2              �յ�ڶ������˶�����
				fcenter1    ���ĵ�һ�����˶����ꡣ
				fcenter2    ���ĵڶ������˶����ꡣ
				idirection  0-��ʱ�룬1-˳ʱ��
				fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
				fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������

  Output:         //
  Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MECLIPSEABS(%f,%f,%f,%f,%d,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //�����Բ�岹SP�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
			imaxaxises			�����˶�������
			piAxislist			����б�
			fend1              �յ��һ�����˶����꣬�������ʼ�㡣
			fend2              �յ�ڶ������˶����꣬�������ʼ�㡣
			fcenter1    ���ĵ�һ�����˶����꣬�������ʼ�㡣
			fcenter2    ���ĵڶ������˶����꣬�������ʼ�㡣
			idirection  0-��ʱ�룬1-˳ʱ��
			fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
			fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis)
{
	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
					
    //��������
	sprintf(tempbuff, "MECLIPSESP(%f,%f,%f,%f,%d,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //������Բ�岹SP�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				fend1              �յ��һ�����˶�����
				fend2              �յ�ڶ������˶�����
				fcenter1    ���ĵ�һ�����˶����ꡣ
				fcenter2    ���ĵڶ������˶����ꡣ
				idirection  0-��ʱ�룬1-˳ʱ��
				fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
				fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������

  Output:         //
  Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
						
    //��������
	sprintf(tempbuff, "MECLIPSEABSSP(%f,%f,%f,%f,%d,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //��� ��Բ + �����岹�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				fend1              �յ��һ�����˶����꣬�������ʼ�㡣
				fend2              �յ�ڶ������˶����꣬�������ʼ�㡣
				fcenter1    ���ĵ�һ�����˶����꣬�������ʼ�㡣
				fcenter2    ���ĵڶ������˶����꣬�������ʼ�㡣
				idirection  0-��ʱ�룬1-˳ʱ��
				fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
				fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������
				fDistance3	����������˶�����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseHelical(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis, float fDistance3)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
						
    //��������
	sprintf(tempbuff, "MECLIPSE(%f,%f,%f,%f,%d,%f,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis, fDistance3);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //������Բ + �����岹�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				fend1              �յ��һ�����˶�����
				fend2              �յ�ڶ������˶�����
				fcenter1    ���ĵ�һ�����˶����ꡣ
				fcenter2    ���ĵڶ������˶����ꡣ
				idirection  0-��ʱ�룬1-˳ʱ��
				fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
				fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������
				fDistance3	����������˶�����
  Output:         //
  Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseHelicalAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis,float fDistance3)
{
	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
	//��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MECLIPSEABS(%f,%f,%f,%f,%d,%f,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis,fDistance3);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��� ��Բ + �����岹SP�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				fend1              �յ��һ�����˶����꣬�������ʼ�㡣
				fend2              �յ�ڶ������˶����꣬�������ʼ�㡣
				fcenter1    ���ĵ�һ�����˶����꣬�������ʼ�㡣
				fcenter2    ���ĵڶ������˶����꣬�������ʼ�㡣
				idirection  0-��ʱ�룬1-˳ʱ��
				fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
				fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������
				fDistance3	����������˶�����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseHelicalSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis, float fDistance3)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MECLIPSESP(%f,%f,%f,%f,%d,%f,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis, fDistance3);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //������Բ + �����岹SP�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				fend1              �յ��һ�����˶�����
				fend2              �յ�ڶ������˶�����
				fcenter1    ���ĵ�һ�����˶����ꡣ
				fcenter2    ���ĵڶ������˶����ꡣ
				idirection  0-��ʱ�룬1-˳ʱ��
				fADis         ��һ�����Բ�뾶���볤����߰���ᶼ�ɡ�
				fBDis        �ڶ������Բ�뾶���볤����߰���ᶼ�ɣ�AB���ʱ�Զ�ΪԲ����������
				fDistance3	����������˶�����
  Output:         //
  Return:         //������
*************************************************************/
int32  ZAux_Direct_MEclipseHelicalAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis,float fDistance3)
{
	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MECLIPSEABSSP(%f,%f,%f,%f,%d,%f,%f,%f)", fend1,fend2,fcenter1,fcenter2,idirection, fADis, fBDis,fDistance3);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //�ռ�Բ�� + �����岹�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��1�����˶��������1	��������
				end2              ��2�����˶��������1	��������  
				end3              ��3�����˶��������1	��������
				centre1    ��1�����˶��������2	��������
				centre2    ��2�����˶��������2	��������
				centre3    ��3�����˶��������2 ��������
				mode      ָ��ǰ�����������
					0 ��ǰ�㣬�м�㣬�յ����㶨Բ�����������1Ϊ�յ���룬�������2Ϊ�м����롣
					1 ����С��Բ�����������1Ϊ�յ���룬�������2ΪԲ�ĵľ��롣
					2 ��ǰ�㣬�м�㣬�յ����㶨Բ���������1Ϊ�յ���룬�������2Ϊ�м����롣
					3 ������С��Բ�����ټ���������Բ���������1Ϊ�յ���룬�������2ΪԲ�ĵľ��롣
				fcenter4	��4�����˶�������� 
				fcenter5	��5�����˶��������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MSpherical(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fend3, float fcenter1, float fcenter2, float fcenter3, int imode, float fcenter4, float fcenter5)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MSPHERICAL(%f,%f,%f,%f,%f,%f,%d,%f,%f)", fend1,fend2,fend3,fcenter1,fcenter2,fcenter3,imode,fcenter4,fcenter5);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //�ռ�Բ�� + ���� �岹SP�˶� 20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��1�����˶��������1	��������
				end2              ��2�����˶��������1	��������  
				end3              ��3�����˶��������1	��������
				centre1    ��1�����˶��������2	��������
				centre2    ��2�����˶��������2	��������
				centre3    ��3�����˶��������2 ��������
				mode      ָ��ǰ�����������
					0 ��ǰ�㣬�м�㣬�յ����㶨Բ�����������1Ϊ�յ���룬�������2Ϊ�м����롣
					1 ����С��Բ�����������1Ϊ�յ���룬�������2ΪԲ�ĵľ��롣
					2 ��ǰ�㣬�м�㣬�յ����㶨Բ���������1Ϊ�յ���룬�������2Ϊ�м����롣
					3 ������С��Բ�����ټ���������Բ���������1Ϊ�յ���룬�������2ΪԲ�ĵľ��롣
				fcenter4	��4�����˶�������� 
				fcenter5	��5�����˶��������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MSphericalSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fend3, float fcenter1, float fcenter2, float fcenter3, int imode, float fcenter4, float fcenter5)
{
	int i;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MSPHERICALSP(%f,%f,%f,%f,%f,%f,%d,%f,%f)", fend1,fend2,fend3,fcenter1,fcenter2,fcenter3,imode, fcenter4, fcenter5);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //������Բ���岹�˶�������ƶ���ʽ������ʼ�뾶0ֱ����ɢʱ��0�Ƕȿ�ʼ
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				centre1: ��1��Բ�ĵ���Ծ���
				centre2: ��2��Բ�ĵ���Ծ���
				circles:  Ҫ��ת��Ȧ��������ΪС��Ȧ��������ʾ˳ʱ��.
				pitch:   ÿȦ����ɢ���룬����Ϊ����
				distance3        ��3�������Ĺ��ܣ�ָ����3�����Ծ��룬���᲻�����ٶȼ��㡣
				distance4        ��4�������Ĺ��ܣ�ָ����4�����Ծ��룬���᲻�����ٶȼ��㡣
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveSpiral(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float centre1, float centre2, float circles, float pitch, float distance3, float distance4)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
				
    //��������
	sprintf(tempbuff, "MOVESPIRAL(%f,%f,%f,%f,%f,%f)", centre1,centre2,circles,pitch,distance3,distance4);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //������Բ���岹SP�˶�������ƶ���ʽ������ʼ�뾶0ֱ����ɢʱ��0�Ƕȿ�ʼ
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				centre1: ��1��Բ�ĵ���Ծ���
				centre2: ��2��Բ�ĵ���Ծ���
				circles:  Ҫ��ת��Ȧ��������ΪС��Ȧ��������ʾ˳ʱ��.
				pitch:   ÿȦ����ɢ���룬����Ϊ����
				distance3        ��3�������Ĺ��ܣ�ָ����3�����Ծ��룬���᲻�����ٶȼ��㡣
				distance4        ��4�������Ĺ��ܣ�ָ����4�����Ծ��룬���᲻�����ٶȼ��㡣
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveSpiralSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float centre1, float centre2, float circles, float pitch, float distance3, float distance4)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
					
    //��������
	sprintf(tempbuff, "MOVESPIRALSP(%f,%f,%f,%f,%f,%f)", centre1,centre2,circles,pitch,distance3,distance4);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //�ռ�ֱ�߲岹�˶���������һ��ֱ���˶��ľ��������ڹս��Զ�����Բ��������Բ�����ʹ���˶����յ���ֱ�ߵ��յ㲻һ�£��սǹ���ʱ�������Բ���������벻��ʱ���Զ���С�뾶
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��1�����˶���������
				end2              ��2�����˶���������
				end3              ��3�����˶���������
				next1      ��1������һ��ֱ���˶���������
				next2      ��2������һ��ֱ���˶���������
				next3      ��3������һ��ֱ���˶���������
				radius      ����Բ���İ뾶���������ʱ���Զ���С��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveSmooth(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float end1, float end2, float end3, float next1, float next2, float next3, float radius)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
					
    //��������
	sprintf(tempbuff, "MOVESMOOTH(%f,%f,%f,%f,%f,%f,%f)", end1,end2,end3,next1,next2,next3,radius);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //�ռ�ֱ�߲岹SP�˶���������һ��ֱ���˶��ľ��������ڹս��Զ�����Բ��������Բ�����ʹ���˶����յ���ֱ�ߵ��յ㲻һ�£��սǹ���ʱ�������Բ���������벻��ʱ���Զ���С�뾶
Input:          //������handle
				imaxaxises			�����˶�������
				piAxislist			����б�
				end1              ��1�����˶���������
				end2              ��2�����˶���������
				end3              ��3�����˶���������
				next1      ��1������һ��ֱ���˶���������
				next2      ��2������һ��ֱ���˶���������
				next3      ��3������һ��ֱ���˶���������
				radius      ����Բ���İ뾶���������ʱ���Զ���С��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveSmoothSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float end1, float end2, float end3, float next1, float next2, float next3, float radius)
{

	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
					
    //��������
	sprintf(tempbuff, "MOVESMOOTHSP(%f,%f,%f,%f,%f,%f,%f)", end1,end2,end3,next1,next2,next3,radius);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //�˶���ͣ		���岹�˶���ͣ���ᡣ���б����һ����
Input:          //������handle  
				��� iaxis
				ģʽ imode	0��ȱʡ�� ��ͣ��ǰ�˶��� 
							1 �ڵ�ǰ�˶���ɺ���׼��ִ����һ���˶�ָ��ʱ��ͣ�� 
							2 �ڵ�ǰ�˶���ɺ���׼��ִ����һ���˶�ָ��ʱ����������ָ���MARK��ʶ��һ��ʱ��ͣ�����ģʽ��������һ�������ɶ��ָ����ʵ��ʱ��������һ����������ɺ���ͣ�� 
  
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MovePause(ZMC_HANDLE handle,int iaxis, int imode)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "MOVE_PAUSE(%d) axis(%d)", imode,iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //ȡ���˶���ͣ
Input:          //������handle  
					��� iaxis
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveResume(ZMC_HANDLE handle,int iaxis)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "MOVE_RESUME AXIS(%d)",iaxis);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //�ڵ�ǰ���˶�ĩβλ�������ٶ����ƣ�����ǿ�ƹսǼ���
Input:          //������handle  
				 ��� iaxis
Output:         //
 Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveLimit(ZMC_HANDLE handle, int iaxis,float limitspeed)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "MOVELIMIT(%f) AXIS(%d)", limitspeed,iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //���˶������м������ָ��
Input:          //������handle  
				��� iaxis
				����ڱ�� ioutnum
				�����״̬	ivalue
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveOp(ZMC_HANDLE handle, int iaxis,int ioutnum, int ivalue)
{

	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
   //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);
				
    //��������
	sprintf(tempbuff, "MOVE_OP(%d,%d)", ioutnum, ivalue);
    strcat(cmdbuff, tempbuff);
		
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���˶������м���������������ָ��
Input:          //������handle  
				��� iaxis
				�������ʼ��� ioutnumfirst
				����ڽ������ ioutnumend
				��Ӧ�����״̬���������ֵ	ivalue
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveOpMulti(ZMC_HANDLE handle, int iaxis, int ioutnumfirst, int ioutnumend, int ivalue)
{

	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);
		
    //��������
	sprintf(tempbuff, "MOVE_OP(%d,%d,%d)", ioutnumfirst, ioutnumend, ivalue);
    strcat(cmdbuff, tempbuff);	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //���˶������м������ָ�� ,ָ��ʱ������״̬��ת
Input:          //������handle  
				��� iaxis
				����ڱ�� ioutnum
				�����״̬	ivalue
				״̬��תʱ�� iofftimems
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveOp2(ZMC_HANDLE handle, int iaxis, int ioutnum, int ivalue, int iofftimems)
{

	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);
			
    //��������
	sprintf(tempbuff, "MOVE_OP2(%d,%d,%d)", ioutnum, ivalue, iofftimems);
    strcat(cmdbuff, tempbuff);		
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���˶������м���AOUT���ָ��
Input:          //������handle  
				��� iaxis
				DA�ڱ�� ioutnum
				ģ����ֵ fvalue ��4ϵ������ 12λ0-4095��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveAout(ZMC_HANDLE handle, int iaxis, int ioutnum, float fvalue)
{

	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);
				
    //��������
	sprintf(tempbuff, "MOVE_AOUT(%d,%f)", ioutnum, fvalue);
    strcat(cmdbuff, tempbuff);			
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���˶������м�����ʱָ��
Input:          //������handle  
				��� iaxis
				��ʱʱ�� itimems ����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveDelay(ZMC_HANDLE handle, int iaxis, int itimems)
{

	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);
				
    //��������
	sprintf(tempbuff, "MOVE_WA(%d)", itimems);
    strcat(cmdbuff, tempbuff);		
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��תֱ̨�߲岹�˶���  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				tablenum			�洢��ת̨������table��� 
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveTurnabs(ZMC_HANDLE handle, int tablenum ,int imaxaxises,int *piAxislist, float *pfDisancelist)
{
	int i;
	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	//
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
	sprintf(tempbuff, "MOVE_TURNABS(%d,",tablenum);
	strcat(cmdbuff, tempbuff);
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%f,",pfDisancelist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%f)",pfDisancelist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //��ת̨Բ��+�����岹�˶���  20130901 �Ժ�Ŀ������汾֧��
Input:          //������handle
				tablenum       �洢��ת������table���
				refpos1    ��һ����ο��㣬����λ��
				refpos2    �ڶ�����ο��㣬����λ��
				mode      1-�ο����ǵ�ǰ��ǰ�棬2-�ο����ǽ�������棬3-�ο������м䣬�������㶨Բ�ķ�ʽ��
				end1              ��һ��������㣬����λ��
				end2              �ڶ���������㣬����λ��
				imaxaxises        �����˶�������
				piAxislist		  ���б�
				pfDisancelist	����������б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_McircTurnabs(ZMC_HANDLE handle, int tablenum ,float refpos1,float refpos2,int mode,float end1,float end2,int imaxaxises,int *piAxislist ,float *pfDisancelist)
{
	int i;
	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	//
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
  //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
		
    //��������
	sprintf(tempbuff, "MCIRC_TURNABS(%d,%f,%f,%d,%f,%f,",tablenum,refpos1,refpos2,mode,end1,end2);
	strcat(cmdbuff, tempbuff);
	
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%f,",pfDisancelist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%f)",pfDisancelist[imaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����͹�� ͬ���˶�
Input:          //������handle  
				iaxis			���
				istartpoint		��ʼ��TABLE���
				iendpoint		������TABLE���
				ftablemulti		λ�ñ�����һ����Ϊ���嵱��ֵ
				fDistance		�ο��˶��ľ��룬�����������˶�ʱ��
 Output:         //
 Return:         //������
*************************************************************/
int32  ZAux_Direct_Cam(ZMC_HANDLE handle,int iaxis, int istartpoint, int iendpoint, float ftablemulti, float fDistance)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "CAM(%d,%d,%f,%f) AXIS(%d)", istartpoint,iendpoint,ftablemulti,fDistance,iaxis);

	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����͹�� ͬ���˶�
Input:          //������handle  
				iaxis			���
				istartpoint		��ʼ��TABLE���
				iendpoint		������TABLE���
				ftablemulti		λ�ñ�����һ����Ϊ���嵱��ֵ
				fDistance		�ο��˶��ľ���
				ilinkaxis		�ο�����
				ioption			�ο�������ӷ�ʽ
				flinkstartpos	ioption�����о������
 Output:         //
 Return:         //������
*************************************************************/
int32  ZAux_Direct_Cambox(ZMC_HANDLE handle, int iaxis,int istartpoint, int iendpoint, float ftablemulti, float fDistance, int ilinkaxis, int ioption, float flinkstartpos)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "CAMBOX(%d,%d,%f,%f,%d,%d,%f) AXIS(%d)", istartpoint,iendpoint,ftablemulti,fDistance, ilinkaxis, ioption, flinkstartpos,iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //����͹�� ͬ���˶�
Input:          //������handle  
				iaxis			�����˶������(������)
				fDistance		ͬ�����̸������˶�����
				fLinkDis		ͬ�����̲ο���(����)�˶����Ծ���
				fLinkAcc		��������ٽ׶Σ��ο����ƶ��ľ��Ծ���
				fLinkDec		��������ٽ׶Σ��ο����ƶ��ľ��Ծ���
				iLinkaxis		�ο�������
				ioption			����ģʽѡ��
				flinkstartpos	����ģʽѡ�����˶�����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Movelink(ZMC_HANDLE handle, int iaxis,float fDistance, float fLinkDis, float fLinkAcc, float fLinkDec,int iLinkaxis, int ioption, float flinkstartpos)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "MOVELINK(%f,%f,%f,%f,%d,%d,%f) AXIS(%d)", fDistance, fLinkDis, fLinkAcc,fLinkDec,iLinkaxis,ioption,flinkstartpos,iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����͹�� ͬ���˶�
Input:          //������handle  
				iaxis			�����˶������(������)
				fDistance		ͬ�����̸������˶�����
				fLinkDis		ͬ�����̲ο���(����)�˶����Ծ���
				startsp			����ʱ������Ͳο�����ٶȱ�����units/units��λ��������ʾ�����Ḻ���˶�
				endsp			����ʱ������Ͳο�����ٶȱ�����units/units��λ, ������ʾ�����Ḻ���˶���
				iLinkaxis		�ο�������
				ioption			����ģʽѡ��
				flinkstartpos	����ģʽѡ�����˶�����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Moveslink(ZMC_HANDLE handle, int iaxis,float fDistance, float fLinkDis, float startsp, float endsp,int iLinkaxis, int ioption, float flinkstartpos)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "MOVESLINK(%f,%f,%f,%f,%d,%d,%f) AXIS(%d)", fDistance, fLinkDis, startsp,endsp,iLinkaxis,ioption,flinkstartpos,iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���� ͬ���˶�ָ�� ���ӳ���
Input:          //������handle  
				ratio		���ʣ������ɸ���ע������������ı�����	
				link_axis	���������ţ�����ʱΪ��������
				move_axis	�涯���
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Connect(ZMC_HANDLE handle, float ratio, int link_axis,int move_axis)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "CONNECT(%f,%d) AXIS(%d)", ratio, link_axis,move_axis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���� ͬ���˶�ָ�� ���ӳ��� ����ǰ���Ŀ��λ����link_axis��Ĳ岹ʸ������ͨ�����ӳ�������
Input:          //������handle  
				ratio		���ʣ������ɸ���ע������������ı�����	
				link_axis	���������ţ�����ʱΪ��������
				move_axis	�涯���
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Connpath(ZMC_HANDLE handle, float ratio, int link_axis,int move_axis)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "CONNPATH(%f,%d) AXIS(%d)", ratio, link_axis,move_axis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //λ������ָ��
Input:          //������handle  
				iaxis	���
				imode	����ģʽ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Regist(ZMC_HANDLE handle,int iaxis, int imode)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "REGIST(%d) AXIS(%d)", imode,iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //������������ֱȣ�ȱʡ(1,1)
Input:          //������handle  
				iaxis		���
				mpos_count	���ӣ���Ҫ����65535
				input_count	 ��ĸ����Ҫ����65535
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_EncoderRatio(ZMC_HANDLE handle, int iaxis,int mpos_count,int input_count)
{

	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);
					
    //��������
	sprintf(tempbuff, "ENCODER_RATIO(%d,%d)", mpos_count,input_count);
    strcat(cmdbuff, tempbuff);	
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //���ò���������ֱȣ�ȱʡ(1,1)
Input:          //������handle  
				iaxis		���
				mpos_count	���ӣ�1-65535
				input_count	 ��ĸ��1-65535
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_StepRatio(ZMC_HANDLE handle, int iaxis, int mpos_count,int input_count)
{

	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\r\n",iaxis);	
    //��������
	sprintf(tempbuff, "STEP_RATIO(%d,%d)", mpos_count,input_count);
    strcat(cmdbuff, tempbuff);		
	
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //����������ֹͣ
Input:          //������handle 
				imode ֹͣģʽ
				0��ȱʡ��ȡ����ǰ�˶�
				1	ȡ��������˶�
				2	ȡ����ǰ�˶��ͻ����˶���
				3	�����ж����巢�͡�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Rapidstop(ZMC_HANDLE handle, int imode)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
    //��������
	sprintf(cmdbuff, "RAPIDSTOP(%d)", imode);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //������˶�ֹͣ
Input:          //������handle  ��ţ� ����
				imaxaxises		����
				piAxislist		���б�
				imode	ģʽ
					0��ȱʡ��ȡ����ǰ�˶�
					1	ȡ��������˶�
					2	ȡ����ǰ�˶��ͻ����˶���
					3	�����ж����巢�͡�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_CancelAxisList(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, int imode)
{
	int i = 0;
	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	sprintf(cmdbuff, "CANCEL(%d) AXIS(%d)\n", imode, piAxislist[i]);

	for(i = 1; i< imaxaxises; i++)
	{
		//
		sprintf(tempbuff, "CANCEL(%d) AXIS(%d)\n", imode, piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}

	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //CONNFRAME��е�����ָ��	2ϵ�����Ͽ�����֧��
Input:          //������handle
				Jogmaxaxises	�ؽ�������
				JogAxislist		�ؽ����б�
				frame			��е������
				tablenum		��е�ֲ���TABLE��ʼ���
				Virmaxaxises	�������������
				VirAxislist		�������б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Connframe(ZMC_HANDLE handle,int Jogmaxaxises,int *JogAxislist, int frame, int tablenum , int Virmaxaxises , int *VirAxislist)
{
	int i;
	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	//
	if(0 > Jogmaxaxises || Jogmaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	if(0 > Virmaxaxises || Virmaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
    //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< Jogmaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",JogAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",JogAxislist[Jogmaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
	
    //��������

	sprintf(tempbuff, "CONNFRAME(%d,%d,",frame,tablenum);
	strcat(cmdbuff, tempbuff);
	
	for(i = 0; i< Virmaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",VirAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",VirAxislist[Virmaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //CONNREFRAME��е������ָ��	2ϵ�����Ͽ�����֧��
Input:          //������handle
				Virmaxaxises	�������������
				VirAxislist		�������б�
				frame			��е������
				tablenum		��е�ֲ���TABLE��ʼ���
				Jogmaxaxises	�ؽ�������
				JogAxislist		�ؽ����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Connreframe(ZMC_HANDLE handle,int Virmaxaxises , int *VirAxislist, int frame, int tablenum , int Jogmaxaxises,int *JogAxislist)
{
	int i;
	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	//
	//
	if(0 > Jogmaxaxises || Jogmaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	if(0 > Virmaxaxises || Virmaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
   //��������
	strcpy(cmdbuff, "BASE(");
	
	for(i = 0; i< Virmaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",VirAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",VirAxislist[Virmaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//����
	strcat(cmdbuff, "\n");
		

    //��������
	
	sprintf(tempbuff, "CONNREFRAME(%d,%d,",frame,tablenum);
	strcat(cmdbuff, tempbuff);
	
	for(i = 0; i< Jogmaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",JogAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	
	//
	sprintf(tempbuff, "%d)",JogAxislist[Jogmaxaxises-1]);
    strcat(cmdbuff, tempbuff);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}



/*********************************�����˶�****************************************************

/*************************************************************
Description:    //������˶�	iaddaxis�˶����ӵ�iaxis�� ��ADDAXָ����ӵ����������
Input:          //������handle  
				 iaxis		��������
				 iaddaxis	������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Single_Addax(ZMC_HANDLE handle, int iaxis, int iaddaxis)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "ADDAX(%d) AXIS(%d)", iaddaxis, iaxis);
	
	
	//��������ִ�к���
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
}


/*************************************************************
Description:    //�����˶�ֹͣ
Input:          //������handle  
				iaxis ���
				imode ģʽ
					0��ȱʡ��ȡ����ǰ�˶�
					1	ȡ��������˶�
					2	ȡ����ǰ�˶��ͻ����˶���
					3	�����ж����巢�͡�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Single_Cancel(ZMC_HANDLE handle, int iaxis, int imode)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "CANCEL(%d) AXIS(%d)", imode, iaxis);
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //���������˶�
Input:          //������handle  
				iaxis ���
				idir ���� 1���� -1����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Single_Vmove(ZMC_HANDLE handle, int iaxis, int idir)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "VMOVE(%d) AXIS(%d)", idir, iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //��������ʽ����
Input:          //������handle  
				iaxis	���
				imode	ģʽ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Single_Datum(ZMC_HANDLE handle, int iaxis, int imode)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "AXIS_STOPREASON(%d) = 0\r\nDATUM(%d) AXIS(%d)",iaxis,imode,iaxis);	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //�������״̬
Input:          //������handle  
				iaxis ���
Output:         //homestatus ������ɱ�־ 0-�����쳣 1����ɹ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetHomeStatus(ZMC_HANDLE handle,uint32 iaxis,uint32 * homestatus)
{

	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	

	if(NULL == homestatus || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	sprintf(cmdbuff, "?IDLE(%d),AXIS_STOPREASON(%d)",iaxis,iaxis,iaxis);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	int temparray[4];
	iresult = ZAux_TransStringtoInt(cmdbuffAck,2,&temparray[0]);
	int idlestatus = temparray[0];
	int stopstatus = temparray[1];

	
	if((idlestatus == -1)  && stopstatus == 0)		//ֹͣ��
	{
		*homestatus = 1;	//�������	
	}
	else
	{
		*homestatus = 0;	//����δ�ɹ�
	}
	return ERR_OK;
}

/*************************************************************
Description:    //��������˶�
Input:          //������handle  
				iaxis ���
				fdistance ����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Single_Move(ZMC_HANDLE handle, int iaxis, float fdistance)
{
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "MOVE(%f) AXIS(%d)", fdistance, iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��������˶�
Input:          //������handle  
				iaxis ���
				fdistance ����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_Single_MoveAbs(ZMC_HANDLE handle, int iaxis, float fdistance)
{

	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "MOVEABS(%f) AXIS(%d)", fdistance, iaxis);
	
	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}



/*********************�ڴ����
/*************************************************************
Description:    //дVR, 
Input:          //������handle 
				vrstartnum		VR��ʼ���
				numes			д�������
				pfValue			д��������б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetVrf(ZMC_HANDLE handle,int vrstartnum, int numes, float *pfValue)
{
	//int i;
	//int32 iresult;
	//
	//char  cmdbuff[2048];
	//char  cmdbuffAck[2048];
	//if(NULL == pfValue || numes < 0)
	//{
	//	return  ERR_AUX_PARAERR;
	//}
	//
	//for(i = 0; i< numes; i++)
	//{
	//	//
	//	sprintf(cmdbuff, "VR(%d) =  %f ", (i + vrstartnum) , pfValue[i]);
	//	//��������ִ�к���
	//	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	//	if(ERR_OK != iresult)
	//	{
	//		return iresult;
	//	}
	//}
	//
	//return ERR_OK;

	int i, icur, isend;
	int32 iresult;
	char  tempbuff[2048];
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue || numes < 0)
	{
		return  ERR_AUX_PARAERR;
	}


	isend = 0;
	while(1)
	{
		//һ�η��͸���40��
		icur = numes - isend;
		if(icur > 200)
		{
			icur = 200;
		}

		iresult = 	ZMC_RegisterWrite(handle,"VR",32,vrstartnum+isend,icur,(uint8*)(pfValue+isend));
		if(ERR_OK != iresult)
		{
			return iresult;
		}

		isend+= icur;
		if(isend >= numes)
		{
			break;
		}
	}
	return ERR_OK;
		
}


/*************************************************************
Description:    //VR��ȡ, ����һ�ζ�ȡ���
Input:          //������handle  
				vrstartnum	��ȡ��VR��ʼ��ַ
				numes		��ȡ������
Output:         //pfValue  ���صĶ�ȡֵ�����ʱ�������ռ�.
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetVrf(ZMC_HANDLE handle, int vrstartnum, int numes, float *pfValue)
{
	//int i, icur, isend;
	//int32 iresult;
	//char  tempbuff[2048];
	//char  cmdbuff[2048];
	//char  cmdbuffAck[2048];
	//
	//if(NULL == pfValue || numes < 0)
	//{
	//	return  ERR_AUX_PARAERR;
	//}
	//
	//isend = 0;
	//while(1)
	//{
	//	//һ�η��͸���15��
	//	icur = numes - isend;
	//	if(icur > 15)
	//	{
	//		icur = 15;
	//	}
	//	
	//	//��������
	//	strcpy(cmdbuff, "?");
	//	for(i = 0; i< icur; i++)
	//	{
	//		//
	//		sprintf(tempbuff, "vr(%d) ",vrstartnum+isend+i);
	//		strcat(cmdbuff, tempbuff);
	//	}
	//	
	//	//��������ִ�к���
	//	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	//	if(ERR_OK != iresult)
	//	{
	//		return iresult;
	//	}
	//	
	//	//
	//	if(0 == strlen(cmdbuffAck))
	//	{
	//		return ERR_NOACK;
	//	}
	//	
	//	//
	//	iresult = ZAux_TransStringtoFloat(cmdbuffAck, icur, pfValue+isend);
	//	if(ERR_OK != iresult)
	//	{
	//		return iresult;
	//	}
	//	
	//	isend+= icur;
	//	if(isend >= numes)
	//	{
	//		break;
	//	}
	//}
	//
	//
	//return ERR_OK;

	int i, icur, isend;
	int32 iresult;
	char  tempbuff[2048];
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue || numes < 0)
	{
		return  ERR_AUX_PARAERR;
	}


	isend = 0;
	while(1)
	{
		//һ�η��͸���40��
		icur = numes - isend;
		if(icur > 200)
		{
			icur = 200;
		}

		iresult = ZMC_RegisterRead(handle,"VR",32,vrstartnum+isend,icur,(uint8*)(pfValue+isend));
		if(ERR_OK != iresult)
		{
			return iresult;
		}

		isend+= icur;
		if(isend >= numes)
		{
			break;
		}
	}
	return ERR_OK;
}

/*************************************************************
Description:    //VRINT��ȡ�� ����150401���ϰ汾��֧��VRINT��DIRECTCOMMAND��ȡ
Input:          //������handle  
				vrstartnum	��ȡ��VR��ʼ��ַ
				numes		��ȡ������
				Output:         //piValue  ���صĶ�ȡֵ�����ʱ�������ռ�.
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetVrInt(ZMC_HANDLE handle, int vrstartnum, int numes, int *piValue)
{
	int i, icur, isend;
	int32 iresult;
	char  tempbuff[2048];
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	
	if(NULL == piValue || numes < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
	isend = 0;
	while(1)
	{
		//һ�η��͸���15��
		icur = numes - isend;
		if(icur > 15)
		{
			icur = 15;
		}
		
		//��������
		strcpy(cmdbuff, "?");
		for(i = 0; i< icur; i++)
		{
			//
			sprintf(tempbuff, "vr_int(%d) ",vrstartnum+isend+i);
			strcat(cmdbuff, tempbuff);
		}
		
		//��������ִ�к���
		iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		if(ERR_OK != iresult)
		{
			return iresult;
		}
		
		//
		if(0 == strlen(cmdbuffAck))
		{
			return ERR_NOACK;
		}
		
		//
		iresult = ZAux_TransStringtoInt(cmdbuffAck, icur, piValue+isend);
		if(ERR_OK != iresult)
		{
			return iresult;
		}
		
		isend+= icur;
		if(isend >= numes)
		{
			break;
		}
	}
	
	return ERR_OK;
}

/*************************************************************
Description:    //дtable 
Input:          //������handle 
				tabstart	д���TABLE��ʼ���
				numes		д�������
				pfValue		д�������ֵ
Output:         //	
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetTable(ZMC_HANDLE handle,int tabstart, int numes, float *pfValue)
{
	int i ,icur;
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	char  tempbuff[2048];
	
	if(NULL == pfValue || numes < 1)
	{
		return  ERR_AUX_PARAERR;
	}

	int isend = 0;
	while(1)
	{
		//һ�η��͸���15��
		icur = numes - isend;
		if(icur > 15)
		{
			icur = 15;
		}

		//��������
		sprintf(cmdbuff, "table(%d",tabstart+isend);
		for(i = 0; i< icur; i++)
		{
			//
			sprintf(tempbuff, ",%f",pfValue[isend+i]);
			strcat(cmdbuff, tempbuff);
		}
		sprintf(tempbuff, ")");
		strcat(cmdbuff, tempbuff);

		//��������ִ�к���
		iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		if(ERR_OK != iresult)
		{
			return iresult;
		}

		//
		isend+= icur;
		if(isend >= numes)
		{
			break;
		}
	}
	
	return ERR_OK;
	
	
}

/*************************************************************
Description:    //table��ȡ, ����һ�ζ�ȡ���
Input:          //������handle  
				tabstart	��ȡTABLE��ʼ��ַ
				numes		��ȡ������
Output:         //pfValue  ���ʱ�������ռ�.
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetTable(ZMC_HANDLE handle, int tabstart, int numes, float *pfValue)
{
	int i, icur, isend;
	int32 iresult;
	char  tempbuff[2048];
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == pfValue || numes < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
	
	isend = 0;
	while(1)
	{
		//һ�η��͸���15��
		icur = numes - isend;
		if(icur > 15)
		{
			icur = 15;
		}
		
		//��������
		strcpy(cmdbuff, "?");
		for(i = 0; i< icur; i++)
		{
			//
			sprintf(tempbuff, "TABLE(%d) ",tabstart+isend+i);
			strcat(cmdbuff, tempbuff);
		}
		
		//��������ִ�к���
		iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		if(ERR_OK != iresult)
		{
			return iresult;
		}
		
		//
		if(0 == strlen(cmdbuffAck))
		{
			return ERR_NOACK;
		}
		
		//
		iresult = ZAux_TransStringtoFloat(cmdbuffAck, icur, pfValue+isend);
		if(ERR_OK != iresult)
		{
			return iresult;
		}
		
		isend+= icur;
		if(isend >= numes)
		{
			break;
		}
	}
	
	
	return ERR_OK;
}



#if 0
//��������
#endif

/*************************************************************
Description:    //�ַ���תΪfloat
Input:          //������handle 
				pstringin ���ݵ��ַ���
				inumes   ת�����ݸ���
Output:         //pfvlaue ת��������
Return:         //������
*************************************************************/
int32  ZAux_TransStringtoFloat(const char* pstringin, int inumes,  float* pfvlaue)
{
    char *ptemp;
	
    ptemp = (char*)pstringin;
    while(' ' == *ptemp)
    {
        ptemp++;
    }
    
    if(!(isdigit(ptemp[0]) || ('-' == ptemp[0])))
    {
        return ERR_ACKERROR;
    }
	
    char *pstringnew = ptemp;
    
    //                
    for(int i =0; i < inumes;i++)
    {
		
        while((' ' == *pstringnew) || ('\t' == *pstringnew))
        {
            pstringnew++;
        }
        if(('\0' == pstringnew[0]) || ('\r' == pstringnew[0]) || ('\n' == pstringnew[0]) || !(isdigit(pstringnew[0]) || ('-' == pstringnew[0])))
        {
            break;
        }
		
        double dvalue = strtod(pstringnew, &ptemp);
        if((pstringnew == ptemp))
        {
            //break;
            return ERR_ACKERROR;
        }    
		
        pfvlaue[i] = dvalue;
        
        //�����ϴε�
        pstringnew = ptemp;
    }
	
	
    return ERR_OK;
}

/*************************************************************
Description:    //�ַ���תΪint
Input:          //������handle 
				pstringin ���ݵ��ַ���
				inumes   ת�����ݸ���
Output:         //pivlaue ת��������
Return:         //������
*************************************************************/
int32  ZAux_TransStringtoInt(const char* pstringin, int inumes,  int* pivlaue)
{
    char *ptemp;
	
    ptemp = (char*)pstringin;
    while(' ' == *ptemp)
    {
        ptemp++;
    }
    
    if(!(isdigit(ptemp[0]) || ('-' == ptemp[0])))
    {
        return ERR_ACKERROR;
    }
	
    char *pstringnew = ptemp;
    
    //               
    for(int i =0; i < inumes;i++)
    {
		
        while((' ' == *pstringnew) || ('\t' == *pstringnew))
        {
            pstringnew++;
        }
        if(('\0' == pstringnew[0]) || ('\r' == pstringnew[0]) || ('\n' == pstringnew[0]) || !(isdigit(pstringnew[0]) || ('-' == pstringnew[0])))
        {
            break;
        }
		
        double dvalue = strtod(pstringnew, &ptemp);
        if((pstringnew == ptemp))
        {
            //break;
            return ERR_ACKERROR;
        }    
		
        pivlaue[i] = dvalue;    //ת��������
        
        //�����ϴε�
        pstringnew = ptemp;
    }
	
	
    return ERR_OK;
}


#if 0
//U�̸�ʽ����غ���
#endif

/*************************************************************
Description:    //��float��ʽ�ı����б��洢���ļ��� ���������U���ļ���ʽһ��.
Input:          //sFilename �ļ�����·��
				pVarlist	д��������б�
				inum		���ݵĳ���
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_WriteUFile(const char *sFilename, float *pVarlist, int inum)
{
    FILE *pfile;
	
    //д���ļ�
    pfile = fopen(sFilename, "wb");
    if(NULL == pfile)
    {
        //
        ZAUX_ERROR("open file:%s err", sFilename);
        return ERR_AUX_FILE_ERROR;
    }
	
    fseek(pfile, 0, SEEK_SET);
	
	if(fwrite(pVarlist, 4, inum, pfile) != inum)
	{
        //
        ZAUX_ERROR("fwrite size != %d", inum);
        fclose(pfile);
        return ERR_AUX_OS_ERR;
	}
	
	//�ر��ļ�
	fclose(pfile);
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡfloat��ʽ�ı����б��� ���������U���ļ���ʽһ��.
Input:          //sFilename �ļ�����·��
				inum		���ݵĳ���	
Output:         //pVarlist	��ȡ�������б�
Return:         //������
*************************************************************/
int32  ZAux_ReadUFile(const char *sFilename, float *pVarlist, int* pinum)
{
    FILE *pfile;
    uint32 uifilesize;
	
    //��ȡ�ļ�
    pfile = fopen(sFilename, "rb");
    if(NULL == pfile)
    {
        //
        ZAUX_ERROR("open file:%s err", sFilename);
        return ERR_AUX_FILE_ERROR;
    }
    fseek(pfile, 0, SEEK_END);
	
    uifilesize = ftell(pfile);
	
    fseek(pfile, 0, SEEK_SET);	
	if(fread(pVarlist, 1, uifilesize, pfile) != uifilesize)
	{
        //
        ZAUX_ERROR("read size != %d", uifilesize);
        fclose(pfile);
		
        return ERR_AUX_OS_ERR;
	}
	//�ر��ļ�
	fclose(pfile);
	
	*pinum = uifilesize/4;
	return ERR_OK;
}



/*************************************************************
Description:    //modbus�Ĵ������� modbus_bit
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
				pdata ���õ�λ״̬  ��λ�洢
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Set0x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint8* pdata)
{
	int32 iresult;
	iresult = ZMC_Modbus_Set0x(handle,start,inum,pdata);
	
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ������� modbus_bit
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
Output:         //pdata ���ص�λ״̬  ��λ�洢
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Get0x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint8* pdata)
{
	int32 iresult;
	iresult = ZMC_Modbus_Get0x(handle,start,inum, pdata);
	
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ�������		MODBUS_REG
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
				pdata	����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Set4x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint16* pdata)
{
	int32 iresult;
	iresult = ZMC_Modbus_Set4x(handle,start,inum, pdata);
	
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ������� MODBUS_REG
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
Output:         //pdata	��ȡ��REG�Ĵ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Get4x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint16* pdata)
{
	int32 iresult;
	iresult = ZMC_Modbus_Get4x(handle, start, inum, pdata);
	
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ�������		MODBUS_IEEE
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
				pfdata	����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Get4x_Float(ZMC_HANDLE handle, uint16 start, uint16 inum, float* pfdata)
{
	int 	i,j;
	int32 	iresult;
	void * pf;
	uint16 * pi;
	uint16  tempdata[2048];
	
	if(NULL == pfdata || inum < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
	
	pi = tempdata;
	pf = pfdata;
	
	iresult = ZMC_Modbus_Get4x(handle,start,inum*2,tempdata);
	if(iresult == 0)
	{
		
		for(i=1;i<inum +1;i++)
		{
			for(j=0;j<2;j++)
			{
				*((uint16 *)pf + j )  = *(pi +j);
			}
			pf = pfdata +i;
			pi = pi+2;
		}
	}
	
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ������� MODBUS_IEEE
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
Output:         //pfdata	��ȡ��REG�Ĵ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Set4x_Float(ZMC_HANDLE handle, uint16 start, uint16 inum, float* pfdata)
{
	int 	i,j;
	int32 	iresult;
	void * pf;
	uint16 * pi;
	uint16  tempdata[2048];
	
	if(NULL == pfdata || inum < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
	pi = tempdata;
	pf = pfdata;
	
	for(i=1;i<inum +1;i++)
	{
		for(j=0;j<2;j++)
		{
			*(pi +j) = *((uint16 *)pf + j);
		}
		pf = pfdata +i;
		pi = pi+2;
	}	
	iresult = ZMC_Modbus_Set4x(handle,start,inum*2,tempdata);
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ�������		MODBUS_LONG
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
				pidata	����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Get4x_Long(ZMC_HANDLE handle, uint16 start, uint16 inum, int32 * pidata)
{
	int 	i,j;
	int32 	iresult;
	void * pitemp;
	uint16 * pi;
	uint16  tempdata[2048];
	
	if(NULL == pidata || inum < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
	pi = tempdata;
	pitemp = pidata;
	
	iresult = ZMC_Modbus_Get4x(handle,start,inum*2,tempdata);
	if(iresult == 0)
	{
		
		for(i=1;i<inum +1;i++)
		{
			for(j=0;j<2;j++)
			{
				*((uint16 *)pitemp + j )  = *(pi +j);
			}
			pitemp = pidata +i;
			pi = pi+2;
		}
	}
	
	return iresult;
}

/*************************************************************
Description:    //modbus�Ĵ������� MODBUS_LONG
Input:          //������handle �Ĵ�����ַ
				start	��ʼ���
				inum	����
Output:         //pidata	��ȡ��REG�Ĵ���ֵ
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Set4x_Long(ZMC_HANDLE handle, uint16 start, uint16 inum, int32 * pidata)
{
	int 	i,j;
	int32 	iresult;
	void * pitemp;
	uint16 * pi;
	uint16  tempdata[2048];
	
	if(NULL == pidata || inum < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
	pi = tempdata;
	pitemp = pidata;
	
	for(i=1;i<inum +1;i++)
	{
		for(j=0;j<2;j++)
		{
			*(pi +j) = *((uint16 *)pitemp + j);
		}
		pitemp = pidata +i;
		pi = pi+2;
	}	
	iresult = ZMC_Modbus_Set4x(handle,start,inum*2,tempdata);
	return iresult;
}

/*************************************************************
Description:    //��ȡmodbus_string
Input:          //������handle
				start	modbus��ʼ��ַ
				inum	����			
Output:         pidata	��ȡ���ص��ַ���
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Get4x_String(ZMC_HANDLE handle, uint16 start, uint16 inum, char * pidata)
{
	int 	i;
	int32 	iresult;
	uint16  tempdata[2048];
	uint16 charnum;

	if(NULL == pidata || inum < 0 || inum > 2048 )
	{
		return  ERR_AUX_PARAERR;
	}
	
	charnum = (inum - 1)/2 + 1;

	iresult = ZMC_Modbus_Get4x(handle,start,charnum,tempdata);
	if(iresult == 0)
	{	
		for(i=0;i<inum ;i++)
		{
			if(i%2 == 0)
			{
				*(pidata + i) = (char)(tempdata[i/2]);
			}
			else
			{
				*(pidata + i) = (char)(tempdata[i/2]>>8);
			}
		}
	}
	
	return iresult;
}

/*************************************************************
Description:    //����modbus_string
Input:          //������handle
				start	modbus��ʼ��ַ
				inum	����
				pidata	д����ַ���
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Modbus_Set4x_String(ZMC_HANDLE handle, uint16 start, uint16 inum, char * pidata)
{
	int 	i;
	int32 	iresult;
	uint16  tempdata[2048];
	uint16 charnum;

	if(NULL == pidata || inum < 0 || inum > 2048 )
	{
		return  ERR_AUX_PARAERR;
	}

	for(i=0;i<inum ;i++)
		{
			if(i%2 == 0)
			{
				tempdata[i/2] =(uint16)(*(pidata + i));
			}
			else
			{
				tempdata[i/2] =(uint16)(*(pidata + i)) * 256 + tempdata[i/2];
			}
		}
	
	charnum = (inum - 1)/2 + 1;

	iresult = ZMC_Modbus_Set4x(handle,start,charnum,tempdata);

	return iresult;
}



/*************************************************************
Description:    //д�û�flash��, float����
Input:          //������handle
				uiflashid 	flash���
				uinumes		��������
				pfvlue		����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_FlashWritef(ZMC_HANDLE handle, uint16 uiflashid, uint32 uinumes, float *pfvlue)
{
	int32 iresult;
	iresult = ZMC_FlashWritef(handle, uiflashid, uinumes,pfvlue);
	
	return iresult;
}


/*************************************************************
Description:    //��ȡ�û�flash��, float����
Input:          //������handle
uiflashid 	flash���
uibuffnum	�����������
Output:         //
puinumesread ��ȡ���ı�������
Return:         //������
*************************************************************/
int32  ZAux_FlashReadf(ZMC_HANDLE handle, uint16 uiflashid, uint32 uibuffnum, float *pfvlue, uint32* puinumesread)
{
	int32 iresult;
	iresult = ZMC_FlashReadf(handle, uiflashid, uibuffnum,pfvlue,puinumesread);
	return iresult;
}


/***********************************2018-08-24 V2.1��������****************************************************************************
���Ӳ������⹦�ܺ���
����������غ���
***************************************************************************************************************************************/

/*************************************************************
Description:    //ʾ������������ 150723�Ժ�汾֧��
Input:          //������handle  
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Trigger(ZMC_HANDLE handle)
{

	//��������ִ�к���
		char  cmdbuffAck[2048];
		return ZAux_Execute(handle, "Trigger", cmdbuffAck, 2048);
}


//MOVE_PARA,MOVE_PWM, MOVE_SYNMOVE,MOVE_ASYNMOVE
/*************************************************************
Description:    //�˶����޸Ĳ���. 20170503���Ϲ̼�֧��
Input:          //������handle  
				base_axis �˶�����
				paraname  �޸ĵ�������ַ�������
				iaxis	  �����޸����
				fvalue	  ����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MovePara(ZMC_HANDLE handle,uint32 base_axis,char *  paraname,uint32 iaxis,float fvalue)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX || 0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_PARA(%s,%d,%f) axis(%d)",paraname,iaxis,fvalue,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�˶����޸�PWM 20170503���Ϲ̼�֧��
Input:          //������handle  
				base_axis	�岹������
				pwm_num		PWM�ڱ��
				pwm_duty	ռ�ձ�0-1
				pwm_freq	Ƶ�� Ӳ��PWM 1M �� ����PWM  2k 
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MovePwm(ZMC_HANDLE handle,uint32 base_axis,uint32 pwm_num,float pwm_duty,float pwm_freq)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_PWM(%d,%f,%f) axis(%d)",pwm_num,pwm_duty,pwm_freq,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�˶���ͬ����������˶�,. 20170503���Ϲ̼�֧��
Input:          //������handle
				base_axis �˶�����
				iaxis	  ͬ�����
				fdist	  ͬ�����˶�����
				ifsp	  �Ƿ�ʹ��SP�˶�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveSynmove(ZMC_HANDLE handle,uint32 base_axis,uint32 iaxis,float fdist,uint32 ifsp)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX || 0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_SYNMOVE(%d,%f,%d) axis(%d)",iaxis,fdist,ifsp,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�˶��д�����������˶�. 20170503���Ϲ̼�֧��
Input:          //������handle
				base_axis �˶�����
				iaxis	  ��������ţ�����Ϊ��ǰ�ᣩ
				fdist	  �������˶�����
				ifsp	  �Ƿ�ʹ��SP�˶�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveASynmove(ZMC_HANDLE handle,uint32 base_axis,uint32 iaxis,float fdist,uint32 ifsp)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX || 0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_ASYNMOVE(%d,%f,%d) axis(%d)",iaxis,fdist,ifsp,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�˶����޸�TABLE
Input:          //������handle  
				base_axis	�岹������
				table_num	TABLE���
				fvalue		�޸�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveTable(ZMC_HANDLE handle,uint32 base_axis,uint32 table_num,float fvalue)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_TABLE(%d,%f) axis(%d)",table_num,fvalue,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //BASE���˶��������һ���ɱ����ʱ  �̼�150802���ϰ汾�� ��XPLC160405���ϰ汾֧�֡�
Input:          //������handle  
				base_axis	�岹������
				paraname	�������ַ��� DPOS MPOS IN AIN VPSPEED MSPEED MODBUS_REG MODBUS_IEEE MODBUS_BIT NVRAM VECT_BUFFED  REMAIN 
				inum		������Ż����
				Cmp_mode	�Ƚ����� 1 >=   0=  -1<=  ��IN��BIT���Ͳ�����Ч��
				fvalue		�޸�ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveWait(ZMC_HANDLE handle,uint32 base_axis,char * paraname,int inum,int Cmp_mode,float fvalue)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_Wait(%s,%d,%d,%f) axis(%d)",paraname,inum,Cmp_mode,fvalue,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //BASE���˶��������һ��TASK���� �������Ѿ�����ʱ���ᱨ��������Ӱ�����ִ�С�
Input:          //������handle			
				base_axis	�岹������
				tasknum   	������ 
				labelname		BAS��ȫ�ֺ��������߱��

Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MoveTask(ZMC_HANDLE handle,uint32 base_axis,uint32 tasknum,char * labelname)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > base_axis|| base_axis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "MOVE_TASK(%d,%s) axis(%d)",tasknum,labelname,base_axis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //λ�ñȽ�PSWITCH
Input:          //������handle
				//�Ƚ������ num 0-15
				//�Ƚ���ʹ�� enable 0/1
				//�Ƚϵ���� axisnum
				//����ڱ�� outnum
				//���״̬	outstate 0/1
				//�Ƚ���ʼλ��	setpos
				//�����λλ��	resetpos
Return:         //������
*************************************************************/
int32  ZAux_Direct_Pswitch(ZMC_HANDLE handle, int num, int enable, int axisnum, int outnum, int  outstate, float setpos,float resetpos)
{
	if(0 > num || num > 15)
	{
		return  ERR_AUX_PARAERR;
	}	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "PSWITCH(%d,%d,%d,%d,%d,%f,%f)", num,enable,axisnum,outnum,outstate,setpos,resetpos);
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, NULL, 0);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //Ӳ��λ�ñȽ���� 4ϵ�в�Ʒ���������������֧��Ӳ���Ƚ����
Input:          //������handle
				//ģʽ	 mode 1-�����Ƚ���, 2- ֹͣ��ɾ��û��ɵıȽϵ�.
				//���� direction 0-���긺��,  1- ��������  
				//Ԥ�� Reserve   Ԥ��
				//TABLE��ʼ�� Tablestart  ��һ���Ƚϵ���������TABLE���
				//TABLE������ tableend ���һ���Ƚϵ���������TABLE���
Return:         //������
*************************************************************/
int32  ZAux_Direct_HwPswitch(ZMC_HANDLE handle,int Axisnum,int Mode, int Direction, int Reserve, int Tablestart, int  Tableend)
{
	if(0 > Axisnum || Axisnum > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "HW_PSWITCH(%d,%d,%d,%d,%d) AXIS(%d)", Mode, Direction, Reserve, Tablestart, Tableend,Axisnum);
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, NULL, 0);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
}


/*************************************************************
Description:    //Ӳ��λ�ñȽ����ʣ�໺���ȡ 4ϵ�в�Ʒ���������������֧��Ӳ���Ƚ����
Input:          //������handle
				//ģʽ	 axisnum				���
output:			//λ�ñȽ����ʣ�໺����		buff
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetHwPswitchBuff(ZMC_HANDLE handle,int axisnum,int * buff)
{
	if(0 > axisnum || axisnum > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}	
	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "?HW_PSWITCH(%d)", axisnum);
	//��������ִ�к���
	//iresult =  ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	iresult =  ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",buff);
	
	return ERR_OK;
}

/*************************************************************
Description:    //Ӳ����ʱ������Ӳ���Ƚ������һ��ʱ���ԭ��ƽ 4ϵ�в�Ʒ֧��
Input:          //������	handle  
				//ģʽ			mode		0ֹͣ,  2-����			
				//����ʱ��	   cyclonetime  us��λ
				//��Чʱ��	   optime		us��λ
				//�ظ�����	   reptimes
				//���ȱʡ״̬ opstate		����ڱ�Ϊ�Ǵ�״̬��ʼ��ʱ
				// ����ڱ��  opnum		������Ӳ���Ƚ�����Ŀ�
Return:         //������
*************************************************************/
int32  ZAux_Direct_HwTimer(ZMC_HANDLE handle,int mode,int cyclonetime,int optime,int reptimes,int opstate,int  opnum )
{
	
	char  cmdbuff[2048];
    //��������
	sprintf(cmdbuff, "HW_TIMER(%d,%d,%d,%d,%d,%d)",mode,cyclonetime,optime,reptimes,opstate,opnum);
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, NULL, 0);
	return ZAux_Execute(handle, cmdbuff, NULL,0);
}

/*************************************************************
Description:    //��ȡ��ֹͣԭ��
Input:          //������handle  
				iaxis ���
Output:         //piValue ����״̬����ӦAXISSTATUS�ж϶�Ӧλ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAxisStopReason(ZMC_HANDLE handle, int iaxis, int *piValue)
{
	int32 iresult;
	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(NULL == piValue || iaxis > MAX_AXIS_AUX || iaxis < 0)
	{
		return  ERR_AUX_PARAERR;
	}
	
    //��������
	sprintf(cmdbuff, "?AXIS_STOPREASON(%d)", iaxis);
	
	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
        return iresult;
	}
	
	//
	if(0 == strlen(cmdbuffAck))
	{
        return ERR_NOACK;
	}
	
	//
	sscanf(cmdbuffAck, "%d",piValue);
	
	return ERR_OK;
}

/*************************************************************
Description:    //�����Ͷ�ȫ�������״̬
Input:          //������handle 
				sParam ����������ַ���
				imaxaxis ������
Output:         pfValue  ���ص�������б�
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAllAxisPara(ZMC_HANDLE handle,const char *sParam,int imaxaxis,float * pfValue)
{

	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue || imaxaxis <=0 || imaxaxis > MAX_AXIS_AUX)
	{
	return  ERR_AUX_PARAERR;
	}

	strcpy(cmdbuff, "?*");
	strcat(cmdbuff, sParam);

	//��������ִ�к���
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
			return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
			return ERR_NOACK;
	}

	//
	iresult = ZAux_TransStringtoFloat(cmdbuffAck,imaxaxis,pfValue);
	if(ERR_OK != iresult)
	{
	return iresult;
	}
	return ERR_OK;
}

/*************************************************************
Description:    //�����Ͷ�ȫ�������״̬  IdleStatus-�˶�״̬ DposStatus-��������  MposStatus-��������  AxisStatus-��״̬
Input:          //������handle 
				imaxaxis ������
Output:         IdleStatus �˶�״̬
				DposStatus ��������
				MposStatus ��������
				AxisStatus ��״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetAllAxisInfo(ZMC_HANDLE handle,int imaxaxis,int * IdleStatus,float * DposStatus,float * MposStatus,int * AxisStatus)
{
	int32 iresult = 0;
	if(imaxaxis <=0 || imaxaxis > MAX_AXIS_AUX || NULL == IdleStatus || NULL == DposStatus || NULL == MposStatus || NULL == AxisStatus)
	{
	return  ERR_AUX_PARAERR;
	}
	float pi_idle[MAX_AXIS_AUX],pf_axisstatus[MAX_AXIS_AUX];
	iresult += ZAux_Direct_GetAllAxisPara(handle,"IDLE",imaxaxis,pi_idle);
	iresult += ZAux_Direct_GetAllAxisPara(handle,"DPOS",imaxaxis,DposStatus);
	iresult += ZAux_Direct_GetAllAxisPara(handle,"MPOS",imaxaxis,MposStatus);
	iresult += ZAux_Direct_GetAllAxisPara(handle,"AXISSTATUS",imaxaxis,pf_axisstatus);

	if(iresult == ERR_OK)
	{
		for(int i = 0;i<imaxaxis;i++)
		{
			*(IdleStatus + i ) = (int)pi_idle[i];
			*(AxisStatus + i ) = (int)pf_axisstatus[i];
		}
	}
	return iresult;
}

/*************************************************************
Description:    //����BASIC�Զ���ȫ������  
Input:          //������handle  
				arrayname ��������
				arraystart ������ʼԪ��
				numes		Ԫ������
				pfValue     ����ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetUserArray(ZMC_HANDLE handle,char * arrayname ,int arraystart, int numes, float *pfValue)
{
	int i;
	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue || numes < 0)
	{
		return  ERR_AUX_PARAERR;
	}

	for(i = 0; i< numes; i++)
	{
		//
		sprintf(cmdbuff, "%s(%d) =  %f ",arrayname ,(i + arraystart) , pfValue[i]);
		//��������ִ�к���
		iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		if(ERR_OK != iresult)
		{
			return iresult;
		}
	}

	return ERR_OK;


}

/*************************************************************
Description:    //��ȡ����BASIC�Զ���ȫ������ , ����һ�ζ�ȡ���
Input:          //������handle  
				arrayname ��������
				arraystart ������ʼԪ��
				numes		Ԫ������
Output:         //pfValue  ���ʱ�������ռ�.
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetUserArray(ZMC_HANDLE handle,char * arrayname ,int arraystart, int numes, float *pfValue)
{
	int i, icur, isend;
	int32 iresult;
	char  tempbuff[2048];
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue || numes < 0)
	{
		return  ERR_AUX_PARAERR;
	}

	isend = 0;
	while(1)
	{
		//һ�η��͸���30��
		icur = numes - isend;
		if(icur > 30)
		{
			icur = 30;
		}

		//��������
		strcpy(cmdbuff, "?*");

		//
		sprintf(tempbuff, "%s(%d,%d)",arrayname,arraystart+isend,icur);
		strcat(cmdbuff, tempbuff);


		//��������ִ�к���
		iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
		if(ERR_OK != iresult)
		{
			return iresult;
		}

		//
		if(0 == strlen(cmdbuffAck))
		{
			return ERR_NOACK;
		}

		//
		iresult = ZAux_TransStringtoFloat(cmdbuffAck, icur, pfValue+isend);
		if(ERR_OK != iresult)
		{
			return iresult;
		}

		isend+= icur;
		if(isend >= numes)
		{
			break;
		}
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�����Զ������, 
Input:          //������handle  
				varname ���������ַ���
				pfValue	�趨ֵ
Output:         //  
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetUserVar(ZMC_HANDLE handle,char * varname , float pfValue)
{
	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	//
	sprintf(cmdbuff, "%s = %f",varname,pfValue);


	//��������ִ�к���
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�Զ���ȫ�ֱ���, 
Input:          //������handle  
				varname ���������ַ���
Output:         //pfValue  ���ʱ�������ռ�.
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetUserVar(ZMC_HANDLE handle,char * varname , float *pfValue)
{
	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == pfValue)
	{
		return  ERR_AUX_PARAERR;
	}

	//
	sprintf(cmdbuff, "?%s",varname);


	//��������ִ�к���
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%f",pfValue);

	return ERR_OK;
}


/*************************************************************
Description:    //���������������
Input:          //PCI���ţ� ���Ŵ�0��ʼ��ʵ������ΪPCI����+1(���ݴ�ͳPCI������)
Output:         //������handle
Return:         //������
*************************************************************/
int32  ZAux_OpenPci(uint32 cardnum, ZMC_HANDLE * phandle)
{
	int32 iresult;
    char buffer[1024];
	sprintf(buffer, "PCI%d", cardnum + 1);

	iresult = ZMC_Open(ZMC_CONNECTION_PCI,buffer, phandle);
	return iresult;
}

/*************************************************************
Description:    //��ȡPCI�Ŀ��ƿ�����
Input:          //
Return:         //�������PCI������
*************************************************************/
int32  ZAux_GetMaxPciCards()
{
	return ZMC_GetMaxPciCards();
}

/*************************************************************
Description:    //��ȡ����������Ϣ
Input:          //������handle  
Output:         SoftType �������ͺ�����
				SoftVersion �����������汾���̼��汾��
				ControllerId	������ΨһID
Return:         //������
*************************************************************/
int32  ZAux_GetControllerInfo(ZMC_HANDLE handle ,char * SoftType ,char * SoftVersion ,char *  ControllerId)
{

	int32 iresult;
	char  cmdbuffAck[2048];

	//�����汾 ?CONTROL SoftType   
	iresult = ZAux_Execute(handle, "?CONTROL", cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}
	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	//
	sscanf(cmdbuffAck, "%s",SoftType);


	//?VERSION_DATE SoftVersion
	iresult = ZAux_Execute(handle, "?VERSION_DATE", cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}
	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	//
	sscanf(cmdbuffAck, "%s",SoftVersion);

	//������ID  ?SERIAL_NUMBER
	iresult = ZAux_Execute(handle, "?SERIAL_NUMBER", cmdbuffAck,2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}
	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	//
	sscanf(cmdbuffAck, "%s",ControllerId);

	return ERR_OK;
}


/**************************************************������غ���***********************************************
�󲿷ֺ���ֻ֧��Execute��ʽ����  
�������߽ڵ���Ϣ��ȡ
���߻��㣬������״̬�жϵ�
***************************************************************************************************************/

/*************************************************************
Description:    //��ȡ�����Ͻڵ�����
Input:          //������handle 
:				 //slot ��λ��ȱʡ0
Output:         //piValue ����ɨ��ɹ��ڵ�����
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetNodeNum(ZMC_HANDLE handle,int slot,int *piValue)
{
	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?NODE_COUNT(%d)", slot);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%d",piValue);

	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�ڵ��ϵ���Ϣ
Input:          //������handle 
				slot	��λ��
				node	�ڵ���
				sel		��Ϣ���	0-���̱��1-�豸��� 2-�豸�汾 3-���� 10-IN���� 11-OUT���� 
Output:         //piValue ������Ϣ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetNodeInfo(ZMC_HANDLE handle,int slot,int node,int sel ,int *piValue)
{
	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?NODE_INFO(%d,%d,%d)", slot,node,sel);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	//
	sscanf(cmdbuffAck, "%d",piValue);
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�ڵ�����״̬
Input:          //������handle  
				slot ��λ��ȱʡ0
				node �ڵ���
Output:         //nodestatus ��λ���� bit0-�ڵ��Ƿ����  bit1-ͨѶ״̬   bit2-�ڵ�״̬
ֵΪ1ʱ��bit0Ϊ1��bit1��bit2Ϊ0���豸ͨѶ����
ֵΪ3ʱ��bit0��bit1Ϊ1��bit2Ϊ0���豸ͨѶ����
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetNodeStatus(ZMC_HANDLE handle,uint32 slot,uint32 node,uint32 * nodestatus)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == nodestatus)
	{
		return  ERR_AUX_PARAERR;
	}
	//��������
	sprintf(cmdbuff, "?NODE_STATUS(%d,%d)", slot,node);
	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	//
	sscanf(cmdbuffAck, "%d",nodestatus);
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ�ڵ�SDO������Ϣ
Input:          //������handle  
				slot	��λ�� ȱʡ0
				node	�ڵ���
				index	�����ֵ��ţ�ע�⺯��Ϊ10�������ݣ�
				subindex	�ӱ��	��ע�⺯��Ϊ10�������ݣ�
				type	��������  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
Output:         //value ��ȡ������ֵ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_SDORead(ZMC_HANDLE handle,uint32 slot,uint32 node,uint32 index,uint32 subindex ,uint32 type ,int32 * value)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == value || node > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "SDO_READ(%d,%d,%d,%d,%d,0)",slot, node,index,subindex,type);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(0 != strlen(cmdbuffAck))
	{
		return ERR_AUX_NOTSUPPORT;
	}
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	float fvalue = 0;
	iresult = ZAux_Direct_GetTable(handle,0,1,&fvalue);
	if(ERR_OK != iresult)
	{
		return iresult;
	}	
	*value = (int)fvalue;
	return ERR_OK;
}

/*************************************************************
Description:    //д�ڵ�SDO������Ϣ
Input:          //������handle  
				slot	��λ�� ȱʡ0
				node	�ڵ���
				index	�����ֵ��ţ�ע�⺯��Ϊ10�������ݣ�
				subindex	�ӱ��	��ע�⺯��Ϊ10�������ݣ�
				type	��������  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
				value	�趨������ֵ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_SDOWrite(ZMC_HANDLE handle,uint32 slot,uint32 node,uint32 index,uint32 subindex ,uint32 type ,int32  value)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(node > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff,"SDO_WRITE(%d,%d,%d,%d,%d,%d)",slot, node,index,subindex,type,value);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(0 != strlen(cmdbuffAck))
	{
		return ERR_AUX_NOTSUPPORT;
	}

	return iresult;
}

/*************************************************************
Description:    //��ȡ��SDO������Ϣ
Input:          //������handle  
				slot	��λ�� ȱʡ0
				node	�ڵ���
				index	�����ֵ��ţ�ע�⺯��Ϊ10�������ݣ�
				subindex	�ӱ��	��ע�⺯��Ϊ10�������ݣ�
				type	��������  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
Output:         //value ��ȡ������ֵ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_SDOReadAxis(ZMC_HANDLE handle,uint32 iaxis,uint32 index,uint32 subindex ,uint32 type ,int32 * value)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == value || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "SDO_READ_AXIS(%d,%d,%d,%d,0)",iaxis,index,subindex,type);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(0 != strlen(cmdbuffAck))
	{
		return ERR_AUX_NOTSUPPORT;
	}
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	float fvalue = 0;
	iresult = ZAux_Direct_GetTable(handle,0,1,&fvalue);
	if(ERR_OK != iresult)
	{
		return iresult;
	}	
	*value = (int)fvalue;
	return ERR_OK;
}

/*************************************************************
Description:    //д��SDO������Ϣ
Input:          //������handle  
				slot	��λ�� ȱʡ0
				node	�ڵ���
				index	�����ֵ��ţ�ע�⺯��Ϊ10�������ݣ�
				subindex	�ӱ��	��ע�⺯��Ϊ10�������ݣ�
				type	��������  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
				value	�趨������ֵ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_SDOWriteAxis(ZMC_HANDLE handle,uint32 iaxis,uint32 index,uint32 subindex ,uint32 type ,int32  value)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff,"SDO_WRITE_AXIS(%d,%d,%d,%d,%d)",iaxis,index,subindex,type,value);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(0 != strlen(cmdbuffAck))
	{
		return ERR_AUX_NOTSUPPORT;
	}

	return iresult;
}


/*************************************************************
Description:    //Rtex��ȡ������Ϣ
Input:          //������handle  
				iaxis	���
				ipara	��������*256 + �������  Pr7.11-ipara = 7*256+11
Output:         //value ��ȡ������ֵ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_RtexRead(ZMC_HANDLE handle,uint32 iaxis ,uint32 ipara ,float * value)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == value || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "BASE(%d)\r\nDRIVE_READ(%d,0)",iaxis,ipara);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}
	if(0 != strlen(cmdbuffAck))
	{
		return ERR_AUX_NOTSUPPORT;
	}

	//
	iresult = ZAux_Direct_GetVrf(handle,0,1,value);
	if(ERR_OK != iresult)
	{
		return iresult;
	}	
	return ERR_OK;
}

/*************************************************************
Description:    //Rtexд������Ϣ
Input:          //������handle  
				iaxis	���
				ipara	��������*256 + �������  Pr7.11-ipara = 7*256+11
				value	�趨������ֵ
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_RtexWrite(ZMC_HANDLE handle,uint32 iaxis ,uint32 ipara,float  value)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff,"BASE(%d)\r\nDRIVE_WRITE(%d,%f)",iaxis, ipara,value);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(0 != strlen(cmdbuffAck))
	{
		return ERR_AUX_NOTSUPPORT;
	}
	return iresult;
}

/*************************************************************
Description:    //���û���ƫ�ƾ���
Input:          //������handle  
				iaxis ���
				fValue ƫ�ƾ���
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_SetDatumOffpos(ZMC_HANDLE handle, int iaxis, float fValue)
{


	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "DATUM_OFFSET(%d)=%f", iaxis, fValue);

	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡ����ƫ�ƾ���
Input:          //������handle  
				iaxis ���
Output:         //fValue ������ƫ�ƾ���
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetDatumOffpos(ZMC_HANDLE handle, int iaxis, float *fValue)
{
	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == fValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?DATUM_OFFSET(%d)", iaxis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%f",fValue);
	return ERR_OK;
}

/*************************************************************
Description:    //��������������
Input:          //������handle  
				homemode ����ģʽ���鿴�������ֲ�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_Datum(ZMC_HANDLE handle,uint32 iaxis,uint32  homemode)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	int32 iresult;

	sprintf(cmdbuff, "AXIS_STOPREASON(%d) = 0\r\nDATUM(21,%d) AXIS(%d)",iaxis,homemode,iaxis);


	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,0);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�������������״̬
Input:          //������handle  
				iaxis ���
Output:         //homestatus ������ɱ�־ 0-�����쳣 1����ɹ�
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetHomeStatus(ZMC_HANDLE handle,uint32 iaxis,uint32 * homestatus)
{

	int32 iresult;
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];	

	if(NULL == homestatus || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	sprintf(cmdbuff, "?DRIVE_STATUS(%d),IDLE(%d),AXIS_STOPREASON(%d)",iaxis,iaxis,iaxis);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,2048);
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}
	int temparray[4];
	iresult = ZAux_TransStringtoInt(cmdbuffAck,4,&temparray[0]);
	int tempstatus = temparray[0];
	int idlestatus = temparray[1];
	int stopstatus = temparray[2];

	
	if((idlestatus == -1) && ((tempstatus >> 12	) & 1) == 1 && stopstatus == 0)		//ֹͣ��
	{
		*homestatus = 1;	//�������	
	}
	else
	{
		*homestatus = 0;	//����δ�ɹ�
	}
	return ERR_OK;
}


/*************************************************************
Description:    //������������ŷ�����
Input:          //������handle 
				iaxis ���
				mode ģʽ 0-�����ǰ�澯  1-�����ʷ�澯  2-����ⲿ����澯
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_DriveClear(ZMC_HANDLE handle,uint32 iaxis,uint32  mode)
{

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	
	if(0 > iaxis || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	
	int32 iresult;

	sprintf(cmdbuff, "BASE(%d)\r\nDRIVE_CLEAR(%d)\r\nDATUM(0)",iaxis,mode);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,0);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck,0);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��ǰ����������ǰ����	��Ҫ���ö�Ӧ��DRIVE_PROFILE����
Input:          //������handle  
				iaxis ���
Output:         //piValue ��ǰת��
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetDriveTorque(ZMC_HANDLE handle, int iaxis, int *piValue)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?DRIVE_TORQUE(%d)", iaxis);

	//��������ִ�к���
	//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%d",piValue);
	return ERR_OK;
}

/*************************************************************
Description:    //���õ�ǰ�����������ת��  ��Ҫ���ö�Ӧ��DRIVE_PROFILE����
Input:          //������handle		
				iaxis ���
				piValue ���ת������
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_SetMaxDriveTorque(ZMC_HANDLE handle, int iaxis, int piValue)
{
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}
	//��������
	sprintf(cmdbuff, "DRIVE_TORQUEMAX(%d)=%d", iaxis, piValue);

	//��������ִ�к���
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,0);
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}


/*************************************************************
Description:    //��ȡ��ǰ�����������ת��  ��Ҫ���ö�Ӧ��DRIVE_PROFILE����
Input:          //������handle  
				iaxis ���
Output:         //piValue ���ص����ת��
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetMaxDriveTorque(ZMC_HANDLE handle, int iaxis, int *piValue)
{
	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == piValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?DRIVE_TORQUEMAX(%d)", iaxis);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%d",piValue);
	return ERR_OK;
}

/*************************************************************
Description:    //����ģ������� ���ء��ٶ�ģʽ�¿���  ����������Ҫ���ö�ӦDRIVE_PROFILE���� ��ATYPE
Input:          //������handle  
				iaxis ���
				ģ���� ���ֵ
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetDAC(ZMC_HANDLE handle, int iaxis, float fValue)
{


	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
	if( iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "DAC(%d)=%f", iaxis, fValue);

	//��������ִ�к���
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck, 2048);
	//return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��ȡģ������� ���ء��ٶ�ģʽ�¿���  ����������Ҫ���ö�ӦDRIVE_PROFILE���� ��ATYPE
Input:          //������handle  
				iaxis ���
Output:         //fValue ģ��������ֵ
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetDAC(ZMC_HANDLE handle, int iaxis, float *fValue)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == fValue || iaxis > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//��������
	sprintf(cmdbuff, "?DAC(%d)", iaxis);

	//��������ִ�к���
	iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
	//iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	if(ERR_OK != iresult)
	{
		return iresult;
	}

	//
	if(0 == strlen(cmdbuffAck))
	{
		return ERR_NOACK;
	}

	//
	sscanf(cmdbuffAck, "%f",fValue);
	return ERR_OK;
}

/*************************************************************
Description:    //����ɨ���ʼ��  ��ֻ���Zmotion tools �����������ù����߲���������ʹ����Ч��
Input:          //������handle  
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_InitBus(ZMC_HANDLE handle)
{
	return  ZAux_Direct_SetUserVar(handle,"P_BusCmd",2);
}


/*************************************************************
Description:    //��ȡ���߳�ʼ�����״̬  ��ֻ���Zmotion tools �����������ù����߲���������ʹ����Ч��
Input:          //������handle  
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_BusCmd_GetInitStatus(ZMC_HANDLE handle,int *piValue)
{
	int32 iresult;
	float scan_flag,start_flag;
	if(NULL == piValue)
	{
		return  ERR_AUX_PARAERR;
	}
	iresult = ZAux_Direct_GetUserVar(handle,"Bus_Scan_Status",&scan_flag);
	if(iresult != ERR_OK)
	{
		return iresult;
	}
	iresult = ZAux_Direct_GetUserVar(handle,"Bus_Start_Status",&start_flag);
	if(iresult != ERR_OK)
	{
		return iresult;
	}
	if((int)scan_flag == 1 && (int)start_flag == 1)
	{
		*piValue = 1;
	}
	else
	{
		*piValue = 0;
	}
	return ERR_OK;
}

/*************************************************************
Description:    //��ȡ��������ź�
Input:          //������handle  
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetInMulti(ZMC_HANDLE handle, int startio , int endio, int32 *piValue)
{

	int32 iresult;

	char  cmdbuff[2048];
	char  cmdbuffAck[2048];

	if(NULL == piValue || startio > endio)
	{
		return  ERR_AUX_PARAERR;
	}

	int32 icur,istart,iend;		//һ�����32��
	icur=0;
	while(1)
	{
		iend = startio + 31;
		if(iend > endio)
		{
			iend = endio;
		}
		//��������
		sprintf(cmdbuff, "?IN(%d,%d)", startio,iend);

		//��������ִ�к���
		//iresult = ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
		iresult = ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
		if(ERR_OK != iresult)
		{
			return iresult;
		}

		//
		if(0 == strlen(cmdbuffAck))
		{
			return ERR_NOACK;
		}

		//
		sscanf(cmdbuffAck, "%d",&piValue[icur]);

		startio = iend + 1;		
		icur ++;
		if(iend == endio)			//��ȡ���
		{
			break;
		}
	}

	return ERR_OK;
}

/*************************************************************
Description:    //�������ʱ�ȴ�ʱ��
Input:          //������handle ����ʱ��
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_SetTimeOut(ZMC_HANDLE  handle, uint32 timems)
{
	g_ZMC_MaxExcuteWaitms = timems;
	return ZMC_SetTimeOut(handle,timems);
}


/*************************************************************
Description:    //Ӳ��λ�ñȽ����2 4ϵ�в�Ʒ, 20170513���ϰ汾֧��.  ZMC306E/306N֧��
Input:          //������					handle
				//ģʽ						mode		
				//����ڱ��				Opnum		4ϵ�� out 0-3ΪӲ��λ�ñȽ����
				//��һ���Ƚϵ�����״̬	Opstate		0-�ر� 1��										
				//�๦�ܲ���				ModePara1	 
				//�๦�ܲ���				ModePara2
				//�๦�ܲ���				ModePara3
				//�๦�ܲ���				ModePara4

mode 1-�����Ƚ���, 
		ModePara1 =  ��һ���Ƚϵ���������TABLE���
		ModePara2 =	 ���һ���Ƚϵ���������TABLE���
		ModePara3 =  ��һ�����жϷ���,  0-���긺��,  1- ��������,  -1-��ʹ�÷���
		ModePara4 =	 Ԥ��

mode 2- ֹͣ��ɾ��û��ɵıȽϵ�. 
		ModePara1 =  Ԥ��
		ModePara2 =	 Ԥ��
		ModePara3 =  Ԥ��
		ModePara4 =	 Ԥ��

mode 3- ʸ���ȽϷ�ʽ
		ModePara1 =  ��һ���Ƚϵ���������TABLE���
		ModePara2 =	 ���һ���Ƚϵ���������TABLE���
		ModePara3 =  Ԥ��
		ModePara4 =	 Ԥ��

Mode=4 :ʸ���ȽϷ�ʽ, �����Ƚϵ�
		ModePara1 =  �Ƚϵ�����
		ModePara2 =	 Ԥ��
		ModePara3 =  Ԥ��
		ModePara4 =	 Ԥ��

Mode=5 :ʸ���ȽϷ�ʽ, ��������ģʽ
		ModePara1 =  �Ƚϵ�����
		ModePara2 =	 �ظ�����, һ�������ڱȽ�����, �������Ч״̬,�������Ч״̬.
		ModePara3 =  ���ھ���, ÿ������������Opstate, �����Ч״̬�ľ��루ModePara4����ԭΪ��Ч״̬.
		ModePara4 =	 �����Ч״̬�ľ���,  (ModePara3- ModePara4) Ϊ��Ч״̬����

Mode=6 :ʸ���ȽϷ�ʽ, ����ģʽ, ����ģʽһ����HW_TIMERһ��ʹ��.
		ModePara1 =  �Ƚϵ�����
		ModePara2 =	 �ظ�����, һ������ֻ�Ƚ�һ��
		ModePara3 =  ���ھ���, ÿ������������һ��
		ModePara4 =	 Ԥ��
Return:         //������
*************************************************************/
int32  ZAux_Direct_HwPswitch2(ZMC_HANDLE handle,int Axisnum,int Mode, int Opnum , int Opstate, float ModePara1, float ModePara2,float ModePara3,float ModePara4)
{
	if(0 > Axisnum || Axisnum > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}	
	char  cmdbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	switch(Mode)
	{
		case 1:
			sprintf(cmdbuff, "HW_PSWITCH2(%d,%d,%d,%f,%f,%f) AXIS(%d)", Mode, Opnum, Opstate, ModePara1,ModePara2,ModePara3,Axisnum);
			break;
		case 2:
			sprintf(cmdbuff, "HW_PSWITCH2(%d) AXIS(%d)", Mode,Axisnum);
			break;
		case 3:
			sprintf(cmdbuff, "HW_PSWITCH2(%d,%d,%d,%f,%f) AXIS(%d)", Mode, Opnum, Opstate, ModePara1,ModePara2,Axisnum);
			break;
		case 4:
			sprintf(cmdbuff, "HW_PSWITCH2(%d,%d,%d,%f) AXIS(%d)", Mode, Opnum, Opstate, ModePara1,Axisnum);
			break;
		case 5:
			sprintf(cmdbuff, "HW_PSWITCH2(%d,%d,%d,%f,%f,%f,%f) AXIS(%d)", Mode, Opnum, Opstate, ModePara1,ModePara2,ModePara3,ModePara4,Axisnum);
			break;
		case 6:
			sprintf(cmdbuff, "HW_PSWITCH2(%d,%d,%d,%f,%f,%f) AXIS(%d)", Mode, Opnum, Opstate, ModePara1,ModePara2,ModePara3,Axisnum);
			break;
		default:
			return  ERR_AUX_PARAERR;			
			break;
	}
	//��������ִ�к���
	//return ZAux_DirectCommand(handle, cmdbuff, NULL, 0);
	return ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //��ȡ�������������
Input:          //������handle 
Output:         //Max_VirtuAxises	 �����������
Output:         //Max_motor			 ���������
Output:         //Max_io			 ���IN,OUT,AD,DA����				
Return:         //������
*************************************************************/
int32  ZAux_GetSysSpecification(ZMC_HANDLE  handle, uint16 * Max_VirtuAxises,uint8 * Max_motor,uint8 *Max_io)
{
	int32 iresult;
	struct_SysMaxSpecification specification;

	if(Max_VirtuAxises == NULL || Max_motor == NULL || Max_io == NULL)
	{
		return  ERR_AUX_PARAERR;
	}

	iresult = ZMC_GetSysSpecification(handle,&specification);
	if(ERR_OK != iresult)
	{
		return iresult;
	}
	*Max_motor = specification.m_MaxAxises;
	*Max_VirtuAxises= specification.m_MaxVirtuAxises;
	*Max_io =  specification.m_MaxInController;
	*(Max_io + 1) =  specification.m_MaxOutController;
	*(Max_io + 2) =  specification.m_MaxAdController;
	*(Max_io + 3) =  specification.m_MaxDaController;

	return ERR_OK;
}

/*************************************************************
Description:    //�������Զ��ϱ�
Input:          //������	handle 
Input:         //�ص�����    pcallback		
		   PZMCAutoUpCallBack������ʽ
				Input:          //������handle
				itypecode: �ϴ�������
				idatalength: ���ݳ���
				pdata: ����ָ��
Return:         //������
*************************************************************/
int32  ZAux_SetAutoUpCallBack(ZMC_HANDLE handle, PZMCAutoUpCallBack pcallback)
{
	return ZMC_SetAutoUpCallBack(handle,pcallback);
}


/*************************************************************
Description:    //IO�ӿ� ���ö�·���
Input:          //������handle
Input:          //IO����ʼ���  iofirst
Input:          //IO�ڽ������  ioend
Input:          //�����״̬    istate��λ�洢��һ��UINT�洢32�������״̬
Output:         //״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_SetOutMulti(ZMC_HANDLE handle, uint16 iofirst,uint16 ioend, uint32 * istate)
{
	int32 iresult;
	if(NULL == istate || iofirst > ioend)
	{
		return  ERR_AUX_PARAERR;
	}

	int32 icur,startio,iend;		//һ�����32��
	startio = iofirst;
	icur=0;
	while(1)
	{
		iend = startio + 31;
		if(iend > ioend)
		{
			iend = ioend;
		}
		iresult = ZMC_SetOutAll(handle, startio,iend, *(istate + icur));
		if(iresult != ERR_OK)
		{
			return  iresult;
		}
		startio = iend + 1;		
		icur ++;
		if(iend == ioend)			//�������
		{
			break;
		}
	}
	return ERR_OK;
}


/*************************************************************
Description:    //IO�ӿ� ���ö�·���
Input:          //������handle
Input:          //IO����ʼ���  iofirst
Input:          //IO�ڽ������  ioend
Input:          //�����״̬    istate��λ�洢��һ��UINT�洢32�������״̬
Output:         //״̬
Return:         //������
*************************************************************/
int32  ZAux_Direct_GetOutMulti(ZMC_HANDLE handle, uint16 iofirst,uint16 ioend, uint32 * istate)
{
	int32 iresult;
	if(NULL == istate || iofirst > ioend)
	{
		return  ERR_AUX_PARAERR;
	}

	int32 icur,startio,iend;		//һ�����32��
	startio = iofirst;
	icur=0;
	while(1)
	{
		iend = startio + 31;
		if(iend > ioend)
		{
			iend = ioend;
		}
		iresult = ZMC_GetOutAll(handle, startio,iend, &istate[icur]);
		if(iresult != ERR_OK)
		{
			return  iresult;
		}
		startio = iend + 1;		
		icur ++;
		if(iend == ioend)			//�������
		{
			break;
		}
	}
	return ERR_OK;
}


/*************************************************************
Description:    //������Զ���ֱ�߲岹 
Input:          //������handle
				iMoveLen			��д���˶�����
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MultiMove(ZMC_HANDLE handle,int iMoveLen, int imaxaxises, int *piAxislist, float *pfDisancelist)
{
	int i,j;
	int32 iresult;
	char  cmdbuff[20480];
	char  tempbuff[20480];
	char  cmdbuffAck[20480];	
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//�ȶ�ȡʣ��ֱ�߻���
	int iBuffLen = 0;
	iresult = ZAux_Direct_GetRemain_LineBuffer(handle,piAxislist[0],&iBuffLen);
	if(iBuffLen <= iMoveLen)
	{
		return 1002;			//���岻��
	}


	//��������BASE����
	strcpy(cmdbuff, "BASE(");
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
	strcat(cmdbuff, tempbuff);
	//����
	strcat(cmdbuff, "\n");

	//�����˶�����
	for(j=0;j<iMoveLen;j++)
	{
		strcat(cmdbuff, "MOVE(");

		for(i = 0; i< imaxaxises-1; i++)
		{
			//
			sprintf(tempbuff, "%f,",pfDisancelist[i +j*imaxaxises]);
			strcat(cmdbuff, tempbuff);
		}

		//
		sprintf(tempbuff, "%f)\n",pfDisancelist[i + j*imaxaxises]);
		strcat(cmdbuff, tempbuff);
	}

	int ilen = strlen(cmdbuff);				//��ȡ�����
	if(ilen > 1000)
	{
		return  20002;
	}
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);

}



/*************************************************************
Description:    //������Զ���ֱ�߲岹 
Input:          //������handle
				iMoveLen			��д���˶�����
				imaxaxises			�����˶�������
				piAxislist			����б�
				pfDisancelist		�����б�
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_MultiMoveAbs(ZMC_HANDLE handle,int iMoveLen, int imaxaxises, int *piAxislist, float *pfDisancelist)
{
	int i,j;
	int32 iresult;
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];	
	if(0 > imaxaxises || imaxaxises > MAX_AXIS_AUX)
	{
		return  ERR_AUX_PARAERR;
	}

	//�ȶ�ȡʣ��ֱ�߻���
	int iBuffLen = 0;
	iresult = ZAux_Direct_GetRemain_LineBuffer(handle,piAxislist[0],&iBuffLen);
	if(iBuffLen <= iMoveLen)
	{
		return 1002;			//���岻��
	}


	//��������BASE����
	strcpy(cmdbuff, "BASE(");
	for(i = 0; i< imaxaxises-1; i++)
	{
		//
		sprintf(tempbuff, "%d,",piAxislist[i]);
		strcat(cmdbuff, tempbuff);
	}
	sprintf(tempbuff, "%d)",piAxislist[imaxaxises-1]);
	strcat(cmdbuff, tempbuff);
	//����
	strcat(cmdbuff, "\n");

	//�����˶�����
	for(j=0;j<iMoveLen;j++)
	{
		strcat(cmdbuff, "MOVEABS(");

		for(i = 0; i< imaxaxises-1; i++)
		{
			//
			sprintf(tempbuff, "%f,",pfDisancelist[i +j*imaxaxises]);
			strcat(cmdbuff, tempbuff);
		}

		//
		sprintf(tempbuff, "%f)\n",pfDisancelist[i + j*imaxaxises]);
		strcat(cmdbuff, tempbuff);
	}


	int ilen = strlen(cmdbuff);				//��ȡ�����
	if(ilen > 1000)
	{
		return  20002;
	}
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
}

/*************************************************************
Description:    //��е������ϵ��ת
Input:          //������handle
				iaxis					��� �ؽ���/������
				pfRotatePara			ƽ����ת����
Output:         //
Return:         //������
*************************************************************/
int32  ZAux_Direct_FrameRotate(ZMC_HANDLE handle,int iaxis, float *pfRotatePara)
{
	int i;	
	char  cmdbuff[2048];
	char  tempbuff[2048];
	char  cmdbuffAck[2048];
    //��������
	sprintf(cmdbuff, "BASE(%d)\n",iaxis);
			
    //��������
	sprintf(tempbuff, "FRAME_ROTATE(%f,%f,%f,%f,%f,%f)", pfRotatePara[0],pfRotatePara[1],pfRotatePara[2],pfRotatePara[3],pfRotatePara[4], pfRotatePara[5]);
    strcat(cmdbuff, tempbuff);	
	//��������ִ�к���
	return ZAux_DirectCommand(handle, cmdbuff, cmdbuffAck, 2048);
	//return ZAux_Execute(handle, cmdbuff, cmdbuffAck,2048);
}

/*************************************************************
Description:    //��ȡCAN��չ��Դ���
Input:          //������handle 
Output:         //CanNum			 ��ǰ���ӵ�CAN��վ����
Output:         //CanId_List		 ��ǰ���ӵ�CAN��վID�б�
Output:         //CanIn_List		 �ڵ����������
Output:         //CanOut_List		 �ڵ����������
Output:         //CanAin_List		 �ڵ�AD����	
Output:         //CanAOut_List		 �ڵ�DA����	
Output:         //CanAxis_List		 �ڵ�������				
Return:         //������
*************************************************************/
int32  ZAux_GetCanInfo(ZMC_HANDLE  handle, uint8 * CanNum,uint16 * CanId_List,uint8 *CanIn_List,uint8 *CanOut_List,uint8 *CanAin_List,uint8 *CanAOut_List,uint8 *CanAxis_List)
{
	int32 iresult;
	uint16 Canid = 0;
	uint8 Temp_CanNum = 0;
	struct_ChildCardInfo *m_CardInfo = (struct_ChildCardInfo *)malloc(sizeof(struct_ChildCardInfo));

	iresult = ZMC_EnumChildCard(handle,-1, m_CardInfo);
	Canid = m_CardInfo->m_cardid;
	while(Canid < 128 && (ERR_OK == iresult))
	{
		*(CanId_List + Temp_CanNum) = Canid;
		*(CanIn_List + Temp_CanNum) = m_CardInfo->m_imaxin;
		*(CanOut_List + Temp_CanNum) = m_CardInfo->m_imaxout;
		*(CanAin_List + Temp_CanNum) = m_CardInfo->m_imaxad;
		*(CanAOut_List + Temp_CanNum) = m_CardInfo->m_imaxda;
		*(CanAxis_List + Temp_CanNum) = m_CardInfo->m_iAxises;

		iresult = ZMC_EnumChildCard(handle, Canid, m_CardInfo);
		Canid = m_CardInfo->m_cardid;

		Temp_CanNum ++;
	}

	*CanNum = Temp_CanNum;
	return ERR_OK;
}
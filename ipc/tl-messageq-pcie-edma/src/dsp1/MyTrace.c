/*
 * MyTrace.c
 *
 *  Created on: 2016-6-13
 *      Author: ljy
 */
#include "MyTrace.h"

#include <stdarg.h>
#include <stdio.h>

/* Set up printf */
#include <xdc/runtime/System.h>
#define printf System_printf


#define IOMSG_MAX_LENGTH 2048

//消息输出级别
u32 uIoMsgLevel;

/*****************************************************************************************
*函数名称：
*传入参数：
*返 回 值：
*函数功能：
*****************************************************************************************/
void IoMsg_Init()
{
#ifdef MY_DBG_TRACE
	uIoMsgLevel = enIoMsgLevel_Infor |  enIoMsgLevel_Warn | enIoMsgLevel_Error | enIoMsgType_Trace;
#else
	uIoMsgLevel = enIoMsgLevel_Infor |  enIoMsgLevel_Warn | enIoMsgLevel_Error;
#endif
}


/*****************************************************************************************
*函数名称：
*传入参数：
*返 回 值：
*函数功能：
*****************************************************************************************/
void IoMsgPrintf(u32 uMsgType,const char* format, ...)
{
    static char buffer[IOMSG_MAX_LENGTH];
    va_list   vArgs;
	if(uIoMsgLevel & uMsgType)
	{
		va_start(vArgs, format);
		vsprintf((char *)buffer, (char const *)format, vArgs);
		va_end(vArgs);

        printf(buffer);
	}
}

void IoMsgBytes(u32 uMsgType,u8* pData,u32 bylen)
{
	u32 i=0;
#if MY_TRACE_BYTES
	if(uIoMsgLevel & uMsgType)
	{
		//process 16 bytes
		for(i=bylen/16;i>0;i--)
		{
			IoMsgPrintf(uMsgType,"\t%02X %02X %02X %02X %02X %02X %02X %02X ---- %02X %02X %02X %02X %02X %02X %02X %02X\r\n"
					,pData[0],pData[1],pData[2],pData[3],pData[4],pData[5],pData[6],pData[7]
					,pData[8],pData[9],pData[10],pData[11],pData[12],pData[13],pData[14],pData[15]);
			pData +=16;
		}

		//process other
		for(i=0;i<(bylen&0x0F);i++)
		{
			const char* pFmt=NULL;
			switch(i)
			{
			case 0:
				pFmt="\t%02X ";
				break;
			case 7:
				pFmt="%02X ---- ";
				break;
			default:
				if(i==((bylen&0x0F)-1))
					pFmt="%02X\r\n";
				else
					pFmt="%02X ";
				break;
			}
			IoMsgPrintf(uMsgType,pFmt,*pData);
			pData ++;
		}
		IoMsgPrintf(uMsgType,"\r\n");
	}
#endif
}


/*
 * MyTrace.h
 *
 *  Created on: 2016-6-13
 *      Author: ljy
 */

#ifndef MYTRACE_H_
#define MYTRACE_H_

#if defined (__cplusplus)
extern "C" {
#endif

#include "include/Typedef.h"

/*===================================================================================================================
										调试信息相关
===================================================================================================================*/

#define MY_TRACE_BYTES 1
#define MY_DBG_TRACE

//消息信息类别
typedef enum
{
	enIoMsgType_Infor	=1,
	enIoMsgType_Trace	=2,
	enIoMsgType_Warn	=4,
	enIoMsgType_Error	=8
} enIoMsgType;

//消息输出级别
typedef enum
{
	enIoMsgLevel_Infor	=1,
	enIoMsgLevel_Trace	=2,
	enIoMsgLevel_Warn	=4,
	enIoMsgLevel_Error	=8
} enIoMsgLevel;

//初始化调试相关
void IoMsg_Init();

//输出信息函数
void IoMsgPrintf(u32 uMsgType,const char* format, ...);

//===========以下带不tag
//普通信息
#define Io_Infor(...) IoMsgPrintf(enIoMsgType_Infor,__VA_ARGS__)
//跟踪信息
#define Io_Trace(...) IoMsgPrintf(enIoMsgType_Trace,__VA_ARGS__)
//警告信息
#define Io_Warn(...)  IoMsgPrintf(enIoMsgType_Warn,__VA_ARGS__)
//错误信息
#define Io_Error(...) IoMsgPrintf(enIoMsgType_Error,__VA_ARGS__)

//===========以下带 tag
//普通信息
#define IoMsg_Infor(...) IoMsgPrintf(enIoMsgType_Infor,"-I- "__VA_ARGS__)
//跟踪信息
#define IoMsg_Trace(...) IoMsgPrintf(enIoMsgType_Trace,"-T- "__VA_ARGS__)
//警告信息
#define IoMsg_Warn(...)  IoMsgPrintf(enIoMsgType_Warn,"-W- "__VA_ARGS__)
//错误信息
#define IoMsg_Error(...) IoMsgPrintf(enIoMsgType_Error,"-E- "__VA_ARGS__)


//字节流数据 -> 16进制字符串
void IoMsgBytes(u32 uMsgType,u8* pData,u32 bylen);

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* MYTRACE_H_ */

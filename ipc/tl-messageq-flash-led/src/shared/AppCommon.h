/*
 * Copyright (c) 2013-2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== AppCommon.h ========
 *
 */

#ifndef AppCommon__include
#define AppCommon__include
#if defined (__cplusplus)
extern "C" {
#endif


/*
 *  ======== Application Configuration ========
 */

/* notify commands 00 - FF */
#define APP_CMD_NOP             0x00
#define APP_CMD_SHUTDOWN        0xFF

#define APP_CMD_ON_D0           1
#define APP_CMD_ON_D1           2
#define APP_CMD_ON_D2           3
#define APP_CMD_ON_ALL          4

#define APP_CMD_OFF_D0          5
#define APP_CMD_OFF_D1          6
#define APP_CMD_OFF_D2          7
#define APP_CMD_OFF_D3          8
#define APP_CMD_OFF_ALL         9

typedef struct {
    MessageQ_MsgHeader  reserved;
    UInt32              cmd;
} App_Msg;

#define App_MsgHeapId           0
#define App_HostMsgQueName      "HOST:MsgQ:01"
#define App_SlaveMsgQueName     "%s:MsgQ:01"  /* %s is each slave's Proc Name */


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* AppCommon__include */

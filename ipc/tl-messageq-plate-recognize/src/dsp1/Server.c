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
 *  ======== Server.c ========
 *
 */

/* this define must precede inclusion of any xdc header file */
#define Registry_CURDESC Test__Desc
#define MODULE_NAME "Server"

/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>
#include <xdc/runtime/Types.h>  
#include <xdc/runtime/Timestamp.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/* package header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Cache.h>
#include <ti/ipc/remoteproc/Resource.h>
#include <ti/sysbios/family/c66/Cache.h>

#include "c6x.h"

/* local header files */
#include "../shared/AppCommon.h"

/* module header file */
#include "Server.h"
#include "PlateID.h"

/* module structure */
typedef struct {
    UInt16              hostProcId;         // host processor id
    MessageQ_Handle     slaveQue;           // created locally
} Server_Module;

/* private data */
Registry_Desc               Registry_CURDESC;
static Server_Module        Module;

#define WIDTH           768
#define HEIGHT          576
//#define WIDTH           1280
//#define HEIGHT          720
#define ALG_BUFFER_SIZE (8 * 1024 * 1024)
char recBuf[ALG_BUFFER_SIZE];


/*
 *  ======== Server_init ========
 */
Void Server_init(Void)
{
    Registry_Result result;

    /* register with xdc.runtime to get a diags mask */
    result = Registry_addModule(&Registry_CURDESC, MODULE_NAME);
    Assert_isTrue(result == Registry_SUCCESS, (Assert_Id)NULL);

    /* initialize module object state */
    Module.hostProcId = MultiProc_getId("HOST");
}


/*
 *  ======== Server_create ========
 */
Int Server_create()
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    /* enable some log events */
    Diags_setMask(MODULE_NAME"+EXF");

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(MultiProc_self()));
    Module.slaveQue = MessageQ_create(msgqName, &msgqParams);

    if (Module.slaveQue == NULL) {
        status = -1;
        goto leave;
    }

    Log_print0(Diags_INFO,"Server_create: server is ready");

leave:
    Log_print1(Diags_EXIT, "<-- Server_create: %d", (IArg)status);
    return (status);
}



UInt32 Utils_getCurTimeInMsec()
{
    #if 1
    static UInt32 cpuKhz = 500*1000; // default
    static Bool isInit = FALSE;

    Types_Timestamp64 ts64;
    UInt64 curTs;

    if(!isInit)
    {
        /* do this only once */

        Types_FreqHz cpuHz;

        isInit = TRUE;

        Timestamp_getFreq(&cpuHz);

        cpuKhz = cpuHz.lo / 1000; /* convert to Khz */
    }

    Timestamp_get64(&ts64);

    curTs = ((UInt64) ts64.hi << 32) | ts64.lo;

    return (UInt32)(curTs/(UInt64)cpuKhz);
    #else
    return Clock_getTicks();
    #endif
}

void CacheInit()
{
    Cache_Size cacheSize;

    cacheSize.l1pSize = Cache_L1Size_32K;
    cacheSize.l1dSize = Cache_L1Size_32K;
    cacheSize.l2Size  = Cache_L2Size_128K;

    Cache_setSize(&cacheSize);

    /* 配置缓存区域 */
    Cache_setMar((Ptr *)0x95000000, 0x4000000, Cache_Mar_ENABLE  | Cache_PFX); // 可缓存可预取区域
    //Cache_setMar((Ptr *)0x95400000, 0x3000000, Cache_Mar_ENABLE); // 可缓存可预取区域
    //Cache_setMar((Ptr *)0x98400000, 0x300000, Cache_Mar_ENABLE); // 可缓存可预取区域
    //Cache_setMar((Ptr *)0xA0000000, 0xC000000, Cache_Mar_ENABLE); // 可缓存可预取区域
    //Cache_setMar((Ptr *)0xA0000000, 0xC000000, Cache_Mar_DISABLE);            // 不可缓存可预取区域
}


/*
 *  ======== Server_exec ========
 */
Int Server_exec()
{
    Int                 status;
    Bool                running = TRUE;
    App_Msg *           msg;
    MessageQ_QueueId    queId;
    Int                 i;
    uint8_t *           data;
    UInt32              virtualAddrIn[4];
    UInt32              virtualAddrOut;

    Log_print0(Diags_ENTRY | Diags_INFO, "--> Server_exec:");
    //CacheInit();
    unsigned int *mar149 = (unsigned int *)0x1848254;
    unsigned int *mar150 = (unsigned int *)0x1848258;
    unsigned int *mar151 = (unsigned int *)0x184825C;
    unsigned int *mar152 = (unsigned int *)0x1848260;
    Log_print4(Diags_INFO, "mar149: %x  mar150: %x  mar151: %x  mar152: %x", *mar149, *mar150, *mar151, *mar152);

	int nRet = InitPlateID(recBuf, ALG_BUFFER_SIZE, WIDTH, HEIGHT);
    Log_print1(Diags_INFO, "InitPlateID ret: %d", nRet);

    while (running) {

        /* wait for inbound message */
        status = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }

        for(i = 0; i < 4; i++) {
            if(Resource_physToVirt(msg->dataInPhys[i], &virtualAddrIn[i]) != Resource_S_SUCCESS) {
                Log_print1(Diags_INFO, "Server_exec: not found resource for phys 0x%x\n",
                                        msg->dataInPhys[i]);
                continue;
            }
            //Log_print4(Diags_INFO, "Server_exec: addrPhys[%d] = 0x%x addrIn[%d]=0x%x",
            //           i, msg->dataInPhys[i], i, virtualAddrIn[i]);
            data = (uint8_t*)virtualAddrIn[i];

            long long start, stop;
            TSCL = 0; // need to write to it to start counting
            //start = TSCL;
            start = _itoll(TSCH, TSCL);	

	        int nRet = RecogImage(data, WIDTH, HEIGHT, NULL);

            //stop = TSCL;
            stop = _itoll(TSCH, TSCL);	
            Log_print1(Diags_INFO, "cost time: %d ms\n", (stop - start) / 750 / 1000);
        }

        if (Resource_physToVirt(msg->dataOutPhys, &virtualAddrOut) == Resource_S_SUCCESS) {
            Log_print1(Diags_INFO, "Server_exec: addrOut=0x%x", virtualAddrOut);

            msg->outXor = virtualAddrOut;
        } else {
            Log_print1(Diags_INFO, "Server_exec: not found resource for phys 0x%x\n", msg->dataOutPhys);
        }

        if (msg->cmd == App_CMD_SHUTDOWN) {
            running = FALSE;
        }

        /* process the message */
        Log_print1(Diags_INFO, "Server_exec: processed cmd=0x%x", msg->cmd);

        /* send message back */
        queId = MessageQ_getReplyQueue(msg); /* type-cast not needed */
        MessageQ_put(queId, (MessageQ_Msg)msg);
    } /* while (running) */

leave:
    Log_print1(Diags_EXIT, "<-- Server_exec: %d", (IArg)status);
    return(status);
}

/*
 *  ======== Server_delete ========
 */

Int Server_delete()
{
    Int         status;

    Log_print0(Diags_ENTRY, "--> Server_delete:");

    /* delete the video message queue */
    status = MessageQ_delete(&Module.slaveQue);

    if (status < 0) {
        goto leave;
    }

leave:
    if (status < 0) {
        Log_error1("Server_finish: error=0x%x", (IArg)status);
    }

    /* disable log events */
    Log_print1(Diags_EXIT, "<-- Server_delete: %d", (IArg)status);
    Diags_setMask(MODULE_NAME"-EXF");

    return(status);
}

/*
 *  ======== Server_exit ========
 */

Void Server_exit(Void)
{
    /*
     * Note that there isn't a Registry_removeModule() yet:
     *     https://bugs.eclipse.org/bugs/show_bug.cgi?id=315448
     *
     * ... but this is where we'd call it.
     */
}

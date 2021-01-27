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
 *  ======== App.c ========
 *
 */

/* host header files */
#include <stdio.h>
#include <unistd.h>

/* package header files */
#include <ti/ipc/Std.h>
#include <ti/ipc/MessageQ.h>

/* cmem header file */
#include <ti/cmem.h>

/* local header files */
#include "../shared/AppCommon.h"
#include "App.h"

/* module structure */
typedef struct {
    MessageQ_Handle         hostQue;    // created locally
    MessageQ_QueueId        slaveQue;   // opened remotely
    UInt16                  heapId;     // MessageQ heapId
    UInt32                  msgSize;
} App_Module;

/* private data */
static App_Module Module;


#define PAYLOADSIZE        (0x100000 * 16) // 16MB

/*
 *  ======== App_create ========
 */

Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

    printf("--> App_create:\n");

    status = CMEM_init();

    if (status < 0) {
        printf("CMEM_init failed\n");
        goto leave;
    }
    else {
        printf("CMEM_init success\n");
    }

    /* setting default values */
    Module.hostQue = NULL;
    Module.slaveQue = MessageQ_INVALIDMESSAGEQ;
    Module.heapId = App_MsgHeapId;
    Module.msgSize = sizeof(App_Msg);

    /* create local message queue (inbound messages) */
    MessageQ_Params_init(&msgqParams);

    Module.hostQue = MessageQ_create(App_HostMsgQueName, &msgqParams);

    if (Module.hostQue == NULL) {
        printf("App_create: Failed creating MessageQ\n");
        status = -1;
        goto leave;
    }

    /* open the remote message queue */
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(remoteProcId));

    do {
        status = MessageQ_open(msgqName, &Module.slaveQue);
        sleep(1);
    } while (status == MessageQ_E_NOTFOUND);

    if (status < 0) {
        printf("App_create: Failed opening MessageQ\n");
        goto leave;
    }

    printf("App_create: Host is ready\n");

leave:
    printf("<-- App_create:\n");
    return(status);
}


/*
 *  ======== App_delete ========
 */
Int App_delete(Void)
{
    Int         status;

    printf("--> App_delete:\n");

    /* close remote resources */
    status = MessageQ_close(&Module.slaveQue);

    if (status < 0) {
        goto leave;
    }

    /* delete the host message queue */
    status = MessageQ_delete(&Module.hostQue);

    if (status < 0) {
        goto leave;
    }

leave:
    printf("<-- App_delete:\n");
    return(status);
}


/*
 *  ======== App_exec ========
 */
Int App_exec(Void)
{
    Int         status = 0;
    Int         i, j;
    App_Msg *   msg;
    CMEM_AllocParams        cmemAttrs;

    cmemAttrs.type = CMEM_HEAP;
    cmemAttrs.flags =  CMEM_NONCACHED;
    cmemAttrs.alignment = 0;

    printf("--> App_exec:\n");

    /* allocate message */
    msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

    if (msg == NULL) {
        status = -1;
        goto leave;
    }

    msg->dataByteSize = PAYLOADSIZE;

    for (i = 0; i < 1; i++) {
        msg->dataIn[i] = CMEM_allocPool(CMEM_getPool(PAYLOADSIZE), &cmemAttrs);
         if (msg->dataIn[i] == NULL) {
             printf("CMEM_alloc() failed (returned NULL)\n");
             status = -1;
             goto leave;
         }
         msg->dataInPhys[i] = CMEM_getPhys(msg->dataIn[i]);

         printf("user: %p phys: 0x%x\n", msg->dataIn[i], msg->dataInPhys[i]);

         uint8_t *dataIn = (uint8_t*)msg->dataIn[i];

         msg->inXor[i] = 0;

         for (j = 0; j < PAYLOADSIZE; j++) {
             dataIn[j] = rand();
             msg->inXor[i] ^= dataIn[j];
         }
	 printf("Data %d in xor = 0x%x\n", i, msg->inXor[i]);
    }

    /* set the return address in the message header */
    MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

    /* fill in message payload */
    msg->cmd = App_CMD_SHUTDOWN;

    MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);

    printf("Message put\n");

    /* wait for return message */
    status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
        MessageQ_FOREVER);

    if (status < 0) {
        goto leave;
    }

    printf("Message get\n");
    printf("The size of data is %d MB\n", PAYLOADSIZE / 1024 / 1024);
    printf("slave data xor result 0x%x, host(arm) 0x%x\n", msg->outXor, msg->inXor[0]);

    CMEM_free(msg->dataIn[0], &cmemAttrs);
    MessageQ_free((MessageQ_Msg)msg);

leave:
    printf("<-- App_exec: %d\n", status);
    return(status);
}

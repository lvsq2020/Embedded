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
#include <stdint.h>
#include <stdlib.h>

/* package header files */
#include <ti/ipc/Std.h>
#include <ti/ipc/MessageQ.h>

#include <ti/cmem.h>


/* local header files */
#include "../shared/AppCommon.h"
#include "App.h"
#include "math.h"

/* module structure */
typedef struct {
    MessageQ_Handle         hostQue;    // created locally
    MessageQ_QueueId        slaveQue;   // opened remotely
    UInt16                  heapId;     // MessageQ heapId
    UInt32                  msgSize;
} App_Module;

/* private data */
static App_Module Module;

#define PAYLOADSIZE         (0x20000)   //0x1C0000 max for float;


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

    printf("MessageQ_Params_init\n");

    Module.hostQue = MessageQ_create(App_HostMsgQueName, &msgqParams);

    if (Module.hostQue == NULL) {
        printf("App_create: Failed creating MessageQ\n");
        status = -1;
        goto leave;
    }

    printf("MessageQ_create\n");

    /* open the remote message queue */
    sprintf(msgqName, App_SlaveMsgQueName, MultiProc_getName(remoteProcId));

    printf("queue name %s\n", msgqName);

    do {
        status = MessageQ_open(msgqName, &Module.slaveQue);
        printf("MessageQ_open\n");
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
    Int         status;
    Int         i, j;
    App_Msg *   msg;

    CMEM_AllocParams         cmemAttrs;
    cmemAttrs.type = CMEM_HEAP;
    cmemAttrs.flags =  CMEM_NONCACHED;
    cmemAttrs.alignment = 0;

    printf("--> App_exec:\n");

    /* fill process pipeline */
    printf("App_exec: sending message\n");
    /* allocate message */
    msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);
    if (msg == NULL) {
        status = -1;
        goto leave;
    }

    msg->dataByteSize = PAYLOADSIZE;

    msg->dataIn = CMEM_allocPool(CMEM_getPool(PAYLOADSIZE), &cmemAttrs);
    if (msg->dataIn == NULL) {
        printf("CMEM_alloc() failed (returned NULL)\n");
        status = -1;
        goto leave;
    }
    msg->dataInPhys = CMEM_getPhys(msg->dataIn);

    printf("user: %p phys: 0x%x\n", msg->dataIn, msg->dataInPhys);

    float *dataIn = (float*)msg->dataIn;

    for(j = 0; j < PAYLOADSIZE; j++) {
        dataIn[j] = sin (2 * 3.1415 * 50 * j / (double) PAYLOADSIZE);;
    }

    msg->dataOut = CMEM_allocPool(CMEM_getPool(PAYLOADSIZE), &cmemAttrs);
    if (msg->dataOut == NULL) {
        printf("CMEM_alloc() failed (returned NULL)\n");
        status = -1;
        goto leave;
    }
    msg->dataOutPhys = CMEM_getPhys(msg->dataOut);

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
    printf("user: %p phys: 0x%x\n", msg->dataOut, msg->dataOutPhys);
    
    /* extract message payload */
    float *dataOut = (float*)msg->dataOut;

    FILE *out_file =fopen("FFTData", "w");
    if(out_file == NULL)
    {
        printf("open file failed !\n");
        goto leave;
    }
    else
    {
	for(i = 0; i < PAYLOADSIZE; i++) 
	{
	    fprintf(out_file,"%f\n",dataOut[i]);
	    //printf("Dataout:%5f\n",dataOut[i]);
        }
	printf("FFT Complete!\n");
    	printf("Data Size:%5d\n",i);
    }
    fclose(out_file);

    /* free the message */
    CMEM_free(msg->dataIn, &cmemAttrs);
    CMEM_free(msg->dataOut, &cmemAttrs);
    MessageQ_free((MessageQ_Msg)msg);

leave:
    printf("<-- App_exec: %d\n", status);
    return(status);
}

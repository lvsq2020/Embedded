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
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

/* package header files */
#include <ti/ipc/Std.h>
#include <ti/ipc/MessageQ.h>

/* local header files */
#include "../shared/AppCommon.h"
#include "App.h"

/* module structure */
typedef struct {
    MessageQ_Handle         hostQue;    // created locally
    MessageQ_QueueId        slaveQue;   // opened remotely
    UInt16                  heapId;     // MessageQ heapId
    UInt32                  msgSize;    // Message size
} App_Module;

/* private data */
static App_Module Module;

/*
 *  ======== App_create ========
 */
Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    Char                msgqName[32];

    printf("--> App_create:\n");

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
Int App_exec(const Params *const params)
{
    Int         status;
    UInt32      i;
    UInt32      err_count = 0;
	float		write_rate = 0, read_rate = 0;
    App_Msg *   msg;

    printf("--> App_exec:\n");

    /* allocate message */
    msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);
    if (msg == NULL) {
        status = -1;
        goto leave;
    }

    /* set the reply address in the message header */
    MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

    /* send setup message,which is contain of address, data size */
    msg->cmd = App_CMD_SETUP;
    MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);

    /* wait for reply messages */
    status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg, MessageQ_FOREVER);
    if (status < 0)
        goto leave;

    /* if failed occur, shutdown the server */
    if (msg->trans_status < 0) {
        printf("--> App_exec: Server response setup command occur error!\r\n");
        goto err_shutdown;
    }

    /* start edma test for params->loop_times cycles */
    printf("--> App_exec: Start EDMA Test ...    \r\n\n\n");

    for (i = 0; i < params->loop_times; i++) {
        /* send message, start the edma test */
        msg->cmd = App_CMD_TEST;
		msg->devmem_phy_addr = params->address;
		msg->transfer_size = params->size;
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);

        /* wait for reply message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg, MessageQ_FOREVER);
		if (status < 0)
			goto leave;

        /* if failed occur, shutdown the server */
        if (msg->trans_status < 0) {
            printf("--> App_exec: Server EDMA Test occur error at %dth times !\r\n", i);
            goto err_shutdown;
        }

		err_count += msg->err_count;

		/* Calculation server edma transfer data speed */
		/* size: KB, time: us; 0.95 = 1000000 / 1024 /1024; 1s = 1000000 us; 1MB = 1024 * 1024B;  */
		write_rate = (float)msg->transfer_size * 0.95 / msg->write_timeuse;
		read_rate = (float)msg->transfer_size * 0.95 / msg->read_timeuse;

        printf("\033[2A--> loop times: %d\r\n"
                "--> EDMA Communication block size: %d KB, errcnt: %d, "
                "average write time: %d us (%.2f MB/s), average read time: %d us (%.2f MB/s)\r\n", \
                i + 1, msg->transfer_size / 1024, msg->err_count, \
                msg->write_timeuse, write_rate, msg->read_timeuse, read_rate);
    }

    printf("--> App_exec: EDMA Test %d cycles complete, errcnt: %d, total size: %d KB \r\n",
            params->loop_times, err_count, params->size / 1024 * params->loop_times);

err_shutdown:
    /* send message, shutdown the data transmit */
    msg->cmd = App_CMD_SHUTDOWN;
    MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);

    /* wait for return message */
    status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg, MessageQ_FOREVER);
	if (status < 0)
		goto leave;

    /* free message */
    MessageQ_free((MessageQ_Msg)msg);


leave:
    printf("<-- App_exec: %d\n", status);
    return(status);
}

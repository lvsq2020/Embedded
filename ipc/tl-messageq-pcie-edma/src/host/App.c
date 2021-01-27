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
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>

/* package header files */
#include <ti/ipc/Std.h>
#include <ti/ipc/MessageQ.h>

/* local header files */
#include "../shared/AppCommon.h"
#include "App.h"

int fd;
int fd_save;
char *addr;

#define SEND_BUF_ADDRESS 0xB0000000
#define RECVICE_BUF_ADDRESS 0xB0500000
#define H2C_DESC_ADDRESS 0xB0A00000
#define C2H_DESC_ADDRESS 0xB0B00000
#define FPAG_DATA_SIZE   0x400000        // 4MB

#define PCIE_DATA_SAVEFILE  "/dev/shm/pcie_save"

/* module structure */
typedef struct {
    MessageQ_Handle         hostQue;    // created locally
    MessageQ_QueueId        slaveQue;   // opened remotely
    UInt16                  heapId;     // MessageQ heapId
    UInt32                  msgSize;
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
    char                msgqName[32];
    off_t               pa_offset;

    printf("--> App_create:\n");

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	    return -1;

    printf("/dev/mem opened.\n");
    fflush(stdout);

    pa_offset = RECVICE_BUF_ADDRESS & ~(sysconf(_SC_PAGE_SIZE) - 1);
    addr = mmap(NULL, FPAG_DATA_SIZE - pa_offset, PROT_READ,
		    MAP_PRIVATE, fd, pa_offset);
    if (addr == MAP_FAILED) {
    	close(fd);
	printf("Memory mapped error %p.\n", RECVICE_BUF_ADDRESS);
    }else
	printf("Memory mapped at address %p.\n", addr);

    if ((fd_save = open(PCIE_DATA_SAVEFILE, O_RDWR | O_CREAT)) == -1)
	    return -1;
    printf("%s opened.\n", PCIE_DATA_SAVEFILE);

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

    close(fd_save);
    munmap(addr, FPAG_DATA_SIZE);
    close(fd);

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
    Int         i;
    Int		offset;
    App_Msg *   msg;
    time_t      tBeginTime, tEndTime;
    double      fCostTime;

    printf("--> App_exec:\n");
    /* process steady state (keep pipeline full) */
    for (i = 1; i <= 2; i++) {
	printf("App_exec: sending message %d\n", i);
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

	if (i == 1) {
	    msg->cmd = App_CMD_SETUP;
	    msg->send_buf_address = (char*)SEND_BUF_ADDRESS;
	    msg->recvice_buf_address = (char*)RECVICE_BUF_ADDRESS;
	    msg->h2c_desc_address = (char*)H2C_DESC_ADDRESS;
	    msg->c2h_desc_address = (char*)C2H_DESC_ADDRESS;
	}
	else if (i == 2) {
	    msg->cmd = APP_CMD_TRANSFER;
	}
        else {
            msg->cmd = App_CMD_NOP;
        }

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
    }

    /* drain process pipeline */
    for (i = 1; i <= 2; i++) {
        printf("App_exec: message received\n");

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }

	if (msg->cmd == APP_CMD_TRANSFER){
		offset = msg->recvice_buf - RECVICE_BUF_ADDRESS;
		tBeginTime = clock();
		if (write(fd_save, addr + offset, msg->buf_size) < msg->buf_size)
			printf("write file error\n");
		tEndTime = clock();
		fCostTime = difftime(tEndTime, tBeginTime);
		printf("file write times %d KB cost: %f Sec\, rate %fMB/s\n",
				msg->buf_size / 1024, fCostTime / CLOCKS_PER_SEC, (float)(msg->buf_size) / 1024 / 1024 / (fCostTime / CLOCKS_PER_SEC));
		MessageQ_free((MessageQ_Msg)msg);
		goto leave;
	}

        /* extract message payload */

        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);
    }

leave:
    printf("<-- App_exec: %d\n", status);
    return(status);
}

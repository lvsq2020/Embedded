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
    UInt32                  msgSize;
} App_Module;

/* private data */
static App_Module Module;

#define FPAG_DATA_ADDR		0x99300000
#define FPAG_DATA_SIZE		(FPAG_DATA_BLOCKS * 0x1000)	// 4KB * 8
#define FPAG_DATA_BLOCKS	8

#define GPMC_DATA_SAVEFILE  "/dev/shm/gpmc_save"

int fd;
int fd_save;
char *addr;

void __attribute__ ((noinline))
neon_copy(char *dst, char *src, size_t size) {
    /**/
    if (size & 0x3f)
        size = (size & (~ 0x3f)) + 64;

    /* NEON memory copy with preload */
    asm volatile (
        "NEONCopyPLD:               \n"
        "   PLD [r1, #0xC0]         \n"
        "   VLDM %[src]!,{d0-d7}        \n"
        "   VSTM %[dst]!,{d0-d7}        \n"
        "   SUBS %[size],%[size],#0x40  \n"
        "   BGT NEONCopyPLD         \n"
        : [dst]"+r"(dst), [src]"+r"(src), [size]"+r"(size)
        : : "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "cc", "memory");
}

/*
 *  ======== App_create ========
 */

Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];
    off_t 		pa_offset;

    printf("--> App_create:\n");

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
    	return -1;
    
    printf("/dev/mem opened.\n");
    fflush(stdout);

    pa_offset = FPAG_DATA_ADDR & ~(sysconf(_SC_PAGE_SIZE) - 1);

    addr = mmap(NULL, FPAG_DATA_SIZE - pa_offset, PROT_READ,
		MAP_PRIVATE, fd, pa_offset);
    if (addr == MAP_FAILED) {
        close(fd);
	printf("Memory mapped error %p.\n", FPAG_DATA_ADDR);
    } else
    	printf("Memory mapped at address %p.\n", addr);

    if ((fd_save = open(GPMC_DATA_SAVEFILE, O_RDWR | O_CREAT)) == -1)
    	return -1;

    printf("%s opened.\n", GPMC_DATA_SAVEFILE);

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

    close(fd_save);
    munmap(addr, FPAG_DATA_SIZE);
    close(fd);

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
    App_Msg *   msg;
    time_t 	tBeginTime, tEndTime;
    double 	fCostTime;
    Int offset;

    printf("--> App_exec:\n");

    /* fill process pipeline */
    for (i = 1; i <= FPAG_DATA_BLOCKS; i++) {
        printf("App_exec: sending message %d\n", i);

        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        /* fill in message payload */
        msg->cmd = App_CMD_SETUP;
	msg->addr = FPAG_DATA_ADDR + (i - 1) * (FPAG_DATA_SIZE / FPAG_DATA_BLOCKS);
	msg->size = FPAG_DATA_SIZE / FPAG_DATA_BLOCKS;

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
    }

    /* process steady state (keep pipeline full) */
    for (i = (FPAG_DATA_BLOCKS + 1); i <= 24; i++) {

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

	printf("App_exec: MessageQ_get msg addr %p, cmd 0x%x\n", msg->addr, msg->cmd);
        if (status < 0) {
            goto leave;
        }

        /* extract message payload */
	if (msg->cmd == App_CMD_NOP) {

		if (i == 17)
			tBeginTime = clock();

		offset = msg->addr - FPAG_DATA_ADDR;
		if (write(fd_save, addr + offset, msg->size) < msg->size)
			printf("write file error\n");

		if (i == 24) {
			tEndTime = clock();
			fCostTime = difftime(tEndTime, tBeginTime);
			printf("file write times %d KB cost: %f Sec\, rate %fMB/s\n",
			FPAG_DATA_SIZE / 1024, fCostTime / CLOCKS_PER_SEC, (float)FPAG_DATA_SIZE / 1024 / 1024 / (fCostTime / CLOCKS_PER_SEC));
		}
	}
		
        /* free the message */
        MessageQ_free((MessageQ_Msg)msg);

        printf("App_exec: message received, sending message %d\n", i);

        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);

        if (msg == NULL) {
            status = -1;
            goto leave;
        }

        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);

        /* fill in message payload */
        if (i == 24) {
            /* Last message will tell the slave to shutdown */
            msg->cmd = App_CMD_SHUTDOWN;
        }
        else {
            msg->cmd = App_CMD_NOP;
        }

        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
    }

    /* drain process pipeline */
    for (i = 1; i <= FPAG_DATA_BLOCKS; i++) {
        printf("App_exec: message received\n");

        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
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

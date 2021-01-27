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

#include <stdio.h>
#include <stdlib.h>

/* package header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/remoteproc/Resource.h>
#include <ti/ipc/ListMP.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* local header files */
#include "../shared/AppCommon.h"
#include "GPMCEDMA.h"

/* module header file */
#include "Server.h"

/* module structure */
typedef struct {
    UInt16              hostProcId;         // host processor id
    MessageQ_Handle     slaveQue;           // created locally
    ListMP_Handle       ListMP_addrHandle;
    ListMP_Handle       ListMP_dataHandle;
} Server_Module;

/* private data */
Registry_Desc               Registry_CURDESC;
static Server_Module        Module;

typedef struct ListMP_Node_Tag {
    ListMP_Elem elem;               /* the first variable in your data structure
                                     * must be of type ListMP_Elem 
                                     */
    UInt32	index;
    UInt32	addr;
    UInt32	Virt_addr;
    UInt32	size;
    UInt32	last_flag;
} ListMP_Node;

ListMP_Node*	ListMP_node[8];
UInt32		node_conter = 0;
UInt32		addr_conter = 0;
UInt32		run_conter;

/* Global variable timers for throughput */
uint64_t totalDMATime = 0;

EDMA3_DRV_Handle hEdma = NULL;


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
    ListMP_Params       listParams;

    /* enable some log events */
    Diags_setMask(MODULE_NAME"+EXF");

    hEdma = edmaInit(hEdma);
    if (hEdma == NULL)
	Log_print0(Diags_INFO,"ERROR: EDMA handle not initialized!\n");

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




/*
 *  ======== Server_exec ========
 */
Int Server_exec()
{
    Int                 status;
    Bool                running = TRUE;
    App_Msg *           msg;
    MessageQ_QueueId    queId;
    ListMP_Node*        node;
    unsigned long 	totalTime=0;
    unsigned long*	totalTimePointer;
    int			ACount, BCount, CCount;

    Log_print0(Diags_ENTRY | Diags_INFO, "--> Server_exec:");

    run_conter = 0;
    node_conter = 0;
    addr_conter = 0;

    while (running) {

        /* wait for inbound message */
        status = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,
            MessageQ_FOREVER);

        if (status < 0) {
            goto leave;
        }

        if (msg->cmd == App_CMD_SHUTDOWN) {
            running = FALSE;
        } else if (msg->cmd == App_CMD_SETUP) {
		Log_print1(Diags_INFO, "Server_exec: alloc addr = 0x%x", msg->addr);
		Log_error0("Server_exec: Failed to malloc node ListMP");
		//Resource_virtToPhys(0x95200000, &msg->addr);
		

		node = malloc(sizeof(ListMP_Node));
		if (node == NULL) {
			Log_error0("Server_exec: Failed to malloc node ListMP");
		} else {
			node->addr = msg->addr;
			node->size = msg->size;
			Resource_physToVirt(msg->addr, &node->Virt_addr);
			ListMP_node[node_conter] = node;
			node_conter++;

#if 0
			/* 2. add node to the beginning of ListMP */
			status = ListMP_putHead(Module.ListMP_addrHandle,(ListMP_Elem *) node);
			if(status < 0) {
				Log_error0("Server_exec: Failed to malloc node ListMP");

			    goto leave;
			}
#endif
		}
	} else if (msg->cmd == App_CMD_NOP) {
#if 0
		/* grab all nodes from the remote ListMP */
		while (ListMP_empty(Module.ListMP_addrHandle) == FALSE) {
			;
		}
		/* 2. get node from the tail of the remote ListMP */
		node = (ListMP_Node*) ListMP_getTail(Module.ListMP_addrHandle);
#endif
		node = ListMP_node[addr_conter];

		run_conter++;
		//memset(node->Virt_addr, run_conter, node->size);
		memset(0x99600000, run_conter, node->size);

		/*
		 * Setting up EDMA parameters
		 *
		 * The following expressions take care of overflow
		 *
		 */

		ACount = 4096;
		BCount = 1;
		CCount = 1;

		totalTimePointer=&totalTime;
		*totalTimePointer=0;

		edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) 0x9f000000, (unsigned int*) node->addr,
			     ACount, BCount, CCount, EDMA3_DRV_SYNC_A,totalTimePointer);

		totalDMATime += *totalTimePointer;

		msg->addr = node->addr;
		msg->size = node->size;
		addr_conter++;
		if (addr_conter >= 8)
			addr_conter = 0;

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
    Int		i;

    Log_print0(Diags_ENTRY, "--> Server_delete:");

    for (i = 0; i < 8; i++)
    	free(ListMP_node[i]);

    node_conter = 0;

    edmaDeinit(hEdma);

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

/* host header files */
#include <stdio.h>
#include <unistd.h>

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

/*
 *  ======== App_create ========
 */

Int App_create(UInt16 remoteProcId)
{
    Int                 status = 0;
    MessageQ_Params     msgqParams;
    char                msgqName[32];

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

Int App_exec_1(Void)
{
    Int         status;
    App_Msg *   msg;
    while(1){
        /* allocate message */
        msg = (App_Msg *)MessageQ_alloc(Module.heapId, Module.msgSize);
        if (msg == NULL) {
            status = -1;
            goto leave;
        }
        /* set the return address in the message header */
        MessageQ_setReplyQueue(Module.hostQue, (MessageQ_Msg)msg);
        /* fill in message payload */
        msg->cmd = App_CMD_NOP;
        /* send message */
        MessageQ_put(Module.slaveQue, (MessageQ_Msg)msg);
	
	sleep(1);
    }
leave:
    return(status);
}

Int App_exec_2(Void)
{
    Int         status;
    App_Msg *   msg;
    while(1){
        /* wait for return message */
        status = MessageQ_get(Module.hostQue, (MessageQ_Msg *)&msg,MessageQ_FOREVER);
        if (status < 0) {
            goto leave;
        }
        if(msg->cmd == App_CMD_TSK1_DSP_PUT){
            printf("message received tsk1 cmd=0x%x\n", msg->cmd);
	    MessageQ_free((MessageQ_Msg)msg);
        }
	if(msg->cmd == App_CMD_TSK2_DSP_PUT){
            printf("message received tsk2 cmd=0x%x\n", msg->cmd);
	    MessageQ_free((MessageQ_Msg)msg);
        }
    }
leave:
    return(status);
}

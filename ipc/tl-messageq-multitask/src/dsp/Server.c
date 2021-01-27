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

/* package header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* local header files */
#include "../shared/AppCommon.h"

/* module header file */
#include "Server.h"

/* module structure */
typedef struct {
    UInt16              hostProcId;         // host processor id
    MessageQ_Handle     slaveQue;           // created locally
} Server_Module;

/* private data */
Registry_Desc           Registry_CURDESC;
static Server_Module    Module;

Int                     stat;
App_Msg *               msg;
MessageQ_QueueId        queId;

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

/*
 *  ======== Tsk1_server_exec ========
 */

Int Tsk1_server_exec()
{
    /* wait for inbound message */
    stat = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,MessageQ_FOREVER);
    if (stat < 0) {
        Log_print1(Diags_EXIT, "<-- Tsk1_server_exec: %d", (IArg)stat);
        return(stat);
    }
    /* process the message */
    if(msg->cmd == App_CMD_NOP){
        Log_print1(Diags_INFO, "tsk1 processed cmd=0x%x", msg->cmd);
        /* send message back */
        queId = MessageQ_getReplyQueue(msg); /* type-cast not needed */
        msg->cmd = App_CMD_TSK1_DSP_PUT;
        MessageQ_put(queId, (MessageQ_Msg)msg);
    }
    return(stat);
}

/*
 *  ======== Tsk2_server_exec ========
 */

Int Tsk2_server_exec()
{
    msg = (App_Msg *)MessageQ_alloc(App_MsgHeapId, sizeof(App_Msg));
    if (msg == NULL) {
        return(-1);
    }
    msg->cmd = App_CMD_TSK2_DSP_PUT;
    MessageQ_put(queId, (MessageQ_Msg)msg);
    Log_print1(Diags_INFO, "tsk2 send cmd=0x%x", msg->cmd);

    /** make a block for test **/
    /**/stat = MessageQ_get(Module.slaveQue, (MessageQ_Msg *)&msg,MessageQ_FOREVER);
    /**/if (stat < 0) {
    /**/    Log_print1(Diags_EXIT, "<-- Tsk2_server_exec: %d", (IArg)stat);
    /**/    return(stat);
    /**/}
    /**/MessageQ_free((MessageQ_Msg)msg);
    /***************************/
    return(1);
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

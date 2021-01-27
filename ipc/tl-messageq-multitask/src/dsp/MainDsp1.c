/* xdctools header files */
#include <xdc/std.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>

/* package header files */
#include <ti/ipc/Ipc.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h> 
#include <ti/sysbios/knl/Semaphore.h>  

/* local header files */
#include "Server.h"
#include "rsc_table_dsp1.h"

/* private functions */
static Void tsk1_func(UArg arg0, UArg arg1);

static Void tsk2_func(UArg arg0, UArg arg1);

Task_Handle tsk1,tsk2;

Semaphore_Handle sem;

Bool running = TRUE;

/*
 *  ======== main ========
 */
Int main(Int argc, Char* argv[])
{
    Error_Block     eb;
    Task_Params     taskParams1;
    Task_Params     taskParams2;

    Log_print0(Diags_ENTRY, "--> main:");

    /* initialize modules */
    Server_init();

    /* must initialize the error block before using it */
    Error_init(&eb);

    if (Server_create() < 0) {
        return(-1);
    }

    /* create main thread (interrupts not enabled in main on BIOS) */
    Task_Params_init(&taskParams1);
    taskParams1.instance->name = "tsk1_func";
    taskParams1.arg0 = (UArg)argc;
    taskParams1.arg1 = (UArg)argv;
    taskParams1.stackSize = 0x1000;
    taskParams1.priority = 2;

    Task_Params_init(&taskParams2);
    taskParams2.instance->name = "tsk2_func";
    taskParams2.arg0 = (UArg)argc;
    taskParams2.arg1 = (UArg)argv;
    taskParams2.stackSize = 0x1000;
    taskParams2.priority = 2;

    tsk1 = Task_create(tsk1_func, &taskParams1, &eb);
    Log_print0(Diags_INFO, "tsk1:create");
    tsk2 = Task_create(tsk2_func, &taskParams2, &eb);
    Log_print0(Diags_INFO, "tsk2:create");

    if (Error_check(&eb)) {
        System_abort("main: failed to create application startup thread");
    }

    /* 
    * Create semaphore with initial count = 0 and default params 
    */  
    sem = Semaphore_create(0,NULL,&eb);
    if(sem == NULL){
	System_abort("Semphore create failed");
    }
    /* start scheduler, this never returns */
    BIOS_start();

    /* should never get here */
    Log_print0(Diags_EXIT, "<-- main:");
    return (0);
}

/*
 *  ======== tsk1_func ========
 */
Void tsk1_func(UArg arg0, UArg arg1)
{
    Diags_setMask("Server+F");
    /* loop forever */
    while (running) {
        while(1){
            if (Tsk1_server_exec() < 0) {
                goto leave;
            } 
	    Semaphore_post(sem); 
        }
        /* server shutdown phase */
        if (Server_delete() < 0) {
            goto leave;
        }
    } /* while (running) */
    /* finalize modules */
    Server_exit();
leave:
    Log_print0(Diags_EXIT, "<-- tsk1_func");
    return;
}

/*
 *  ======== tsk2_func ========
 */
Void tsk2_func(UArg arg0, UArg arg1)
{
    Semaphore_pend(sem, BIOS_WAIT_FOREVER);  
    Diags_setMask("Server+F");
    while(running){
	while(1){
            if (Tsk2_server_exec() < 0) {
                goto leave;
            }
	}
        /* server shutdown phase */
        if (Server_delete() < 0) {
            goto leave;
        }
    } /* while (running) */
    /* finalize modules */
    Server_exit();
leave:
    Log_print0(Diags_EXIT, "<-- tsk2_func");
    return;
}

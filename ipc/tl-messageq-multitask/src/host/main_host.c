/* cstdlib header files */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

/* package header files */
#include <ti/ipc/Std.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/transports/TransportRpmsg.h>

#include <ti/ipc/MultiProc.h>

/* local header files */
#include "App.h"

/* private functions */
static Int Main_main(Void);
static Int Main_parseArgs(Int argc, Char *argv[]);
static void * Dsp_msg_handle(void);


#define Main_USAGE "\
Usage:\n\
    app_host [options] procName\n\
\n\
Arguments:\n\
    procName      : the name of the remote processor\n\
\n\
Options:\n\
    h   : print this help message\n\
    l   : list the available remote names\n\
\n\
Examples:\n\
    app_host DSP\n\
    app_host -l\n\
    app_host -h\n\
\n"

/* private data */
static String   Main_remoteProcName = NULL;


/*
 *  ======== main ========
 */
Int main(Int argc, Char* argv[])
{
    Int status;

    printf("--> main:\n");

    /* configure the transport factory */
    Ipc_transportConfig(&TransportRpmsg_Factory);

    /* parse command line */
    status = Main_parseArgs(argc, argv);

    if (status < 0) {
        goto leave;
    }

    /* Ipc initialization */
    status = Ipc_start();

    if (status >= 0) {
        /* application create, exec, delete */
        status = Main_main();

        /* Ipc finalization */
        Ipc_stop();
    }
    else {
        printf("Ipc_start failed: status = %d\n", status);
        goto leave;
    }

leave:
    printf("<-- main:\n");
    status = (status >= 0 ? 0 : status);

    return (status);
}


/*
 *  ======== Main_main ========
 */
Int Main_main(Void)
{
    pthread_t   id; 

    UInt16      remoteProcId;
    Int         status = 0;

    printf("--> Main_main:\n");

    remoteProcId = MultiProc_getId(Main_remoteProcName);

    /* application create phase */
    status = App_create(remoteProcId);

    if (status < 0) {
        goto leave;
    }

    if(pthread_create(&id, NULL, (void *)Dsp_msg_handle,NULL)){  
        printf("Failure of thread creation !\n");  
        return 1;  
    }

    /* application execute phase */
    status = App_exec_1();

    if (status < 0) {
        goto leave;
    }

    pthread_join(id,NULL); 

    /* application delete phase */
    status = App_delete();

    if (status < 0) {
        goto leave;
    }

leave:
    printf("<-- Main_main:\n");

    status = (status >= 0 ? 0 : status);
    return (status);
}



void * Dsp_msg_handle(void){
    if (App_exec_2() < 0) {
        printf("<-- Thread leave:\n");
    }
    return (void *)0;
}


/*
 *  ======== Main_parseArgs ========
 */
Int Main_parseArgs(Int argc, Char *argv[])
{
    Int             x, cp, opt, argNum;
    UInt16          i, numProcs;
    String          name;
    Int             status = 0;


    /* parse the command line options */
    for (opt = 1; (opt < argc) && (argv[opt][0] == '-'); opt++) {
        for (x = 0, cp = 1; argv[opt][cp] != '\0'; cp++) {
            x = (x << 8) | (int)argv[opt][cp];
        }

        switch (x) {
            case 'h': /* -h */
                printf("%s", Main_USAGE);
                exit(0);
                break;

            case 'l': /* -l */
                printf("Processor List\n");
                status = Ipc_start();
                if (status >= 0) {
                    numProcs = MultiProc_getNumProcessors();
                    for (i = 0; i < numProcs; i++) {
                        name = MultiProc_getName(i);
                        printf("    procId=%d, procName=%s\n", i, name);
                    }
                    Ipc_stop();
                }
                else {
                    printf(
                        "Error: %s, line %d: Ipc_start failed\n",
                        __FILE__, __LINE__);
                    goto leave;
                }
                exit(0);
                break;

            default:
                printf(
                    "Error: %s, line %d: invalid option, %c\n",
                    __FILE__, __LINE__, (Char)x);
                printf("%s", Main_USAGE);
                status = -1;
                goto leave;
        }
    }

    /* parse the command line arguments */
    for (argNum = 1; opt < argc; argNum++, opt++) {

        switch (argNum) {
            case 1: /* name of proc #1 */
                Main_remoteProcName = argv[opt];
                break;

            default:
                printf(
                    "Error: %s, line %d: too many arguments\n",
                    __FILE__, __LINE__);
                printf("%s", Main_USAGE);
                status = -1;
                goto leave;
        }
    }

    /* validate command line arguments */
    if (Main_remoteProcName == NULL) {
        printf("Error: missing procName argument\n");
        printf("%s", Main_USAGE);
        status = -1;
        goto leave;
    }

leave:
    return(status);
}

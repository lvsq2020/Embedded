#ifndef Server__include
#define Server__include

#if defined (__cplusplus)
extern "C" {
#endif

Void Server_init(Void);
Void Server_exit(Void);

Int Server_create(Void);
Int Tsk1_server_exec(Void);
Int Tsk2_server_exec(Void);
Int Server_delete(Void);

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* Server__include */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#define SYS_MBOX_NULL NULL
#define SYS_SEM_NULL  NULL

struct sys_sem;
typedef struct sys_sem * sys_sem_t;
#define sys_sem_valid(sem)             (((sem) != NULL) && (*(sem) != NULL))
#define sys_sem_valid_val(sem)         ((sem) != NULL)
#define sys_sem_set_invalid(sem)       do { if((sem) != NULL) { *(sem) = NULL; }}while(0)
#define sys_sem_set_invalid_val(sem)   do { (sem) = NULL; }while(0)

struct sys_mutex;
typedef struct sys_mutex * sys_mutex_t;
#define sys_mutex_valid(mutex)         sys_sem_valid(mutex)
#define sys_mutex_set_invalid(mutex)   sys_sem_set_invalid(mutex)

struct sys_mbox;
typedef struct sys_mbox * sys_mbox_t;
#define sys_mbox_valid(mbox)           sys_sem_valid(mbox)
#define sys_mbox_valid_val(mbox)       sys_sem_valid_val(mbox)
#define sys_mbox_set_invalid(mbox)     sys_sem_set_invalid(mbox)
#define sys_mbox_set_invalid_val(mbox) sys_sem_set_invalid_val(mbox)

typedef void *sys_thread_t;

void sys_mark_tcpip_thread(void);
#define LWIP_MARK_TCPIP_THREAD()   sys_mark_tcpip_thread()

#if LWIP_TCPIP_CORE_LOCKING
void sys_lock_tcpip_core(void);
#define LOCK_TCPIP_CORE()          sys_lock_tcpip_core()
void sys_unlock_tcpip_core(void);
#define UNLOCK_TCPIP_CORE()        sys_unlock_tcpip_core()
#endif

struct timeval;

#ifdef __cplusplus
}
#endif


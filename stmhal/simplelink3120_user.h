#ifndef __USER_H__
#define __USER_H__

#ifdef  __cplusplus
extern "C" {
#endif

#include <string.h>
//#include <ti/drivers/net/wifi/porting/cc_pal.h>
typedef int Fd_t;
typedef void (*SL_P_EVENT_HANDLER)(unsigned int index);
#define P_EVENT_HANDLER SL_P_EVENT_HANDLER

int spi_Close(Fd_t Fd);
Fd_t spi_Open(char* pIfName , unsigned long flags);

// Contrary to the documentation pBuff is unsigned char type not char type
int spi_Read(Fd_t Fd , unsigned char* pBuff , int Len);
int spi_Write(Fd_t Fd , unsigned char* pBuff , int Len);
void NwpPowerOnPreamble(void);
void NwpPowerOn(void);
void NwpPowerOff(void);

int NwpRegisterInterruptHandler(SL_P_EVENT_HANDLER InterruptHdl , void* pValue);
void NwpMaskInterrupt();
void NwpUnMaskInterrupt();

typedef Fd_t   _SlFd_t;

#define SL_TIMESTAMP_TICKS_IN_10_MILLISECONDS     (10)
#define SL_TIMESTAMP_MAX_VALUE                    (0xFFFFFFFF)

#ifndef SL_TINY_EXT
#define MAX_CONCURRENT_ACTIONS 10
#else
#define MAX_CONCURRENT_ACTIONS 1
#endif


#define SL_RUNTIME_EVENT_REGISTERATION
#define SL_INC_ARG_CHECK
#define SL_INC_INTERNAL_ERRNO
#define SL_INC_EXT_API
#define SL_INC_WLAN_PKG
#define SL_INC_SOCKET_PKG
#define SL_INC_NET_APP_PKG
#define SL_INC_NET_CFG_PKG
#define SL_INC_NVMEM_PKG
#define SL_INC_NVMEM_EXT_PKG
#define SL_INC_SOCK_SERVER_SIDE_API
#define SL_INC_SOCK_CLIENT_SIDE_API
#define SL_INC_SOCK_RECV_API
#define SL_INC_SOCK_SEND_API

#define sl_DeviceEnablePreamble()   NwpPowerOnPreamble()
#define sl_DeviceEnable()           NwpPowerOn()
#define sl_DeviceDisable()          NwpPowerOff()

#define sl_IfOpen                   spi_Open
#define sl_IfClose                  spi_Close
#define sl_IfRead                   spi_Read
#define sl_IfWrite                  spi_Write
#define sl_IfRegIntHdlr(InterruptHdl, pValue) NwpRegisterInterruptHandler(InterruptHdl , pValue)
#define sl_IfMaskIntHdlr()          NwpMaskInterrupt()
#define sl_IfUnMaskIntHdlr()        NwpUnMaskInterrupt()
/*
#define SL_START_WRITE_STAT
*/

#ifdef SL_START_WRITE_STAT
#define sl_IfStartWriteSequence
#define sl_IfEndWriteSequence
#endif

#ifndef SL_TINY_EXT
#undef slcb_GetTimestamp
#define slcb_GetTimestamp           NwpSystemTicks
#endif

#define WAIT_NWP_SHUTDOWN_READY

#ifndef SL_INC_INTERNAL_ERRNO
#define slcb_SetErrno
#endif

//#define SL_PLATFORM_MULTI_THREADED

#ifdef SL_PLATFORM_MULTI_THREADED

#define SL_OS_RET_CODE_OK                       ((int)OS_OK)
#define SL_OS_WAIT_FOREVER                      ((time_t)OS_WAIT_FOREVER)
#define SL_OS_NO_WAIT                            ((time_t)OS_NO_WAIT)
#define _SlTime_t                time_t

#endif

/*
#define _SlSyncObj_t                SemaphoreP_Handle
#define sl_SyncObjCreate(pSyncObj,pName)            Semaphore_create_handle(pSyncObj)
#define sl_SyncObjDelete(pSyncObj)                  SemaphoreP_delete(*(pSyncObj))
#define sl_SyncObjSignal(pSyncObj)                 SemaphoreP_post(*(pSyncObj))
#define sl_SyncObjSignalFromIRQ(pSyncObj)           SemaphoreP_post(*(pSyncObj))
#define sl_SyncObjWait(pSyncObj,Timeout)            SemaphoreP_pend((*(pSyncObj)),Timeout)
#define _SlLockObj_t            MutexP_Handle
#define sl_LockObjCreate(pLockObj, pName)     Mutex_create_handle(pLockObj)
#define sl_LockObjDelete(pLockObj)                  MutexP_delete(*(pLockObj))
#define sl_LockObjLock(pLockObj,Timeout)          Mutex_lock(*(pLockObj))
#define sl_LockObjUnlock(pLockObj)                   Mutex_unlock(*(pLockObj))
*/

#define __NON_OS_SYNC_OBJ_CLEAR_VALUE               ((int*)0x11)
#define __NON_OS_SYNC_OBJ_SIGNAL_VALUE              ((int*)0x22)
#define __NON_OS_LOCK_OBJ_UNLOCK_VALUE              ((int*)0x33)
#define __NON_OS_LOCK_OBJ_LOCK_VALUE                ((int*)0x44)
typedef int *_SlNonOsSemObj_t; // SL requires that _SlSyncObj_t be a pointer type
extern int _SlNonOsSemGet(_SlNonOsSemObj_t* pSyncObj, _SlNonOsSemObj_t WaitValue, _SlNonOsSemObj_t SetValue, unsigned int Timeout);
extern int _SlNonOsSemSet(_SlNonOsSemObj_t* pSemObj , _SlNonOsSemObj_t Value);
#define _SlSyncObj_t    _SlNonOsSemObj_t
#define sl_SyncObjCreate(pSyncObj,pName)           _SlNonOsSemSet(pSyncObj,__NON_OS_SYNC_OBJ_CLEAR_VALUE)
#define sl_SyncObjDelete(pSyncObj)                  _SlNonOsSemSet(pSyncObj,0)
#define sl_SyncObjSignal(pSyncObj)                  _SlNonOsSemSet(pSyncObj,__NON_OS_SYNC_OBJ_SIGNAL_VALUE)
#define sl_SyncObjSignalFromIRQ(pSyncObj)           _SlNonOsSemSet(pSyncObj,__NON_OS_SYNC_OBJ_SIGNAL_VALUE)
#define sl_SyncObjWait(pSyncObj,Timeout)            _SlNonOsSemGet(pSyncObj,__NON_OS_SYNC_OBJ_SIGNAL_VALUE,__NON_OS_SYNC_OBJ_CLEAR_VALUE,Timeout)
#define _SlLockObj_t    _SlNonOsSemObj_t
#define sl_LockObjCreate(pLockObj,pName)            _SlNonOsSemSet(pLockObj,__NON_OS_LOCK_OBJ_UNLOCK_VALUE)
#define sl_LockObjDelete(pLockObj)                  _SlNonOsSemSet(pLockObj,0)
#define sl_LockObjLock(pLockObj,Timeout)            _SlNonOsSemGet(pLockObj,__NON_OS_LOCK_OBJ_UNLOCK_VALUE,__NON_OS_LOCK_OBJ_LOCK_VALUE,Timeout)
#define sl_LockObjUnlock(pLockObj)                  _SlNonOsSemSet(pLockObj,__NON_OS_LOCK_OBJ_UNLOCK_VALUE)
#define sl_Spawn(pEntry,pValue,flags)               _SlNonOsSpawn(pEntry,pValue,flags)
#define _SlTaskEntry                                _SlNonOsMainLoopTask

//#define SL_PLATFORM_EXTERNAL_SPAWN

#ifdef SL_PLATFORM_EXTERNAL_SPAWN
#define sl_Spawn(pEntry,pValue,flags)       os_Spawn(pEntry,pValue,flags)
#endif

//#define SL_MEMORY_MGMT_DYNAMIC

#ifdef SL_MEMORY_MGMT_DYNAMIC

#ifdef SL_PLATFORM_MULTI_THREADED

#define sl_Malloc(Size)                                 mem_Malloc(Size)
#define sl_Free(pMem)                                   mem_Free(pMem)

#else
#include <stdlib.h>
#define sl_Malloc(Size)                                 malloc(Size)
#define sl_Free(pMem)                                   free(pMem)
#endif

#endif


#define slcb_DeviceFatalErrorEvtHdlr                SimpleLinkFatalErrorEventHandler
#define slcb_DeviceGeneralEvtHdlr          SimpleLinkGeneralEventHandler
#define slcb_WlanEvtHdlr                     SimpleLinkWlanEventHandler
#define slcb_NetAppEvtHdlr                      SimpleLinkNetAppEventHandler
#define slcb_NetAppHttpServerHdlr   SimpleLinkHttpServerEventHandler
#define slcb_NetAppRequestHdlr  SimpleLinkNetAppRequestEventHandler
#define slcb_NetAppRequestMemFree  SimpleLinkNetAppRequestMemFreeEventHandler
#define slcb_SockEvtHdlr         SimpleLinkSockEventHandler

#ifndef SL_PLATFORM_MULTI_THREADED
#define slcb_SocketTriggerEventHandler SimpleLinkSocketTriggerEventHandler
#endif

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __USER_H__

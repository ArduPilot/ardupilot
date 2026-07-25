/*
**  File Name:  cfe_psp.h
**
**      Copyright (c) 2004-2006, United States government as represented by the
**      administrator of the National Aeronautics Space Administration.
**      All rights reserved. This software(cFE) was created at NASA's Goddard
**      Space Flight Center pursuant to government contracts.
**
**      This software may be used only pursuant to a United States government
**      sponsored project and the United States government may not be charged
**      for use thereof.
**
**  Author:  A. Cudmore
**
**  Purpose:  This file contains the cFE Platform Support Package(PSP) 
**            prototypes.
**            The PSP routines serve as the "glue" between the RTOS and
**            the cFE Flight Software. The routines fill gaps that are not
**            really considered part of the OS Abstraction, but are required
**            for the cFE flight software implementation. It is possible that
**            some of these routines could migrate into the OS AL.
**
**  $Log: cfe_psp.h  $
**  Revision 1.3 2009/07/29 12:04:46GMT-05:00 acudmore 
**  Added Bank parameter to EEPROM Power up/down and EEPROM write enable/disable functions.
**  Revision 1.2 2009/07/22 17:34:10EDT acudmore 
**  Added new watchdog API
**  Revision 1.1 2009/06/10 09:28:44EDT acudmore 
**  Initial revision
**  Member added to project c:/MKSDATA/MKS-REPOSITORY/CFE-PSP-REPOSITORY/fsw/inc/project.pj
**
*/

#ifndef _cfe_psp_
#define _cfe_psp_

/*
** Include Files
*/

#include "common_types.h"
#include "osapi.h"
#include "cfe_psp_config.h"
#include "psp_version.h"

/*
** Macro Definitions
*/

/*
** Error and return codes
*/
#define CFE_PSP_SUCCESS                     (0)
#define CFE_PSP_ERROR                       (-1)
#define CFE_PSP_INVALID_POINTER             (-2)
#define CFE_PSP_ERROR_ADDRESS_MISALIGNED    (-3)
#define CFE_PSP_ERROR_TIMEOUT               (-4)
#define CFE_PSP_INVALID_INT_NUM             (-5)
#define CFE_PSP_INVALID_MEM_ADDR            (-21)
#define CFE_PSP_INVALID_MEM_TYPE            (-22)
#define CFE_PSP_INVALID_MEM_RANGE           (-23)
#define CFE_PSP_INVALID_MEM_WORDSIZE        (-24)
#define CFE_PSP_INVALID_MEM_SIZE            (-25)
#define CFE_PSP_INVALID_MEM_ATTR            (-26)
#define CFE_PSP_ERROR_NOT_IMPLEMENTED       (-27)



/*
** Definitions for PSP PANIC types
*/
#define CFE_PSP_PANIC_STARTUP         1
#define CFE_PSP_PANIC_VOLATILE_DISK   2
#define CFE_PSP_PANIC_MEMORY_ALLOC    3
#define CFE_PSP_PANIC_NONVOL_DISK     4
#define CFE_PSP_PANIC_STARTUP_SEM     5
#define CFE_PSP_PANIC_CORE_APP        6
#define CFE_PSP_PANIC_GENERAL_FAILURE 7

/*
** Macros for the file loader
*/
#define BUFF_SIZE         256
#define SIZE_BYTE         1
#define SIZE_HALF         2
#define SIZE_WORD         3

/*
** Define memory types
*/
#define CFE_PSP_MEM_RAM        1
#define CFE_PSP_MEM_EEPROM     2
#define CFE_PSP_MEM_ANY        3
#define CFE_PSP_MEM_INVALID    4

/*
** Define Memory Read/Write Attributes
*/
#define CFE_PSP_MEM_ATTR_WRITE     0x01
#define CFE_PSP_MEM_ATTR_READ      0x02
#define CFE_PSP_MEM_ATTR_READWRITE 0x03

/*
** Define the Memory Word Sizes
*/
#define CFE_PSP_MEM_SIZE_BYTE     0x01
#define CFE_PSP_MEM_SIZE_WORD     0x02
#define CFE_PSP_MEM_SIZE_DWORD    0x04

/*
** Type Definitions
*/

/*
** Memory table type
*/
typedef struct
{
   uint32 MemoryType;
   uint32 WordSize;
   uint32 StartAddr;
   uint32 Size;
   uint32 Attributes;
} CFE_PSP_MemTable_t;

/*
** Function prototypes
*/

/*
** PSP entry point and reset routines
*/
extern void          CFE_PSP_Main(int ModeId, char *StartupFilePath);

/*
** CFE_PSP_Main is the entry point that the real time OS calls to start our
** software. This routine will do any BSP/OS specific setup, then call the
** entrypoint of the flight software ( i.e. the cFE main entry point ).
** The flight software (i.e. cFE ) should not call this routine.
*/

extern void         CFE_PSP_GetTime(OS_time_t *LocalTime);
/* This call gets the local time from the hardware on the Vxworks system
 * on the mcp750s
 * on the other os/hardware setup, it will get the time the normal way */


extern void          CFE_PSP_Restart(uint32 resetType);
/*
** CFE_PSP_Restart is the entry point back to the BSP to restart the processor.
** The flight software calls this routine to restart the processor.
*/


extern uint32        CFE_PSP_GetRestartType(uint32 *restartSubType );
/*
** CFE_PSP_GetRestartType returns the last reset type and if a pointer to a valid
** memory space is passed in, it returns the reset sub-type in that memory.
** Right now the reset types are application specific. For the cFE they
** are defined in the cfe_es.h file.
*/


extern void          CFE_PSP_FlushCaches(uint32 type, uint32 address, uint32 size);
/*
** This is a BSP specific cache flush routine
*/

extern uint32        CFE_PSP_GetProcessorId ( void );
/*
** CFE_PSP_GetProcessorId returns the CPU ID as defined by the specific board
** and BSP.
*/


extern uint32        CFE_PSP_GetSpacecraftId ( void );
/*
** CFE_PSP_GetSpacecraftId retuns the Spacecraft ID (if any )
*/


extern uint32 CFE_PSP_Get_Timer_Tick(void);
/*
** CFE_PSP_Get_Timer_Tick returns the underlying OS timer tick value
** It is used for the performance monitoring software
*/

extern uint32 CFE_PSP_GetTimerTicksPerSecond(void);
/*
** CFE_PSP_GetTimerTicksPerSecond provides the resolution of the least significant
** 32 bits of the 64 bit time stamp returned by CFE_PSP_Get_Timebase in timer
** ticks per second.  The timer resolution for accuracy should not be any slower
** than 1000000 ticks per second or 1 us per tick
*/

extern uint32 CFE_PSP_GetTimerLow32Rollover(void);
/*
** CFE_PSP_GetTimerLow32Rollover provides the number that the least significant
** 32 bits of the 64 bit time stamp returned by CFE_PSP_Get_Timebase rolls over.
** If the lower 32 bits rolls at 1 second, then the CFE_PSP_TIMER_LOW32_ROLLOVER
** will be 1000000.  if the lower 32 bits rolls at its maximum value (2^32) then
** CFE_PSP_TIMER_LOW32_ROLLOVER will be 0.
*/

extern void CFE_PSP_Get_Timebase(uint32 *Tbu, uint32 *Tbl);
/*
** CFE_PSP_Get_Timebase
*/

extern uint32 CFE_PSP_Get_Dec(void);
/*
** CFE_PSP_Get_Dec
*/


extern int32 CFE_PSP_InitProcessorReservedMemory(uint32 RestartType );
/*
** CFE_PSP_InitProcessorReservedMemory initializes all of the memory in the
** BSP that is preserved on a processor reset. The memory includes the
** Critical Data Store, the ES Reset Area, the Volatile Disk Memory, and
** the User Reserved Memory. In general, the memory areas will be initialized
** ( cleared ) on a Power On reset, and preserved during a processor reset.
*/

extern int32 CFE_PSP_GetCDSSize(uint32 *SizeOfCDS);
/*
** CFE_PSP_GetCDSSize fetches the size of the OS Critical Data Store area.
*/

extern int32 CFE_PSP_WriteToCDS(void *PtrToDataToWrite, uint32 CDSOffset, uint32 NumBytes);
/*
** CFE_PSP_WriteToCDS writes to the CDS Block.
*/

extern int32 CFE_PSP_ReadFromCDS(void *PtrToDataToRead, uint32 CDSOffset, uint32 NumBytes);
/*
** CFE_PSP_ReadFromCDS reads from the CDS Block
*/

extern int32 CFE_PSP_GetResetArea (void *PtrToResetArea, uint32 *SizeOfResetArea);
/*
** CFE_PSP_GetResetArea returns the location and size of the ES Reset information area.
** This area is preserved during a processor reset and is used to store the
** ER Log, System Log and reset related variables
*/

extern int32 CFE_PSP_GetUserReservedArea(void *PtrToUserArea, uint32 *SizeOfUserArea );
/*
** CFE_PSP_GetUserReservedArea returns the location and size of the memory used for the cFE
** User reserved area.
*/

extern int32 CFE_PSP_GetVolatileDiskMem(void *PtrToVolDisk, uint32 *SizeOfVolDisk );
/*
** CFE_PSP_GetVolatileDiskMem returns the location and size of the memory used for the cFE
** volatile disk.
*/

extern int32 CFE_PSP_GetKernelTextSegmentInfo(void *PtrToKernelSegment, uint32 *SizeOfKernelSegment);
/*
** CFE_PSP_GetKernelTextSegmentInfo returns the location and size of the kernel memory.
*/

extern int32 CFE_PSP_GetCFETextSegmentInfo(void *PtrToCFESegment, uint32 *SizeOfCFESegment);
/*
** CFE_PSP_GetCFETextSegmentInfo returns the location and size of the kernel memory.
*/

extern void CFE_PSP_WatchdogInit(void);
/*
** CFE_PSP_WatchdogInit configures the watchdog timer.
*/

extern void CFE_PSP_WatchdogEnable(void);
/*
** CFE_PSP_WatchdogEnable enables the watchdog timer.
*/

extern void CFE_PSP_WatchdogDisable(void);
/*
** CFE_PSP_WatchdogDisable disables the watchdog timer.
*/

extern void CFE_PSP_WatchdogService(void);
/*
** CFE_PSP_WatchdogService services the watchdog timer according to the 
** value set in WatchDogSet.
*/

extern uint32 CFE_PSP_WatchdogGet(void);
/*
** CFE_PSP_WatchdogGet gets the watchdog time in milliseconds 
*/

extern void CFE_PSP_WatchdogSet(uint32 WatchdogValue);
/*
** CFE_PSP_WatchdogSet sets the watchdog time in milliseconds
*/

extern void CFE_PSP_Panic(int32 ErrorCode);
/*
** CFE_PSP_Panic is called by the cFE Core startup code when it needs to abort the
** cFE startup. This should not be called by applications.
*/

extern int32 CFE_PSP_InitSSR(uint32 bus, uint32 device, char *DeviceName );
/*
** CFE_PSP_InitSSR will initialize the Solid state recorder memory for a particular platform
*/

extern int32 CFE_PSP_Decompress( char * srcFileName, char * dstFileName );
/*
** CFE_PSP_Decompress will uncompress the source file to the file specified in the
** destination file name. The Decompress uses the "gzip" algorithm. Files can
** be compressed using the "gzip" program available on almost all host platforms.
*/

extern void  CFE_PSP_AttachExceptions(void);
/*
** CFE_PSP_AttachExceptions will setup the exception environment for the chosen platform
** On a board, this can be configured to look at a debug flag or switch in order to
** keep the standard OS exeption handlers, rather than restarting the system
*/


extern void CFE_PSP_SetDefaultExceptionEnvironment(void);
/*
**
**   CFE_PSP_SetDefaultExceptionEnvironment defines the CPU and FPU exceptions that are enabled for each cFE Task/App
**
**   Notes: The exception environment is local to each task Therefore this must be
**          called for each task that that wants to do floating point and catch exceptions
*/


/*
** I/O Port API
*/
int32 CFE_PSP_PortRead8         (uint32 PortAddress, uint8 *ByteValue);
int32 CFE_PSP_PortWrite8        (uint32 PortAddress, uint8 ByteValue);
int32 CFE_PSP_PortRead16        (uint32 PortAddress, uint16 *uint16Value);
int32 CFE_PSP_PortWrite16       (uint32 PortAddress, uint16 uint16Value);
int32 CFE_PSP_PortRead32        (uint32 PortAddress, uint32 *uint32Value);
int32 CFE_PSP_PortWrite32       (uint32 PortAddress, uint32 uint32Value);

/*
** Memory API
*/
int32 CFE_PSP_MemRead8          (uint32 MemoryAddress, uint8 *ByteValue);
int32 CFE_PSP_MemWrite8         (uint32 MemoryAddress, uint8 ByteValue);
int32 CFE_PSP_MemRead16         (uint32 MemoryAddress, uint16 *uint16Value);
int32 CFE_PSP_MemWrite16        (uint32 MemoryAddress, uint16 uint16Value);
int32 CFE_PSP_MemRead32         (uint32 MemoryAddress, uint32 *uint32Value);
int32 CFE_PSP_MemWrite32        (uint32 MemoryAddress, uint32 uint32Value);

int32 CFE_PSP_MemCpy            (void *dest, void *src, uint32 n);
int32 CFE_PSP_MemSet            (void *dest, uint8 value, uint32 n);

int32  CFE_PSP_MemValidateRange (uint32 Address, uint32 Size, uint32 MemoryType);
uint32 CFE_PSP_MemRanges        (void);
int32  CFE_PSP_MemRangeSet      (uint32 RangeNum, uint32 MemoryType, uint32 StartAddr, 
                                 uint32 Size,     uint32 WordSize,   uint32 Attributes);
int32  CFE_PSP_MemRangeGet      (uint32 RangeNum, uint32 *MemoryType, uint32 *StartAddr, 
                                 uint32 *Size,    uint32 *WordSize,   uint32 *Attributes);

int32 CFE_PSP_EepromWrite8      (uint32 MemoryAddress, uint8 ByteValue);
int32 CFE_PSP_EepromWrite16     (uint32 MemoryAddress, uint16 uint16Value);
int32 CFE_PSP_EepromWrite32     (uint32 MemoryAddress, uint32 uint32Value);

int32 CFE_PSP_EepromWriteEnable (uint32 Bank);
int32 CFE_PSP_EepromWriteDisable(uint32 Bank);
int32 CFE_PSP_EepromPowerUp     (uint32 Bank);
int32 CFE_PSP_EepromPowerDown   (uint32 Bank);

#endif  /* _cfe_psp_ */

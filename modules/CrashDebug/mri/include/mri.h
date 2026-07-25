/* Copyright 2017 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Monitor for Remote Inspection. */
#ifndef _MRI_H_
#define _MRI_H_

#include <stdint.h>

/* Used to insert hardcoded breakpoint into user's code. */
#ifndef __debugbreak
    #define __debugbreak()  { __asm volatile ("bkpt #0"); }
#endif

/* Error strings that can be returned to GDB. */
#define     MRI_ERROR_INVALID_ARGUMENT      "E01"   /* Encountered error when parsing command arguments. */
#define     MRI_ERROR_MEMORY_ACCESS_FAILURE "E03"   /* Couldn't access requested memory. */
#define     MRI_ERROR_BUFFER_OVERRUN        "E04"   /* Overflowed internal input/output buffer. */
#define     MRI_ERROR_NO_FREE_BREAKPOINT    "E05"   /* No free FPB breakpoint comparator slots. */


#ifdef __cplusplus
extern "C"
{
#endif


/* pDebuggerParameters string passed into __mriInit contains a space separated list of configuration parameters to be
   used to initialize the debug monitor.  The supported options include:
   
   One of these options to indicate which UART to be used for the debugger connection:
        Valid options for LPC1768:
            MRI_UART_MBED_USB
            MRI_UART_MBED_P9_P10
            MRI_UART_MBED_P13_P14
            MRI_UART_MBED_P28_P27
            MRI_UART_0
            MRI_UART_1
            MRI_UART_2
            MRI_UART_3
        Valid options for STM32F429xx:
            MRI_UART_1
            MRI_UART_2
            MRI_UART_3
        Valid options for LPC43xx when specifying TX & RX pins separately:
            One from this list:
                MRI_UART_TX_P1_13
                MRI_UART_TX_P1_15
                MRI_UART_TX_P2_0
                MRI_UART_TX_P2_3
                MRI_UART_TX_P2_10
                MRI_UART_TX_P3_4
                MRI_UART_TX_P4_1
                MRI_UART_TX_P5_6
                MRI_UART_TX_P6_4
                MRI_UART_TX_P7_1
                MRI_UART_TX_P9_3
                MRI_UART_TX_P9_5
                MRI_UART_TX_PA_1
                MRI_UART_TX_PC_13
                MRI_UART_TX_PE_11
                MRI_UART_TX_PF_2
                MRI_UART_TX_PF_10
            And another from this list:
                MRI_UART_RX_P1_14
                MRI_UART_RX_P1_16
                MRI_UART_RX_P2_1
                MRI_UART_RX_P2_4
                MRI_UART_RX_P2_11
                MRI_UART_RX_P3_5
                MRI_UART_RX_P4_2
                MRI_UART_RX_P5_7
                MRI_UART_RX_P6_5
                MRI_UART_RX_P7_2
                MRI_UART_RX_P9_4
                MRI_UART_RX_P9_6
                MRI_UART_RX_PA_2
                MRI_UART_RX_PC_14
                MRI_UART_RX_PE_12
                MRI_UART_RX_PF_3
                MRI_UART_RX_PF_11
        Valid options for LPC43xx on Bambino210E:
            MRI_UART_MBED_USB
            MRI_UART_0
            MRI_UART_1
            MRI_UART_2
            MRI_UART_3

    By default the debug monitor expects to take full control of the UART to configure baud rate, etc.  However 
    including the following option will tell the monitor to assume that the user's firmware will configure and use the
    serial port until the first exception occurs:
        MRI_UART_SHARE
        
    When not sharing the UART, MRI will typically try to use the auto-baud functionality of the device so that the user
    can select the desired baud rate when they start GDB.  However it is possible to override this in the init string.
    For example the following option would set the baud rate to 230400 (note that spaces aren't allowed before or after
    the '=' character):
        MRI_UART_BAUD=230400
    NOTE: LPC176x version of MRI supports a maximum baud rate of 3Mbaud and the core clock can't run faster than
          128MHz or calculating baud rate divisors will fail.
*/
void __mriInit(const char* pDebuggerParameters);


/* Simple assembly language stubs that can be called from user's newlib stubs routines which will cause the operations
   to be redirected to the GDB host via MRI. */
int __mriNewLib_SemihostOpen(const char *pFilename, int flags, int mode);
int __mriNewLib_SemihostRename(const char *pOldFilename, const char *pNewFilename);
int __mriNewLib_SemihostUnlink(const char *pFilename);
int __mriNewLib_SemihostStat(const char *pFilename, void *pStat);
int __mriNewlib_SemihostWrite(int file, const char *ptr, int len);
int __mriNewlib_SemihostRead(int file, char *ptr, int len);
int __mriNewlib_SemihostLSeek(int file, int offset, int whence);
int __mriNewlib_SemihostClose(int file);
int __mriNewlib_SemihostFStat(int file, void *pStat);



/* Can be used by semihosting hooks to determine the index of the UART being used by MRI. */
int __mriPlatform_CommUartIndex(void);


#ifdef __cplusplus
}
#endif

#endif /* _MRI_H_ */

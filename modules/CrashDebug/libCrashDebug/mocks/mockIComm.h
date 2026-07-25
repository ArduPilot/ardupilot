/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef _MOCK_ICOMM_H_
#define _MOCK_ICOMM_H_

#include <IComm.h>


IComm*      mockIComm_Get(void);
void        mockIComm_SetShouldStopRunFlag(int shouldStopRun);
void        mockIComm_InitReceiveData(const char* pDataToReceive1, const char* pDataToReceive2 = NULL);
void        mockIComm_InitReceiveChecksummedData(const char* pDataToReceive1, const char* pDataToReceive2 = NULL);
void        mockIComm_DelayReceiveData(int delayCount);
void        mockIComm_InitTransmitDataBuffer(size_t Size);
const char* mockIComm_GetTransmittedData();
const char* mockIComm_ChecksumData(const char* pData);
void        mockIComm_SetIsGdbConnectedFlag(int isGdbConnected);
void        mockIComm_Uninit(void);


#endif /* _MOCK_ICOMM_H_ */

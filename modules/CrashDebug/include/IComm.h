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
#ifndef _ICOMM_H_
#define _ICOMM_H_


typedef struct IComm IComm;

typedef struct ICommVTable
{
    int  (*hasReceiveData)(IComm* pComm);
    int  (*receiveChar)(IComm* pComm);
    void (*sendChar)(IComm* pComm, int character);
    int  (*shouldStopRun)(IComm* pThis);
    int  (*isGdbConnected)(IComm* pComm);
} ICommVTable;

struct IComm
{
    ICommVTable* pVTable;
};


static inline int IComm_HasReceiveData(IComm* pThis)
{
    return pThis->pVTable->hasReceiveData(pThis);
}

static inline int IComm_ReceiveChar(IComm* pThis)
{
    return pThis->pVTable->receiveChar(pThis);
}

static inline void IComm_SendChar(IComm* pThis, int character)
{
    pThis->pVTable->sendChar(pThis, character);
}

static inline int IComm_ShouldStopRun(IComm* pThis)
{
    return pThis->pVTable->shouldStopRun(pThis);
}

static inline int IComm_IsGdbConnected(IComm* pThis)
{
    return pThis->pVTable->isGdbConnected(pThis);
}


#endif /* _ICOMM_H_ */

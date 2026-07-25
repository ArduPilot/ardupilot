/*  Copyright (C) 2017  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <assert.h>
#include <common.h>
#include <Console.h>
#include <StandardIComm.h>
#include <string.h>


/* Implementation of IComm interface. */
typedef struct StandardIComm StandardIComm;

static int  hasReceiveData(IComm* pComm);
static int  receiveChar(IComm* pComm);
static void sendChar(IComm* pComm, int character);
static int  shouldStopRun(IComm* pComm);
static int  isGdbConnected(IComm* pComm);

static ICommVTable g_icommVTable = {hasReceiveData, receiveChar, sendChar, shouldStopRun, isGdbConnected};

static struct StandardIComm
{
    ICommVTable* pVTable;
    int          hasGdbConnected;
} g_comm = {&g_icommVTable, FALSE};


__throws IComm* StandardIComm_Init()
{
    StandardIComm* pThis = &g_comm;
    pThis->hasGdbConnected = FALSE;
    return (IComm*)pThis;
}


void StandardIComm_Uninit(IComm* pComm)
{
}



/* IComm Interface Implementation. */
static int hasReceiveData(IComm* pComm)
{
    volatile int hasData = FALSE;

    __try
    {
        hasData = Console_HasStdInDataToRead();
    }
    __catch
    {
        clearExceptionCode();
        return FALSE;
    }
    return hasData;
}

static int receiveChar(IComm* pComm)
{
    StandardIComm* pThis = (StandardIComm*)pComm;
    int c = Console_ReadStdIn();
    pThis->hasGdbConnected = TRUE;
    return c;
}

static void sendChar(IComm* pComm, int character)
{
    Console_WriteStdOut(character);
}

static int shouldStopRun(IComm* pComm)
{
    return FALSE;
}

static int  isGdbConnected(IComm* pComm)
{
    StandardIComm* pThis = (StandardIComm*)pComm;
    return pThis->hasGdbConnected || hasReceiveData(pComm);
}


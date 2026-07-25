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
extern "C"
{
#include <common.h>
}
#include <assert.h>
#include <string.h>
#include "mockIComm.h"


#define EXTRACT_HI_NIBBLE(X) (((X) >> 4) & 0xF)
#define EXTRACT_LO_NIBBLE(X) ((X) & 0xF)

static const char NibbleToHexChar[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                                          '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };


class RecvBuffer
{
public:
    RecvBuffer()
    {
        m_pStart = m_pEnd = m_pCurr = NULL;
    }

    void init(const char* pString)
    {
        m_pStart = pString;
        m_pCurr = pString;
        m_pEnd = pString + strlen(pString);
    }

    char getNextChar()
    {
        char nextChar = 0;

        assert (m_pCurr < m_pEnd);
        if (m_pCurr < m_pEnd)
            nextChar = *m_pCurr++;
        return nextChar;
    }

    int bytesLeft()
    {
        return m_pEnd - m_pCurr;
    }

protected:
    const char* m_pStart;
    const char* m_pEnd;
    const char* m_pCurr;
};


int                g_shouldStopRun = TRUE;
static const char  g_emptyPacket[] = "$#00";
static RecvBuffer  g_receiveBuffers[2];
static size_t      g_receiveIndex;
static char*       g_pAlloc1;
static char*       g_pAlloc2;
static char*       g_pAllocChecksum;
static int         g_delayReceiveCount;
static char*       g_pTransmitDataBufferStart;
static char*       g_pTransmitDataBufferEnd;
static char*       g_pTransmitDataBufferCurr;
static int         g_isGdbConnected = TRUE;


static void waitForReceiveData(IComm* pComm);
static int isReceiveBufferEmpty();
static void freeReceiveAllocations();
static char* allocateAndCopyChecksummedData(const char* pData);
static size_t countPoundSigns(const char* p);
static void copyChecksummedData(char* pDest, const char* pSrc);
static void commUninitTransmitDataBuffer();


/* Implementation of IComm interface for this mock. */
static int  hasReceiveData(IComm* pComm);
static int  receiveChar(IComm* pComm);
static void sendChar(IComm* pComm, int character);
static int  shouldStopRun(IComm* pComm);
static int  isGdbConnected(IComm* pComm);

static ICommVTable g_icommVTable = {hasReceiveData, receiveChar, sendChar, shouldStopRun, isGdbConnected};

static struct TestIComm
{
    ICommVTable* pVTable;
} g_icomm = {&g_icommVTable};
typedef struct TestIComm TestIComm;

IComm* mockIComm_Get(void)
{
    return (IComm*)&g_icomm;
}

static int hasReceiveData(IComm* pComm)
{
    if (g_delayReceiveCount > 0)
    {
        g_delayReceiveCount--;
        return FALSE;
    }

    if (isReceiveBufferEmpty())
    {
        if (g_receiveIndex < ARRAY_SIZE(g_receiveBuffers))
            g_receiveIndex++;
        return FALSE;
    }

    return TRUE;
}

static int isReceiveBufferEmpty()
{
    if (g_receiveIndex >= ARRAY_SIZE(g_receiveBuffers))
        return TRUE;
    return g_receiveBuffers[g_receiveIndex].bytesLeft() == 0;
}

static int receiveChar(IComm* pComm)
{
    waitForReceiveData(pComm);
    return g_receiveBuffers[g_receiveIndex].getNextChar();
}

static void waitForReceiveData(IComm* pComm)
{
    while (!IComm_HasReceiveData(pComm))
    {
    }
}

static void sendChar(IComm* pComm, int character)
{
    if (g_pTransmitDataBufferCurr < g_pTransmitDataBufferEnd)
        *g_pTransmitDataBufferCurr++ = (char)character;
}

static int shouldStopRun(IComm* pComm)
{
    return g_shouldStopRun;
}

static int isGdbConnected(IComm* pComm)
{
    return g_isGdbConnected;
}



/* Routines used by tests to control how IComm mock interface should behave. */
void mockIComm_SetShouldStopRunFlag(int shouldStopRun)
{
    g_shouldStopRun = shouldStopRun;
}


void mockIComm_InitReceiveData(const char* pDataToReceive1, const char* pDataToReceive2 /*= NULL*/)
{
    g_receiveBuffers[0].init(pDataToReceive1);
    if (pDataToReceive2)
        g_receiveBuffers[1].init(pDataToReceive2);
    else
        g_receiveBuffers[1].init(g_emptyPacket);
    g_receiveIndex = 0;
    g_delayReceiveCount = 0;
}


void mockIComm_InitReceiveChecksummedData(const char* pDataToReceive1, const char* pDataToReceive2 /*= NULL*/)
{
    freeReceiveAllocations();
    g_pAlloc1 = allocateAndCopyChecksummedData(pDataToReceive1);
    g_receiveBuffers[0].init(g_pAlloc1);
    if (pDataToReceive2)
    {
        g_pAlloc2 = allocateAndCopyChecksummedData(pDataToReceive2);
        g_receiveBuffers[1].init(g_pAlloc2);
    }
    else
    {
        g_receiveBuffers[1].init(g_emptyPacket);
    }
    g_receiveIndex = 0;
    g_delayReceiveCount = 0;
}

static void freeReceiveAllocations()
{
    free(g_pAlloc1);
    free(g_pAlloc2);
    g_pAlloc1 = g_pAlloc2 = NULL;
}

static char* allocateAndCopyChecksummedData(const char* pData)
{
    size_t len = strlen(pData) + 2 * countPoundSigns(pData) + 1;
    char*  pAlloc = (char*) malloc(len);
    copyChecksummedData(pAlloc, pData);
    return pAlloc;
}

static size_t countPoundSigns(const char* p)
{
    size_t count = 0;
    while (*p)
    {
        if (*p++ == '#')
            count++;
    }
    return count;
}

static void copyChecksummedData(char* pDest, const char* pSrc)
{
    char checksum = 0;

    while (*pSrc)
    {
        char curr = *pSrc++;

        *pDest++ = curr;
        switch (curr)
        {
        case '$':
            checksum = 0;
            break;
        case '#':
            *pDest++ = NibbleToHexChar[EXTRACT_HI_NIBBLE(checksum)];
            *pDest++ = NibbleToHexChar[EXTRACT_LO_NIBBLE(checksum)];
            break;
        default:
            checksum += curr;
            break;
        }
    }
    *pDest++ = '\0';
}

void mockIComm_DelayReceiveData(int delayCount)
{
    g_delayReceiveCount = delayCount;
}

void mockIComm_InitTransmitDataBuffer(size_t size)
{
    commUninitTransmitDataBuffer();
    g_pTransmitDataBufferStart = (char*)malloc(size + 1);
    g_pTransmitDataBufferCurr = g_pTransmitDataBufferStart;
    g_pTransmitDataBufferEnd = g_pTransmitDataBufferStart + size;
}

const char* mockIComm_GetTransmittedData()
{
    *g_pTransmitDataBufferCurr = '\0';
    return g_pTransmitDataBufferStart;
}


const char* mockIComm_ChecksumData(const char* pData)
{
    free(g_pAllocChecksum);
    g_pAllocChecksum = allocateAndCopyChecksummedData(pData);
    return g_pAllocChecksum;
}


void mockIComm_SetIsGdbConnectedFlag(int isGdbConnected)
{
    g_isGdbConnected = isGdbConnected;
}


void mockIComm_Uninit(void)
{
    free(g_pAlloc1);
    free(g_pAlloc2);
    free(g_pAllocChecksum);
    g_pAlloc1 = NULL;
    g_pAlloc2 = NULL;
    g_pAllocChecksum = NULL;
    g_receiveBuffers[0].init("");
    g_receiveBuffers[1].init("");
    commUninitTransmitDataBuffer();
    g_isGdbConnected = TRUE;
}

static void commUninitTransmitDataBuffer()
{
    free(g_pTransmitDataBufferStart);
    g_pTransmitDataBufferStart = NULL;
    g_pTransmitDataBufferCurr = NULL;
    g_pTransmitDataBufferEnd = NULL;
}

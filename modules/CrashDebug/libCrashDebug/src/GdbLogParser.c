/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#include <common.h>
#include <CrashCatcher.h>
#include <ctype.h>
#include <GdbLogParser.h>
#include <FileFailureInject.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef struct ParseObject
{
    IMemory*         pMem;
    RegisterContext* pContext;
    uint32_t         regionStart;
    uint32_t         regionSize;
    uint32_t         nextExpectedAddress;
    char             lineText[1024];
} ParseObject;

typedef enum ParseType
{
    TYPE_OTHER,
    TYPE_MEMORY,
    TYPE_REGISTER
} ParseType;

typedef struct ParseResults
{
    union
    {
        struct
        {
            uint32_t  address;
            uint32_t  values[4];
            uint32_t  valueCount;
        };
        struct
        {
            size_t   registerOffset;
            uint32_t registerValue;
        };
    };
    ParseType type;
} ParseResults;


static FILE* openFileAndThrowOnError(const char* pLogFilename);
static void initPSPandMSP(RegisterContext* pContext);
static void firstPassHandler(ParseObject* pObject, const ParseResults* pParseResults);
static void firstPassMemoryHandler(ParseObject* pObject, const ParseResults* pParseResults);
static void firstPassRegisterHandler(ParseObject* pObject, const ParseResults* pParseResults);
static int isFloatingPointRegister(size_t registerOffset);
static void secondPassHandler(ParseObject* pObject, const ParseResults* pParseResults);
static void parseLines(IMemory* pMem,
                       RegisterContext* pContext,
                       FILE* pLogFile,
                       void (*passHandler)(ParseObject*, const ParseResults*));
static ParseResults parseLine(const char* pLine);
static int isMemoryLine(const char* pLine);
static int is8DigitHexValue(const char* pLine);
static int isHexDigit(char c);
static ParseResults parseMemoryLine(const char* pLine);
static const char* parseAddress(ParseResults* pResults, const char* pLine);
static int areMoreValuesToParseOnLine(const ParseResults* pResults, const char* pLine);
static const char* skipWhitespaceSymbolsAndParseNextValue(ParseResults* pResults, const char* pLine);
static const char* skipWhitespaceAndSymbol(const char* pLine);
static const char* skipWhitespace(const char* pLine);
static const char* skipSymbol(const char* pLine);
static const char* parseValue(ParseResults* pResults, const char* pLine);
static int isRegisterLine(const char* pLine, size_t* pRegisterOffset);
static ParseResults parseRegisterLine(const char* pLine, size_t registerOffset);
static uint32_t parseFloatRegisterLine(const char* pLine);
static const char* findWhitespace(const char* pLine);
static ParseResults parseOtherLine(const char* pLine);
static void rewindLogFileAndThrowOnError(FILE* pLogFile);


__throws void GdbLogParse(IMemory* pMem, RegisterContext* pContext, const char* pLogFilename)
{
    FILE* volatile pLogFile = NULL;

    __try
    {
        pLogFile = openFileAndThrowOnError(pLogFilename);
        initPSPandMSP(pContext);
        parseLines(pMem, pContext, pLogFile, firstPassHandler);
        rewindLogFileAndThrowOnError(pLogFile);
        parseLines(pMem, pContext, pLogFile, secondPassHandler);
        fclose(pLogFile);
    }
    __catch
    {
        if (pLogFile)
            fclose(pLogFile);
        __rethrow;
    }
}

static FILE* openFileAndThrowOnError(const char* pLogFilename)
{
    FILE* pLogFile = fopen(pLogFilename, "r");
    if (!pLogFile)
        __throw_msg(fileException, "Failed to open \"%s\" GDB log.", pLogFilename);
    return pLogFile;
}

static void initPSPandMSP(RegisterContext* pContext)
{
    pContext->R[MSP] = DEFAULT_SP_VALUE;
    pContext->R[PSP] = DEFAULT_SP_VALUE;
}

static void firstPassHandler(ParseObject* pObject, const ParseResults* pParseResults)
{
    switch (pParseResults->type)
    {
    case TYPE_MEMORY:
        firstPassMemoryHandler(pObject, pParseResults);
        break;
    case TYPE_REGISTER:
        firstPassRegisterHandler(pObject, pParseResults);
        break;
    case TYPE_OTHER:
    default:
        break;
    }
}

static void firstPassMemoryHandler(ParseObject* pObject, const ParseResults* pParseResults)
{
    if (pParseResults->address != pObject->nextExpectedAddress)
    {
        if (pObject->regionStart != 0xFFFFFFFF)
            MemorySim_CreateRegion(pObject->pMem, pObject->regionStart, pObject->regionSize);
        pObject->regionStart = pParseResults->address;
        pObject->regionSize = 0;
    }
    pObject->regionSize += pParseResults->valueCount * sizeof(uint32_t);
    pObject->nextExpectedAddress = pParseResults->address + pParseResults->valueCount * sizeof(uint32_t);
}

static void firstPassRegisterHandler(ParseObject* pObject, const ParseResults* pParseResults)
{
    if (isFloatingPointRegister(pParseResults->registerOffset))
        pObject->pContext->flags |= CRASH_CATCHER_FLAGS_FLOATING_POINT;
    *(uint32_t*)((uint8_t*)pObject->pContext + pParseResults->registerOffset) = pParseResults->registerValue;
}

static int isFloatingPointRegister(size_t registerOffset)
{
    return registerOffset >= offsetof(RegisterContext, FPR[S0]) &&
           registerOffset <= offsetof(RegisterContext, FPR[S31]);
}

static void secondPassHandler(ParseObject* pObject, const ParseResults* pParseResults)
{
    uint32_t i;
    if (pParseResults->type != TYPE_MEMORY)
        return;
    for (i = 0 ; i < pParseResults->valueCount ; i++)
        IMemory_Write32(pObject->pMem, pParseResults->address + i * sizeof(uint32_t), pParseResults->values[i]);
}

static void parseLines(IMemory* pMem,
                       RegisterContext* pContext,
                       FILE* pLogFile,
                       void (*passHandler)(ParseObject*, const ParseResults*))
{
    ParseObject object;
    object.pMem = pMem;
    object.pContext = pContext;
    object.regionStart = 0xFFFFFFFF;
    object.regionSize = 0;
    object.nextExpectedAddress = 0xFFFFFFFF;

    while (NULL != fgets(object.lineText, sizeof(object.lineText), pLogFile))
    {
        ParseResults parseResults = parseLine(object.lineText);
        passHandler(&object, &parseResults);
    }
    if (object.regionSize != 0)
        MemorySim_CreateRegion(object.pMem, object.regionStart, object.regionSize);
}

static ParseResults parseLine(const char* pLine)
{
    size_t registerOffset = 0;
    if (isMemoryLine(pLine))
        return parseMemoryLine(pLine);
    else if (isRegisterLine(pLine, &registerOffset))
        return parseRegisterLine(pLine, registerOffset);
    else
        return parseOtherLine(pLine);
}

static int isMemoryLine(const char* pLine)
{
    return (strlen(pLine) > 11 && is8DigitHexValue(pLine));
}

static int is8DigitHexValue(const char* pLine)
{
    return (pLine[0] == '0' &&
            pLine[1] == 'x' &&
            isHexDigit(pLine[2]) &&
            isHexDigit(pLine[3]) &&
            isHexDigit(pLine[4]) &&
            isHexDigit(pLine[5]) &&
            isHexDigit(pLine[6]) &&
            isHexDigit(pLine[7]) &&
            isHexDigit(pLine[8]) &&
            isHexDigit(pLine[9]));
}

static int isHexDigit(char c)
{
    return ((c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'f') ||
            (c >= 'A' && c <= 'F'));
}

static ParseResults parseMemoryLine(const char* pLine)
{
    ParseResults results;

    results.type = TYPE_MEMORY;
    results.valueCount = 0;
    pLine = parseAddress(&results, pLine);
    while (areMoreValuesToParseOnLine(&results, pLine))
        pLine = skipWhitespaceSymbolsAndParseNextValue(&results, pLine);
    return results;
}

static const char* parseAddress(ParseResults* pResults, const char* pLine)
{
    pResults->address = strtoul(pLine, NULL, 0);
    // Skip 0xXXXXXXXX: address formatted string.
    return pLine + (2 + 8 + 1);
}

static int areMoreValuesToParseOnLine(const ParseResults* pResults, const char* pLine)
{
    return (*pLine && pResults->valueCount < ARRAY_SIZE(pResults->values));
}

static const char* skipWhitespaceSymbolsAndParseNextValue(ParseResults* pResults, const char* pLine)
{
    pLine = skipWhitespaceAndSymbol(pLine);
    pLine = parseValue(pResults, pLine);
    return pLine;
}

static const char* skipWhitespaceAndSymbol(const char* pLine)
{
    pLine = skipWhitespace(pLine);
    pLine = skipSymbol(pLine);
    pLine = skipWhitespace(pLine);
    return pLine;
}

static const char* skipWhitespace(const char* pLine)
{
    while (*pLine && isspace(*pLine))
        pLine++;
    return pLine;
}

static const char* skipSymbol(const char* pLine)
{
    int nestingLevel = 0;

    if (*pLine != '<')
        return pLine;
    while (*pLine)
    {
        if (*pLine == '<')
            nestingLevel++;
        else if (*pLine == '>')
            nestingLevel--;
        else if (isspace(*pLine) && nestingLevel == 0)
            break;
        pLine++;
    }
    return pLine;

}

static const char* parseValue(ParseResults* pResults, const char* pLine)
{
    char* pNext = NULL;
    pResults->values[pResults->valueCount++] = strtoul(pLine, &pNext, 0);
    return pNext;
}

static int isRegisterLine(const char* pLine, size_t* pRegisterOffset)
{
    size_t i;
    struct
    {
        const char* pName;
        size_t      offset;
    } registers[] =
    {
        { "r0             ", offsetof(RegisterContext, R[R0]) },
        { "r1             ", offsetof(RegisterContext, R[R1]) },
        { "r2             ", offsetof(RegisterContext, R[R2]) },
        { "r3             ", offsetof(RegisterContext, R[R3]) },
        { "r4             ", offsetof(RegisterContext, R[R4]) },
        { "r5             ", offsetof(RegisterContext, R[R5]) },
        { "r6             ", offsetof(RegisterContext, R[R6]) },
        { "r7             ", offsetof(RegisterContext, R[R7]) },
        { "r8             ", offsetof(RegisterContext, R[R8]) },
        { "r9             ", offsetof(RegisterContext, R[R9]) },
        { "r10            ", offsetof(RegisterContext, R[R10]) },
        { "r11            ", offsetof(RegisterContext, R[R11]) },
        { "r12            ", offsetof(RegisterContext, R[R12]) },
        { "sp             ", offsetof(RegisterContext, R[SP]) },
        { "lr             ", offsetof(RegisterContext, R[LR]) },
        { "pc             ", offsetof(RegisterContext, R[PC]) },
        { "xpsr           ", offsetof(RegisterContext, R[XPSR]) },
        { "msp            ", offsetof(RegisterContext, R[MSP]) },
        { "psp            ", offsetof(RegisterContext, R[PSP]) },
        { "s0             ", offsetof(RegisterContext, FPR[S0]) },
        { "s1             ", offsetof(RegisterContext, FPR[S1]) },
        { "s2             ", offsetof(RegisterContext, FPR[S2]) },
        { "s3             ", offsetof(RegisterContext, FPR[S3]) },
        { "s4             ", offsetof(RegisterContext, FPR[S4]) },
        { "s5             ", offsetof(RegisterContext, FPR[S5]) },
        { "s6             ", offsetof(RegisterContext, FPR[S6]) },
        { "s7             ", offsetof(RegisterContext, FPR[S7]) },
        { "s8             ", offsetof(RegisterContext, FPR[S8]) },
        { "s9             ", offsetof(RegisterContext, FPR[S9]) },
        { "s10            ", offsetof(RegisterContext, FPR[S10]) },
        { "s11            ", offsetof(RegisterContext, FPR[S11]) },
        { "s12            ", offsetof(RegisterContext, FPR[S12]) },
        { "s13            ", offsetof(RegisterContext, FPR[S13]) },
        { "s14            ", offsetof(RegisterContext, FPR[S14]) },
        { "s15            ", offsetof(RegisterContext, FPR[S15]) },
        { "s16            ", offsetof(RegisterContext, FPR[S16]) },
        { "s17            ", offsetof(RegisterContext, FPR[S17]) },
        { "s18            ", offsetof(RegisterContext, FPR[S18]) },
        { "s19            ", offsetof(RegisterContext, FPR[S19]) },
        { "s20            ", offsetof(RegisterContext, FPR[S20]) },
        { "s21            ", offsetof(RegisterContext, FPR[S21]) },
        { "s22            ", offsetof(RegisterContext, FPR[S22]) },
        { "s23            ", offsetof(RegisterContext, FPR[S23]) },
        { "s24            ", offsetof(RegisterContext, FPR[S24]) },
        { "s25            ", offsetof(RegisterContext, FPR[S25]) },
        { "s26            ", offsetof(RegisterContext, FPR[S26]) },
        { "s27            ", offsetof(RegisterContext, FPR[S27]) },
        { "s28            ", offsetof(RegisterContext, FPR[S28]) },
        { "s29            ", offsetof(RegisterContext, FPR[S29]) },
        { "s30            ", offsetof(RegisterContext, FPR[S30]) },
        { "s31            ", offsetof(RegisterContext, FPR[S31]) },
        { "fpscr          ", offsetof(RegisterContext, FPR[FPSCR]) }
    };

    for (i = 0 ; i < ARRAY_SIZE(registers) ; i++)
    {
        if (0 == strncmp(registers[i].pName, pLine, 15))
        {
            *pRegisterOffset  = registers[i].offset;
            return TRUE;
        }
    }
    return FALSE;
}

static ParseResults parseRegisterLine(const char* pLine, size_t registerOffset)
{
    ParseResults results;
    results.type = TYPE_REGISTER;
    results.registerOffset = registerOffset;

    if (isFloatingPointRegister(registerOffset))
    {
        results.registerValue = parseFloatRegisterLine(pLine);
    }
    else
    {
        // Register value is found after 15 character long register name field.
        results.registerValue = strtoul(&pLine[15], NULL, 0);
    }
    return results;
}

static uint32_t parseFloatRegisterLine(const char* pLine)
{
    // Floating point  lines haves following format and we want to use the raw hexadecimal value.
    // "s1             1	(raw 0x3f800000)"

    // The two values start after the 15 character long register name field.
    // Skip over the first value to find the start of the raw value.
    pLine = findWhitespace(&pLine[15]);
    pLine = skipWhitespace(pLine);

    if (strncmp(pLine, "(raw ", 5) != 0)
        return -1;
    return strtoul(&pLine[5], NULL, 0);
}

static const char* findWhitespace(const char* pLine)
{
    while (*pLine && !isspace(*pLine))
        pLine++;
    return pLine;
}

static ParseResults parseOtherLine(const char* pLine)
{
    ParseResults results;
    results.type = TYPE_OTHER;
    return results;
}

static void rewindLogFileAndThrowOnError(FILE* pLogFile)
{
    int seekResult = fseek(pLogFile, SEEK_SET, 0);
    if (seekResult == -1)
        __throw_msg(fileException, "Failed to rewind GDB log for second pass.");
}

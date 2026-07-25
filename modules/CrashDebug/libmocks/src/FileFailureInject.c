/*  Copyright (C) 2012  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Module for injecting failures into file I/O calls. */
#include <stdio.h>
#include <string.h>


#define FAIL_ALL_CALLS -1


FILE*  (*hook_fopen)(const char* filename, const char* mode) = fopen;
int    (*hook_fclose)(FILE* stream) = fclose;
int    (*hook_fseek)(FILE* stream, long offset, int whence) = fseek;
long   (*hook_ftell)(FILE* stream) = ftell;
size_t (*hook_fwrite)(const void* ptr, size_t size, size_t nitems, FILE* stream) = fwrite;
size_t (*hook_fread)(void* ptr, size_t size, size_t nitems, FILE* stream) = fread;
char*   (*hook_fgets)(char * str, int size, FILE * stream) = fgets;


static FILE*        g_fopenFailureReturn;
static int          g_fseekFailureReturn;
static int          g_fseekCallsToPass;
static long         g_ftellFailureReturn;
static size_t       g_fwriteFailureReturn;
static size_t       g_freadFailureReturn;
static int          g_freadToFail;
static const char** g_ppFgetsStart;
static const char** g_ppFgetsEnd;
static const char** g_ppFgetsCurr;

void freadRestore(void);


static FILE* mock_fopen(const char* filename, const char* mode);
void fopenSetReturn(FILE* pFailureReturn)
{
    hook_fopen = mock_fopen;
    g_fopenFailureReturn = pFailureReturn;
}

static FILE* mock_fopen(const char* filename, const char* mode)
{
    return g_fopenFailureReturn;
}


void fopenRestore(void)
{
    hook_fopen = fopen;
}


static int mock_fclose(FILE* pFile);
void fcloseIgnore(void)
{
    hook_fclose = mock_fclose;
}

static int mock_fclose(FILE* pFile)
{
    return 0;
}



void fcloseRestore(void)
{
    hook_fclose = fclose;
}


static int mock_fseek(FILE* stream, long offset, int whence);
void fseekSetReturn(int failureReturn)
{
    g_fseekFailureReturn = failureReturn;
    hook_fseek = mock_fseek;
}

static int mock_fseek(FILE* stream, long offset, int whence)
{
    if (g_fseekCallsToPass > 0)
    {
        g_fseekCallsToPass--;
        return fseek(stream, offset, whence);
    }

    return g_fseekFailureReturn;
}


void fseekSetCallsBeforeFailure(int callCountToAllowBeforeFailing)
{
    g_fseekCallsToPass = callCountToAllowBeforeFailing;
}


void fseekRestore(void)
{
    hook_fseek = fseek;
}


static long mock_ftell(FILE* stream);
void ftellFail(long failureReturn)
{
    g_ftellFailureReturn = failureReturn;
    hook_ftell = mock_ftell;
}

static long mock_ftell(FILE* stream)
{
    return g_ftellFailureReturn;
}


void ftellRestore(void)
{
    hook_ftell = ftell;
}


size_t mock_fwrite(const void* ptr, size_t size, size_t nitems, FILE* stream);
void fwriteFail(size_t failureReturn)
{
    g_fwriteFailureReturn = failureReturn;
    hook_fwrite = mock_fwrite;
}

size_t mock_fwrite(const void* ptr, size_t size, size_t nitems, FILE* stream)
{
    return g_fwriteFailureReturn;
}


void fwriteRestore(void)
{
    hook_fwrite = fwrite;
}


static size_t mock_fread(void* ptr, size_t size, size_t nitems, FILE* stream);
void freadFail(size_t failureReturn)
{
    g_freadFailureReturn = failureReturn;
    g_freadToFail = FAIL_ALL_CALLS;
    hook_fread = mock_fread;
}

static size_t mock_fread(void* ptr, size_t size, size_t nitems, FILE* stream)
{
    if (g_freadToFail > 0)
    {
        if (--g_freadToFail == 0)
        {
            freadRestore();
            return g_freadFailureReturn;
        }
        return fread(ptr, size, nitems, stream);
    }
    return g_freadFailureReturn;
}


void freadToFail(int readToFail)
{
    g_freadToFail = readToFail;
}


void freadRestore(void)
{
    hook_fread = fread;
    g_freadToFail = 0;
}


static char*   mock_fgets(char * str, int size, FILE * stream);
void fgetsSetData(const char** ppLines, size_t lineCount)
{
    hook_fgets = mock_fgets;
    g_ppFgetsStart = ppLines;
    g_ppFgetsCurr = ppLines;
    g_ppFgetsEnd = ppLines + lineCount;
}

static char* mock_fgets(char * str, int size, FILE * stream)
{
    if (g_ppFgetsCurr >= g_ppFgetsEnd)
    {
        // Rewind to start so that file can be processed again.
        g_ppFgetsCurr = g_ppFgetsStart;
        // Return EOF condition once.
        return NULL;
    }

    strncpy(str, *g_ppFgetsCurr++, size);
    str[size-1] = '\0';
    return str;
}

void fgetsRestore(void)
{
    hook_fgets = fgets;
    g_ppFgetsStart = NULL;
    g_ppFgetsCurr = NULL;
    g_ppFgetsEnd = NULL;
}

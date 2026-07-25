/*  Copyright (C) 2013  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* Module for spying on printf output from code under test. */
#ifndef _FILE_FAILURE_INJECT_H_
#define _FILE_FAILURE_INJECT_H_

#include <stdio.h>

/* Pointer to file I/O routines which can intercepted by this module. */
extern FILE*  (*hook_fopen)(const char* filename, const char* mode);
extern int    (*hook_fclose)(FILE* stream);
extern int    (*hook_fseek)(FILE* stream, long offset, int whence);
extern long   (*hook_ftell)(FILE* stream);
extern size_t (*hook_fwrite)(const void* ptr, size_t size, size_t nitems, FILE* stream);
extern size_t (*hook_fread)(void* ptr, size_t size, size_t nitems, FILE* stream);
extern char*  (*hook_fgets)(char * str, int size, FILE * stream);

void fopenSetReturn(FILE* pFailureReturn);
void fopenRestore(void);

void fcloseIgnore(void);
void fcloseRestore(void);

void fseekSetReturn(int failureReturn);
void fseekSetCallsBeforeFailure(int callCountToAllowBeforeFailing);
void fseekRestore(void);

void ftellFail(long failureReturn);
void ftellRestore(void);

void fwriteFail(size_t failureReturn);
void fwriteRestore(void);

void freadFail(size_t failureReturn);
void freadToFail(int readToFail);
void freadRestore(void);

void fgetsSetData(const char** ppLines, size_t lineCount);
void fgetsRestore(void);


/* Force file I/O routines to go through hooking routines in unit tests. */
#define fopen  hook_fopen
#define fclose hook_fclose
#define fseek  hook_fseek
#define ftell  hook_ftell
#define fwrite hook_fwrite
#define fread  hook_fread
#define fgets  hook_fgets


#endif /* _FILE_FAILURE_INJECT_H_ */

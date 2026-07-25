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
#include <common.h>
#include <FileFailureInject.h>


long GetFileSize(FILE* pFile)
{
    int    result = -1;
    long   fileSize = 0;

    result = fseek(pFile, 0, SEEK_END);
    if (result == -1)
        __throw(fileException);

    fileSize = ftell(pFile);
    if (fileSize < 0)
        __throw(fileException);

    result = fseek(pFile, 0, SEEK_SET);
    if (result == -1)
        __throw(fileException);

    return fileSize;
}

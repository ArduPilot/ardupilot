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
/* Very rough exception handling like macros for C. */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdio.h>
#include <try_catch.h>


#define FALSE 0
#define TRUE  1

#define ARRAY_SIZE(X) (sizeof(X)/sizeof(X[0]))

__throws long GetFileSize(FILE* pFile);


#endif /* _COMMON_H_ */

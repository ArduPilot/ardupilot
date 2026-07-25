/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
#ifndef __VERSION_H_
#define __VERSION_H_


/* Version Information. */
#define VERSION_MAJOR       1
#define VERSION_MINOR       7
#define VERSION_BUILD       20210805
#define VERSION_SUBBUILD    0

#define VER_STR(X) VER_STR2(X)
#define VER_STR2(X) #X

#define VERSION_STRING VER_STR(VERSION_MAJOR) "." VER_STR(VERSION_MINOR) "-" VER_STR(VERSION_BUILD) "." VER_STR(VERSION_SUBBUILD)


/* Copyright notice. */
#define COPYRIGHT_NOTICE    "Copyright (C) 2021 Adam Green\n" \
                            "This is free software; see the source for copying conditions.  There is NO\n" \
                            "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n"

#endif

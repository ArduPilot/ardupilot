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
#ifndef _STANDARD_ICOMM_H_
#define _STANDARD_ICOMM_H_

#include <IComm.h>
#include <try_catch.h>


__throws IComm* StandardIComm_Init();
         void   StandardIComm_Uninit(IComm* pComm);


#endif /* _STANDARD_ICOMM_H_ */

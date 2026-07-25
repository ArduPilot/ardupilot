/* Copyright 2014 Adam Green (http://mbed.org/users/AdamGreen/)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Definition of _sys_*() functions and associated constants implemented in mbed/capi.ar */

#ifndef _MBEDSYS_H_
#define _MBEDSYS_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Types used by functions implemented in mbed.ar */
typedef int FILEHANDLE;

/* File openmode values for mbed _sys_open() */
#define OPENMODE_R      0 
#define OPENMODE_B      1 
#define OPENMODE_PLUS   2 
#define OPENMODE_W      4 
#define OPENMODE_A      8 

/* Functions implemented in mbed.ar */
FILEHANDLE  _sys_open(const char* name, int openmode);
int         _sys_close(FILEHANDLE fh);
int         _sys_write(FILEHANDLE fh, const unsigned char* buf, unsigned len, int mode);
int         _sys_read(FILEHANDLE fh, unsigned char* buf, unsigned len, int mode);
int         _sys_seek(FILEHANDLE fh, long pos); 
long        _sys_flen(FILEHANDLE fh); 
int         _sys_istty(FILEHANDLE fh); 


#ifdef __cplusplus
}
#endif

#endif /* _MBEDSYS_H_ */
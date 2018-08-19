/******************************************************************************
 * The MIT License

(c) 2017 night_ghost@ykoctpa.ru
 
based on:

 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

#pragma GCC optimize ("O2")

/**
 * @file syscalls.c
 * @brief Low level system routines used by Newlib for basic I/O and
 * memory allocation.
 */

#include <syscalls.h>
#include <systick.h>

/* _end and _eccm are set in the linker command file */
extern caddr_t _end;
extern caddr_t _eccm;

#pragma GCC diagnostic ignored "-Wunused-variable"

int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}

void _exit(int status)
{
    __error(13, status, 0, 0);
}

int _getpid(void)
{
	return 1;
}


/*
 * sbrk -- changes heap size. Get nbytes more
 *         RAM. We just increment a pointer in what's
 *         left of memory on the board.
 */
 
void *__brkval=0;
void *__brkval_ccm=0;

caddr_t stack_bottom=0;
bool sbrk_need_dma=false;

static caddr_t _sbrk_ram(int nbytes) {
    static caddr_t heap_ptr = NULL;
    caddr_t        base;

    if (heap_ptr == NULL) {
        heap_ptr = (caddr_t)&_end;
    }

    uint32_t top = (uint32_t)get_stack_top();

    if ( top - 256 > (unsigned int)heap_ptr+nbytes // there is place in stack
        || top < SRAM1_BASE /* 0x20000000*/  ) //      or stack not in RAM 
    {
        base = heap_ptr;
        heap_ptr += nbytes;
        __brkval = heap_ptr;
        return (base);
    } else {
        return ((caddr_t)-1);
    }
}

static caddr_t _sbrk_ccm(int nbytes) {
    static caddr_t heap_ptr = NULL;
    caddr_t        base;

    if (heap_ptr == NULL) {
        heap_ptr = (caddr_t)&_eccm;
    }

    uint32_t top = (uint32_t)get_stack_top() - 256; // reserve some memory

    if(stack_bottom) top = (uint32_t)stack_bottom;

    if ( top > (unsigned int)heap_ptr+nbytes) {// there is place in stack, if stack in RAM it will be true too
        base = heap_ptr;
        heap_ptr += nbytes;
        __brkval_ccm = heap_ptr;
        return (base);
    } else {
        return ((caddr_t)-1);
    }
}

caddr_t sbrk_ccm(int nbytes) {
    nbytes = (nbytes & ~3)+4; // alignment
    return _sbrk_ccm(nbytes);
}


caddr_t _sbrk(int nbytes) {
    return _sbrk_ram(nbytes);    
}


int _open(const char *path, int flags, ...) {
    return 1;
}

int _close(int fd) {
    return 0;
}

int _fstat(int fd, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) {
    return 1;
}

int isatty(int fd) {
    return 1;
}

int _lseek(int fd, off_t pos, int whence) {
    return -1;
}


void cgets(char *s, int bufsize) {
    char *p;
    int c;
    int i;

    for (i = 0; i < bufsize; i++) {
        *(s+i) = 0;
    }

    p = s;

    for (p = s; p < s + bufsize-1;) {
        c = getch();
        switch (c) {
        case '\r' :
        case '\n' :
            *p = '\n';
            return;

        case '\b' :
            if (p > s) {
                *p-- = 0;
            }
            break;

        default :
            *p++ = c;
            break;
        }
    }
    return;
}

int _write(int fd, const char *buf, size_t cnt) {
    size_t i;

    for (i = 0; i < cnt; i++)
        putch(buf[i]);

    return cnt;
}


void clock_gettime(uint32_t a1, void *a2) { return; } 

int val_read(void *dest, volatile const void *src, int bytes)
{

        int i;

        for (i = 0; i < bytes / 4; i++) {
                *(((volatile unsigned *)dest) + i) = *(((volatile unsigned *)src) + i);
        }

        return i * 4;
}


#define UDID_START              0x1FFF7A10
void get_board_serial(uint8_t *serialid)
{
        const volatile uint32_t *udid_ptr = (const uint32_t *)UDID_START;
        union udid id;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align" // GCC lies in this warning - union udid already contains uint32_t

        val_read((uint32_t *)&id, udid_ptr, sizeof(id));

#pragma GCC diagnostic pop


        /* Copy the serial from the chips non-write memory and swap endianess */
        serialid[0] = id.data[3];   serialid[1] = id.data[2];  serialid[2] = id.data[1];  serialid[3] = id.data[0];
        serialid[4] = id.data[7];   serialid[5] = id.data[6];  serialid[6] = id.data[5];  serialid[7] = id.data[4];
        serialid[8] = id.data[11];   serialid[9] = id.data[10];  serialid[10] = id.data[9];  serialid[11] = id.data[8];

}

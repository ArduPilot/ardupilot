
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <apps/nsh.h>
#include <fcntl.h>
#include "UARTDriver.h"

extern const AP_HAL::HAL& hal;

/*
  implement vsnprintf with support for %S meaning a progmem string
 */
static int libc_vsnprintf(char* str, size_t size, const char *fmt, va_list ap) 
{
    int i, ret;
    char *fmt2 = (char *)fmt;
    if (strstr(fmt2, "%S") != NULL) {
        fmt2 = strdup(fmt);
        for (i=0; fmt2[i]; i++) {
            // cope with %S
            if (fmt2[i] == '%' && fmt2[i+1] == 'S') {
                fmt2[i+1] = 's';
            }
        }
    }
    ret = vsnprintf(str, size, fmt2, ap);
    if (fmt2 != fmt) {
        free(fmt2);
    }
    return ret;
}

#include "Util.h"
using namespace PX4;

int PX4Util::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int PX4Util::snprintf_P(char* str, size_t size, const prog_char_t *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = libc_vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}


int PX4Util::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

int PX4Util::vsnprintf_P(char* str, size_t size, const prog_char_t *format,
            va_list ap)
{
    return libc_vsnprintf(str, size, format, ap);
}

extern bool _px4_thread_should_exit;

/*
  start an instance of nsh
 */
bool PX4Util::run_debug_shell(AP_HAL::BetterStream *stream)
{
	PX4UARTDriver *uart = (PX4UARTDriver *)stream;
	int fd;

	// trigger exit in the other threads. This stops use of the
	// various driver handles, and especially the px4io handle,
	// which otherwise would cause a crash if px4io is stopped in
	// the shell
	_px4_thread_should_exit = true;

	// take control of stream fd
	fd = uart->_get_fd();

	// mark it blocking (nsh expects a blocking fd)
        unsigned v;
        v = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, v & ~O_NONBLOCK);	

	// setup the UART on stdin/stdout/stderr
	close(0);
	close(1);
	close(2);
	dup2(fd, 0);
	dup2(fd, 1);
	dup2(fd, 2);

	nsh_consolemain(0, NULL);

	// this shouldn't happen
	hal.console->printf("shell exited\n");
	return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include "replace.h"

extern "C" {

// this is not declared in qurt headers
void HAP_debug(const char *msg, int level, const char *filename, int line);
    
void HAP_printf(const char *file, int line, const char *format, ...)
{
	va_list ap;
        char buf[300];
        
	va_start(ap, format);
	vsnprintf(buf, sizeof(buf), format, ap);
	va_end(ap);
        HAP_debug(buf, 0, file, line);
        //usleep(20000);
}
    
/**
   QURT doesn't have strnlen()
**/
size_t strnlen(const char *s, size_t max)
{
        size_t len;
  
        for (len = 0; len < max; len++) {
                if (s[len] == '\0') {
                        break;
                }
        }
        return len;  
}

int vasprintf(char **ptr, const char *format, va_list ap)
{
	int ret;
	va_list ap2;

	va_copy(ap2, ap);
	ret = vsnprintf(nullptr, 0, format, ap2);
	va_end(ap2);
	if (ret < 0) return ret;

	(*ptr) = (char *)malloc(ret+1);
	if (!*ptr) return -1;

	va_copy(ap2, ap);
	ret = vsnprintf(*ptr, ret+1, format, ap2);
	va_end(ap2);

	return ret;
}

int asprintf(char **ptr, const char *format, ...)
{
	va_list ap;
	int ret;
	
	*ptr = nullptr;
	va_start(ap, format);
	ret = vasprintf(ptr, format, ap);
	va_end(ap);

	return ret;
}

}

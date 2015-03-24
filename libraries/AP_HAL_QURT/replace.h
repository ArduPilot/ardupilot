#pragma once
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QURT
#include <stdlib.h>
#include <stdint.h>
#include <types.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>

#include <types.h>
#include <dirent.h>
extern "C" {

    /*
      work around broken headers
     */
    size_t strnlen(const char *s, size_t maxlen);
    int asprintf(char **, const char *, ...);
    off_t lseek(int, off_t, int);
    DIR *opendir (const char *);
    int unlink(const char *pathname);

    //typedef int32_t pid_t;
    pid_t getpid (void);
    
    void HAP_printf(const char *file, int line, const char *fmt, ...);
}

#define HAP_PRINTF(...) HAP_printf(__FILE__, __LINE__, __VA_ARGS__)

extern volatile int _last_dsp_line;
extern volatile const char *_last_dsp_file;
extern volatile uint32_t _last_counter;

#define HAP_LINE() do { _last_dsp_line = __LINE__; _last_dsp_file = __FILE__; _last_counter++; } while (0)

#endif // CONFIG_HAL_BOARD

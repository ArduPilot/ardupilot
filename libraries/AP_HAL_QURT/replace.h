#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <types.h>
#include <sys/types.h>
#include <unistd.h>
#include <dirent.h>

#include <types.h>
#include <dirent.h>
#include "ap_host/src/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
  work around broken headers
 */
size_t strnlen(const char *s, size_t maxlen);
char *strndup(const char *s, size_t n);
int asprintf(char **, const char *, ...);
off_t lseek(int, off_t, int);
DIR *opendir (const char *);
int unlink(const char *pathname);
void *memmem(const void *haystack, size_t haystacklen,
             const void *needle, size_t needlelen);

//typedef int32_t pid_t;
pid_t getpid (void);

void HAP_printf(const char *file, int line, const char *fmt, ...);

int __wrap_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#define HAP_PRINTF(...) HAP_printf(__FILE__, __LINE__, __VA_ARGS__)

extern volatile int _last_dsp_line;
extern volatile const char *_last_dsp_file;
extern volatile uint32_t _last_counter;

#define HAP_LINE() do { _last_dsp_line = __LINE__; _last_dsp_file = __FILE__; _last_counter++; } while (0)

// missing defines from math.h
#define M_SQRT1_2 0.70710678118654752440

#ifdef __cplusplus
// send a message to the host
bool qurt_rpc_send(struct qurt_rpc_msg &msg);
#endif


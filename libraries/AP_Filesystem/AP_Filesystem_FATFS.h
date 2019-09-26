/*
  FATFS backend for AP_Filesystem
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stddef.h>
#include <ff.h>

// Seek offset macros
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

#if FF_USE_LFN != 0
#define MAX_NAME_LEN FF_MAX_LFN 
#else
#define MAX_NAME_LEN 13
#endif

#define DT_REG 0
#define DT_DIR 1

struct dirent {
   char           d_name[MAX_NAME_LEN]; /* filename */
   uint8_t d_type;
};

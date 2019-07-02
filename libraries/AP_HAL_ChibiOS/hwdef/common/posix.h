/**
 @file fatfs/posix.h

 @brief POSIX wrapper for FatFS

 @par Copyright &copy; 2015 Mike Gore, GPL License
 @par You are free to use this code under the terms of GPL
   please retain a copy of this notice in any code you use it in.

This is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option)
any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _POSIX_H_
#define _POSIX_H_
#include <board.h>
#ifdef USE_POSIX
#define POSIX
#pragma GCC diagnostic ignored "-Wshadow"
#include <stdint.h>
#include <stddef.h>
#include <ff.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>

#define MAXLN 128
#define ISSPACE " \t\n\r\f\v"

#define BUFSIZ 256

///@brief make sure we use our strerror_r function

#ifdef __cplusplus
extern "C" {
#endif

///@brief POSIX stat structure
///@see stat()
///@see fstat()
struct stat
{
    dev_t     st_dev;    /*<  ID of device containing file */
    ino_t     st_ino;    /*<  inode number */
    mode_t    st_mode;   /*<  protection */
    nlink_t   st_nlink;  /*<  number of hard links */
    uid_t     st_uid;    /*<  user ID of owner */
    gid_t     st_gid;    /*<  group ID of owner */
    dev_t     st_rdev;   /*<  device ID (if special file) */
    off_t     st_size;   /*<  total size, in bytes */
    uint32_t st_blksize;/*<  blocksize for filesystem I/O */
    uint32_t  st_blocks; /*<  number of 512B blocks allocated */
    time_t    st_atime;  /*<  time of last access */
    time_t    st_mtime;  /*<  time of last modification */
    time_t    st_ctime;  /*<  time of last status change */
};

///@brief POSIX utimbuf structure
///@see utime()
typedef struct utimbuf
{
   time_t actime;       /* access time */
   time_t modtime;      /* modification time */
} utime_t;

#if FF_USE_LFN != 0
#define MAX_NAME_LEN FF_MAX_LFN 
#else
#define MAX_NAME_LEN 13
#endif

struct dirent {
#if 0 // unsupported
   ino_t          d_ino;       /* inode number */
   off_t          d_off;       /* not an offset; see NOTES */
   unsigned short d_reclen;    /* length of this record */
   unsigned char  d_type;      /* type of file; not supported
								  by all filesystem types */
#endif
   char           d_name[MAX_NAME_LEN]; /* filename */
};

typedef struct dirent dirent_t;


///@brief POSIX lstat()
///@see stat()
#define lstat stat
// =============================================
///@brief FILE type structure
struct __file {
    char    *buf;       /* buffer pointer */
    unsigned char unget;    /* ungetc() buffer */
    uint8_t flags;      /* flags, see below */
#define __SRD   0x0001      /* OK to read */
#define __SWR   0x0002      /* OK to write */
#define __SSTR  0x0004      /* this is an sprintf/snprintf string */
#define __SPGM  0x0008      /* fmt string is in progmem */
#define __SERR  0x0010      /* found error */
#define __SEOF  0x0020      /* found EOF */
#define __SUNGET 0x040      /* ungetc() happened */
#define __SMALLOC 0x80      /* handle is malloc()ed */
#if 0
	/* possible future extensions, will require uint16_t flags */
	#define __SRW   0x0100      /* open for reading & writing */
	#define __SLBF  0x0200      /* line buffered */
	#define __SNBF  0x0400      /* unbuffered */
	#define __SMBF  0x0800      /* buf is from malloc */
#endif
    int size;       /* size of buffer */
    int len;        /* characters read or written so far */
    int (*put)(char, struct __file *);  				/* write one char to device */
    int (*get)(struct __file *);    					/* read one char from device */
// FIXME add all low level functions here like _open, _close, ... like newlib does
    void    *udata;     /* User defined and accessible data. */
};
// =============================================
///@brief POSIX open modes  - no other combination are allowed.
/// - man page open(2)
/// - Note: The POSIX correct test of O_RDONLY is: (mode & O_ACCMODE) == O_RDONLY.
#define O_ACCMODE  00000003 /*< read, write, read-write modes */
#define O_RDONLY   00000000 /*< Read only */
#define O_WRONLY   00000001 /*< Write only */
#define O_RDWR     00000002 /*< Read/Write */
#define O_CREAT    00000100 /*< Create file only if it does not exist */
#define O_EXCL     00000200 /*< O_CREAT option, Create fails if file exists 
*/
#define O_NOCTTY   00000400 /*< @todo */
#define O_TRUNC    00001000 /*< Truncate if exists */
#define O_APPEND   00002000 /*< All writes are to EOF */
#define O_NONBLOCK 00004000 /*< @todo */
#define O_BINARY   00000004 /*< Binary */
#define O_TEXT     00000004 /*< Text End Of Line translation */
#define O_CLOEXEC  00000000
///@brief POSIX File types, see fstat and stat.
#define S_IFMT     0170000  /*< These bits determine file type.  */
#define S_IFDIR    0040000  /*< Directory.  */
#define S_IFCHR    0020000  /*< Character device.  */
#define S_IFBLK    0060000  /*< Block device.  */
#define S_IFREG    0100000  /*< Regular file.  */
#define S_IFIFO    0010000  /*< FIFO.  */
#define S_IFLNK    0120000  /*< Symbolic link.  */
#define S_IFSOCK   0140000  /*< Socket.  */
#define S_IREAD    0400     /*< Read by owner.  */
#define S_IWRITE   0200     /*< Write by owner.  */
#define S_IEXEC    0100     /*< Execute by owner.  */

///@brief POSIX File type test macros.
#define S_ISTYPE(mode, mask)  (((mode) & S_IFMT) == (mask))
#define S_ISDIR(mode)    S_ISTYPE((mode), S_IFDIR)
#define S_ISCHR(mode)    S_ISTYPE((mode), S_IFCHR)
#define S_ISBLK(mode)    S_ISTYPE((mode), S_IFBLK)
#define S_ISREG(mode)    S_ISTYPE((mode), S_IFREG)

//@brief POSIX File permissions, see fstat and stat  
#define S_IRUSR S_IREAD                     /*< Read by owner.  */
#define S_IWUSR S_IWRITE                    /*< Write by owner.  */
#define S_IXUSR S_IEXEC                     /*< Execute by owner.  */
#define S_IRWXU (S_IREAD|S_IWRITE|S_IEXEC)	/*< Read,Write,Execute by owner */

#define S_IRGRP (S_IRUSR >> 3)              /*< Read by group.  */
#define S_IWGRP (S_IWUSR >> 3)              /*< Write by group.  */
#define S_IXGRP (S_IXUSR >> 3)              /*< Execute by group.  */
#define S_IRWXG (S_IRWXU >> 3)				/*< Read,Write,Execute by user */

#define S_IROTH (S_IRGRP >> 3)              /*< Read by others.  */
#define S_IWOTH (S_IWGRP >> 3)              /*< Write by others.  */
#define S_IXOTH (S_IXGRP >> 3)              /*< Execute by others.  */
#define S_IRWXO (S_IRWXG >> 3)				/*< Read,Write,Execute by other */
// =============================================

///@brief used in posix.c to compare to ascii file modes
#define modecmp(str, pat) (strcmp(str, pat) == 0 ? 1: 0)

// =============================================
///@brief  FATFS open modes
#define FATFS_R (S_IRUSR | S_IRGRP | S_IROTH)	/*< FatFs Read perms */
#define FATFS_W (S_IWUSR | S_IWGRP | S_IWOTH)	/*< FatFs Write perms */
#define FATFS_X (S_IXUSR | S_IXGRP | S_IXOTH)	/*< FatFs Execute perms */

// =============================================
///@brief End of file or device read
#define EOF (-1)

///@brief Seek offset macros
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

// =============================================
///@brief define FILE type
typedef struct __file FILE;

///@brief Maximum number of POSIX file handles.
#define MAX_FILES 16
extern FILE *__iob[MAX_FILES];

///@brief define stdin, stdout and stderr
#undef stdin
#undef stdout
#undef stderr

// Hard coded stdin,stdout and stderr locations
#define stdin (__iob[0])
#define stdout (__iob[1])
#define stderr (__iob[2])

// =============================================
//#define IO_MACROS
#ifdef IO_MACROS
///@briefdefine putc, putchar and getc in terms of the posix fgetc() and fputc() interface
/// MAKE SURE that fdevopen() has been called BEFORE any input/output is processed
/// @see uart.c for the fdevopen() call
#define putc(__c, __stream) fputc(__c, __stream)
#define getc(__stream) fgetc(__stream)
/**
   The macro \c putchar sends character \c c to \c stdout.
*/
#define putchar(__c) fputc(__c,stdout)

#define puts(__str) fputs(__str,stdout)
#endif

// =============================================
///@brief device IO udata
#define fdev_set_udata(stream, u) do { (stream)->udata = u; } while(0)
#define fdev_get_udata(stream) ((stream)->udata)

///@brief  device status flags
#define _FDEV_EOF (-1)
#define _FDEV_ERR (-2)
//@brief device read/write flags
#define _FDEV_SETUP_READ  __SRD /**< fdev_setup_stream() with read intent */
#define _FDEV_SETUP_WRITE __SWR /**< fdev_setup_stream() with write intent */
#define _FDEV_SETUP_RW    (__SRD|__SWR) /**< fdev_setup_stream() with read/write intent */

// =============================================


/* posix.c */
int isatty ( int fileno );
int fgetc ( FILE *stream );
int fputc ( int c , FILE *stream );
void clearerr(FILE *stream);
#ifndef IO_MACROS
int getchar ( void );
int putchar ( int c );
#endif
//int ungetc ( int c , FILE *stream );
#ifndef IO_MACROS
int putc ( int c , FILE *stream );
#endif
char *fgets ( char *str , int size , FILE *stream );
int fputs ( const char *str , FILE *stream );
#ifndef IO_MACROS
int puts ( const char *str );
#endif
int feof ( FILE *stream );
int fgetpos ( FILE *stream , size_t *pos );
int fseek ( FILE *stream , long offset , int whence );
int fsetpos ( FILE *stream , size_t *pos );
long ftell ( FILE *stream );
off_t lseek ( int fileno , off_t position , int whence );
void rewind ( FILE *stream );
int close ( int fileno );
int fileno ( FILE *stream );
FILE *fileno_to_stream ( int fileno );
FILE *fopen ( const char *path , const char *mode );
size_t __wrap_fread ( void *ptr , size_t size , size_t nmemb , FILE *stream );
int ftruncate ( int fd , off_t length );
size_t fwrite ( const void *ptr , size_t size , size_t nmemb , FILE *stream );
int open (const char *pathname, int flags);
ssize_t read ( int fd , void *buf , size_t count );
void sync ( void );
int syncfs(int fd);
int fsync ( int fd );
int truncate ( const char *path , off_t length );
ssize_t write ( int fd , const void *buf , size_t count );
int __wrap_fclose ( FILE *stream );
FILE * __wrap_freopen ( const char * filename, const char * mode, FILE * stream );
int getc(FILE *fp);
char *gets (char *p);
//void dump_stat ( struct stat *sp );

#if 0
int fstat ( int fd , struct stat *buf );
#endif
int64_t fs_getfree(void);
int64_t fs_gettotal(void);
int stat ( const char *name , struct stat *buf );
char *basename (const char *str );
char *baseext ( char *str );
int chdir ( const char *pathname );
int chmod ( const char *pathname , mode_t mode );
int dirname ( char *str );
//int utime(const char *filename, const struct utimbuf *times);

#if 0
 int fchmod ( int fd , mode_t mode );
#endif

char *getcwd ( char *pathname , size_t len );
int mkdir ( const char *pathname , mode_t mode );
int rename ( const char *oldpath , const char *newpath );
int rmdir ( const char *pathname );
int unlink ( const char *pathname );
int __wrap_remove(const char *pathname);
int closedir ( DIR *dirp );
DIR *opendir ( const char *pathdir );
struct dirent *readdir ( DIR *dirp );
void clrerror ( FILE *stream );
int ferror ( FILE *stream );
char *strerror ( int errnum );
char *__wrap_strerror_r ( int errnum , char *buf , size_t buflen );
FILE *fdevopen ( int (*put )(char ,FILE *), int (*get )(FILE *));
//int mkfs(char *name );
int fatfs_getc ( FILE *stream );
int fatfs_putc ( char c , FILE *stream );
int fatfs_to_errno ( FRESULT Result );
int fatfs_to_fileno ( FIL *fh );
time_t fat_time_to_unix ( uint16_t date , uint16_t time );
void unix_time_to_fat(time_t epoch, uint16_t *date, uint16_t *time);
FIL *fileno_to_fatfs ( int fileno );
int free_file_descriptor ( int fileno );
int new_file_descriptor ( void );
int posix_fopen_modes_to_open ( const char *mode );

int __wrap_fprintf(FILE *fp, const char *format, ...);

#ifdef __cplusplus
}
#endif

// =============================================
#endif  //USE_POSIX
#endif                                            //_POSIX_H_

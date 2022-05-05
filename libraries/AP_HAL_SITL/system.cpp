#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include "Scheduler.h"

extern const AP_HAL::HAL& hal;

using HALSITL::Scheduler;

namespace AP_HAL {

static struct {
    struct timeval start_time;
} state;

void init()
{
    gettimeofday(&state.start_time, nullptr);
}

#if defined(__CYGWIN__) || defined(__CYGWIN64__) || defined(CYGWIN_BUILD)
void panic(const char *errormsg, ...)
#else
void WEAK panic(const char *errormsg, ...)
#endif
{
    va_list ap;

    fflush(stdout);
    printf("PANIC: ");
    va_start(ap, errormsg);
    vprintf(errormsg, ap);
    va_end(ap);
    printf("\n");

    dump_stack_trace();
    dump_core_file();

    if (getenv("SITL_PANIC_EXIT")) {
        // this is used on the autotest server to prevent us waiting
        // 10 hours for a timeout
        exit(1);
    }
    for(;;);
}

// partly flogged from: https://github.com/tridge/junkcode/blob/master/segv_handler/segv_handler.c
static void run_command_on_ownpid(const char *commandname)
{
    // find dumpstack command:
    const char *command_filepath = commandname; // if we can't find it trust in PATH
    struct stat statbuf;
    const char *paths[] {
        "Tools/scripts/%s",
        "APM/Tools/scripts/%s", // for autotest server
        "../Tools/scripts/%s", // when run from e.g. ArduCopter subdirectory
    };
    char buffer[60];
    for (uint8_t i=0; i<ARRAY_SIZE(paths); i++) {
        // form up a filepath from each path and commandname; if it
        // exists, use it
        snprintf(buffer, sizeof(buffer), paths[i], commandname);
        if (::stat(buffer, &statbuf) != -1) {
            command_filepath = buffer;
            break;
        }
    }

	char progname[100];
	int n = readlink("/proc/self/exe", progname, sizeof(progname)-1);
	if (n == -1) {
        strncpy(progname, "unknown", sizeof(progname));
        n = strlen(progname);
	}
	progname[n] = 0;

	char *p = strrchr(progname, '/');
    if (p != nullptr) {
	    *p = 0;
    } else {
        p = progname;
    }

    char output_filepath[80];
    snprintf(output_filepath,
             ARRAY_SIZE(output_filepath),
             "%s_%s.%d.out",
             commandname,
             p+1,
             (int)getpid());
    char cmd[200];
	snprintf(cmd,
             sizeof(cmd),
             "sh %s %d >%s 2>&1",
             command_filepath,
             (int)getpid(),
             output_filepath);
    fprintf(stderr, "Running: %s\n", cmd);

	if (system(cmd)) {
        fprintf(stderr, "Failed\n");
        return;
    }
    fprintf(stderr, "%s has been run.  Output was:\n", commandname);
    fprintf(stderr, "-------------- begin %s output ----------------\n", commandname);
    // print the trace on stderr:
    int fd = open(output_filepath, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "Failed to open stack dump filepath: %m");
        return;
    }
    char buf[1024]; // let's hope we're not here because we ran out of stack
    while (true) {
        const ssize_t ret = read(fd, buf, ARRAY_SIZE(buf));
        if (ret == -1) {
            fprintf(stderr, "Read error: %m");
            break;
        }
        if (ret == 0) {
            break;
        }
        if (write(2, buf, ret) != ret) {
            // *sigh*
            break;
        }
    }
    fprintf(stderr, "-------------- end %s output ----------------\n", commandname);
    close(fd);
}
void dump_stack_trace()
{
    run_command_on_ownpid("dumpstack.sh");
}
void dump_core_file()
{
    run_command_on_ownpid("dumpcore.sh");
}

uint32_t micros()
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis()
{
    return millis64() & 0xFFFFFFFF;
}

/*
  we define a millis16() here to avoid an issue with sitl builds in cygwin
 */
uint16_t millis16()
{
    return millis64() & 0xFFFF;
}
    
uint64_t micros64()
{
    const HALSITL::Scheduler* scheduler = HALSITL::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec;
    }

    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e6 * ((tp.tv_sec + (tp.tv_usec * 1.0e-6)) -
                            (state.start_time.tv_sec +
                             (state.start_time.tv_usec * 1.0e-6)));
    return ret;
}

uint64_t millis64()
{
    const HALSITL::Scheduler* scheduler = HALSITL::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec / 1000;
    }

    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) -
                          (state.start_time.tv_sec +
                           (state.start_time.tv_usec*1.0e-6)));
    return ret;
}


uint32_t native_micros()
{
    return native_micros64() & 0xFFFFFFFF;
}

uint32_t native_millis()
{
    return native_millis64() & 0xFFFFFFFF;
}

/*
  we define a millis16() here to avoid an issue with sitl builds in cygwin
 */
uint16_t native_millis16()
{
    return native_millis64() & 0xFFFF;
}
    

uint64_t native_micros64()
{
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e6 * ((tp.tv_sec + (tp.tv_usec * 1.0e-6)) -
                            (state.start_time.tv_sec +
                             (state.start_time.tv_usec * 1.0e-6)));
    return ret;
}

uint64_t native_millis64()
{
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    uint64_t ret = 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) -
                          (state.start_time.tv_sec +
                           (state.start_time.tv_usec*1.0e-6)));
    return ret;
}


} // namespace AP_HAL

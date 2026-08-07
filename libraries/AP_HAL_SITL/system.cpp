#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/system.h>

#include "Scheduler.h"
#include <AP_Math/div1000.h>

extern const AP_HAL::HAL& hal;

using HALSITL::Scheduler;

namespace AP_HAL {

static struct {
    uint64_t start_time_ns;
} state;

static uint64_t ts_to_nsec(struct timespec &ts)
{
    return ts.tv_sec*1000000000ULL + ts.tv_nsec;
}
    
void init()
{
    struct timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    state.start_time_ns = ts_to_nsec(ts);
}

void WEAK panic(const char *errormsg, ...)
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
        printf("panic and SITL_PANIC_EXIT set - exitting");
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
    const char *custom_scripts_dir_path = getenv("AP_SCRIPTS_DIR_PATH");
    char *custom_scripts_dir_path_pattern = nullptr;
    if (custom_scripts_dir_path != nullptr) {
        if (asprintf(&custom_scripts_dir_path_pattern, "%s/%%s", custom_scripts_dir_path) == -1) {
            custom_scripts_dir_path_pattern = nullptr;
        }
    }
    const char *paths[] {
        custom_scripts_dir_path_pattern,
        "Tools/scripts/%s",
        "APM/Tools/scripts/%s", // for autotest server
        "../Tools/scripts/%s", // when run from e.g. ArduCopter subdirectory
    };
    char buffer[60];
    for (uint8_t i=0; i<ARRAY_SIZE(paths); i++) {
        if (paths[i] == nullptr) {
            continue;
        }
        // form up a filepath from each path and commandname; if it
        // exists, use it
        snprintf(buffer, sizeof(buffer), paths[i], commandname);
        if (::stat(buffer, &statbuf) != -1) {
            command_filepath = buffer;
            break;
        }
    }
    free(custom_scripts_dir_path_pattern);

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
    
uint64_t micros64()
{
    const HALSITL::Scheduler* scheduler = HALSITL::Scheduler::from(hal.scheduler);
    uint64_t stopped_usec = scheduler->stopped_clock_usec();
    if (stopped_usec) {
        return stopped_usec;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_div1000(ts_to_nsec(ts) - state.start_time_ns);
}

uint64_t millis64()
{
    return uint64_div1000(micros64());
}

} // namespace AP_HAL

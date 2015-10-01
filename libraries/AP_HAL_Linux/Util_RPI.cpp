#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT

#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <istream>
#include <sys/types.h>
#include <signal.h>

#include "Util_RPI.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

LinuxUtilRPI::LinuxUtilRPI()
{
    _check_rpi_version();
}

#define MAX_SIZE_LINE 50
int LinuxUtilRPI::_check_rpi_version()
{
    char buffer[MAX_SIZE_LINE];
    const char* hardware_description_entry = "Hardware";
    const char* v1 = "BCM2708";
    const char* v2 = "BCM2709";
    char* flag;
    FILE* fd;

    fd = fopen("/proc/cpuinfo", "r");

    while (fgets(buffer, MAX_SIZE_LINE, fd) != NULL) {
        flag = strstr(buffer, hardware_description_entry);
        if (flag != NULL) {
            if (strstr(buffer, v2) != NULL) {
                printf("Raspberry Pi 2 with BCM2709!\n");
                fclose(fd);

                _rpi_version = 2;
                return _rpi_version;
            }
            else if (strstr(buffer, v1) != NULL) {
                printf("Raspberry Pi 1 with BCM2708!\n");
                fclose(fd);

                _rpi_version = 1;
                return _rpi_version;
            }
        }
    }

    /* defaults to 1 */
    fprintf(stderr, "Could not detect RPi version, defaulting to 1\n");
    fclose(fd);

    _rpi_version = 1;
    return _rpi_version;
}

pid_t LinuxUtilRPI::_find_process(const char* name)
{
    DIR* dir;
    struct dirent* ent;
    char buf[512];

    long  pid;
    char pname[100] = {0,};
    char state;
    FILE *fp=NULL; 

    if (!(dir = opendir("/proc"))) {
        perror("LinuxUtilRPI: can't open /proc");
        return -1;
    }

    while((ent = readdir(dir)) != NULL) {
        long lpid = atol(ent->d_name);
        if(lpid < 0)
            continue;
        snprintf(buf, sizeof(buf), "/proc/%ld/stat", lpid);
        fp = fopen(buf, "r");

        if (fp) {
            if ( (fscanf(fp, "%ld (%[^)]) %c", &pid, pname, &state)) != 3 ){
                printf("LinuxUtilRPI: fscanf failed \n");
                fclose(fp);
                closedir(dir);
                return -1; 
            }
            if (!strcmp(pname, name)) {
                fclose(fp);
                closedir(dir);
                return (pid_t)lpid;
            }
            fclose(fp);
        }
    }

    closedir(dir);
    return -1;
}

int LinuxUtilRPI::terminate_process(const char* name) 
{
    int ret = 0;
    pid_t pid = _find_process(name);
    if(pid > 0) {
        ret = kill(pid, SIGTERM);
        printf("LinuxUtilRPI: process %s was terminated with return value %d\n", name, ret);
    }
    
    return ret;
}

int LinuxUtilRPI::get_rpi_version() const
{
    return _rpi_version;
}

#endif

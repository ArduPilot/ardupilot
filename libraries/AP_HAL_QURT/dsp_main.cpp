#include <AP_HAL/AP_HAL.h>
#include "UDPDriver.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "Scheduler.h"
#include "Storage.h"
#include "replace.h"
#include <qurt_dsp.h>

extern const AP_HAL::HAL& hal;

extern "C" {
    int ArduPilot_main(int argc, const char *argv[]);
}

volatile int _last_dsp_line = __LINE__;
volatile const char *_last_dsp_file = __FILE__;
volatile uint32_t _last_counter;

static void *main_thread(void *cmdptr)
{
    char *cmdline = (char *)cmdptr;
    HAP_PRINTF("Ardupilot main_thread started");

    // break cmdline into argc/argv
    int argc = 0;
    for (int i=0; cmdline[i]; i++) {
        if (cmdline[i] == '\n') argc++;
    }
    const char **argv = (const char **)calloc(argc+2, sizeof(char *));
    argv[0] = &cmdline[0];
    argc = 0;
    for (int i=0; cmdline[i]; i++) {
        if (cmdline[i] == '\n') {
            cmdline[i] = 0;
            argv[argc+1] = &cmdline[i+1];
            argc++;
        }
    }
    argv[argc] = nullptr;
    
    ArduPilot_main(argc, argv);
    return NULL;
}


uint32_t ardupilot_start(const char *cmdline, int len)
{
    HAP_PRINTF("Starting Ardupilot");
    pthread_attr_t thread_attr;
    struct sched_param param;
    pthread_t thread_ctx;

    pthread_attr_init(&thread_attr);
    pthread_attr_setstacksize(&thread_attr, 80000);

    param.sched_priority = APM_MAIN_PRIORITY;
    (void)pthread_attr_setschedparam(&thread_attr, &param);

    pthread_create(&thread_ctx, &thread_attr, main_thread, (void *)strdup((char*)cmdline));
    return 0;
}

uint32_t ardupilot_heartbeat()
{
    HAP_PRINTF("tick %u %s:%d", (unsigned)_last_counter, _last_dsp_file, _last_dsp_line);
    return 0;
}

uint32_t ardupilot_set_storage(const uint8_t *buf, int len)
{
    if (len != sizeof(QURT::Storage::buffer)) {
        HAP_PRINTF("Incorrect storage size %u", len);
        return 1;
    }
    QURT::Storage::lock.take(0);
    memcpy(QURT::Storage::buffer, buf, len);
    QURT::Storage::lock.give();
    return 0;
}

uint32_t ardupilot_get_storage(uint8_t *buf, int len)
{
    if (len != sizeof(QURT::Storage::buffer)) {
        HAP_PRINTF("Incorrect storage size %u", len);
        return 1;
    }
    if (!QURT::Storage::dirty) {
        return 1;
    }
    QURT::Storage::lock.take(0);
    memcpy(buf, QURT::Storage::buffer, len);
    QURT::Storage::lock.give();
    return 0;
}


uint32_t ardupilot_socket_check(uint8_t *buf, int len, uint32_t *nbytes)
{
    return QURT::UDPDriver::from(hal.uartA)->socket_check(buf, len, nbytes);
}

uint32_t ardupilot_socket_input(const uint8_t *buf, int len, uint32_t *nbytes)
{
    return QURT::UDPDriver::from(hal.uartA)->socket_input(buf, len, nbytes);
}

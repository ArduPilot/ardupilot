/*
  main program for HAL_QURT port
 */
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <errno.h>
#include <AP_HAL/utility/Socket.h>
#include <qurt_dsp.h>

static SocketAPM sock{true};
static bool connected;
static uint32_t last_get_storage_us;
static uint64_t start_time;

// location of virtual eeprom in Linux filesystem
#define STORAGE_DIR "/var/APM"
#define STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"

extern const char *get_ipv4_broadcast(void);

// time since startup in microseconds
static uint64_t micros64()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t ret = ts.tv_sec*1000*1000ULL + ts.tv_nsec/1000U;
    if (start_time == 0) {
        start_time = ret;
    }
    ret -= start_time;
    return ret;
}

/*
  send storage file to DSPs
 */
static void send_storage(void)
{
    int fd = open(STORAGE_FILE, O_RDWR|O_CREAT, 0644);
    if (fd == -1) {
        printf("Unable to open %s", STORAGE_FILE);
        exit(1);
    }
    uint8_t buf[16384];
    memset(buf, 0, sizeof(buf));
    read(fd, buf, sizeof(buf));
    if (ardupilot_set_storage(buf, sizeof(buf)) != 0) {
        printf("Failed to send initial storage");
        exit(1);        
    }
    close(fd);
}

/*
  get updated storage file from DSPs
 */
static void get_storage(void)
{
    uint8_t buf[16384];
    if (ardupilot_get_storage(buf, sizeof(buf)) != 0) {
        return;
    }
    int fd = open(STORAGE_FILE ".new", O_WRONLY|O_CREAT|O_TRUNC, 0644);
    if (fd == -1) {
        printf("Unable to open %s - %s\n", STORAGE_FILE ".new", strerror(errno));
    }
    write(fd, buf, sizeof(buf));
    close(fd);
    // atomic rename
    if (rename(STORAGE_FILE ".new", STORAGE_FILE) != 0) {
        printf("Unable to rename to %s - %s\n", STORAGE_FILE, strerror(errno));
    }
}

/*
  handle any incoming or outgoing UDP socket data on behalf of the DSPs
 */
static void socket_check(void)
{
    static const char *bcast = NULL;
    uint8_t buf[300];
    ssize_t ret = sock.recv(buf, sizeof(buf), 0);
    if (ret > 0) {
        uint32_t nbytes;
        ardupilot_socket_input(buf, ret, &nbytes);
        if (!connected) {
            const char *ip;
            uint16_t port;
            sock.last_recv_address(ip, port);
            connected = sock.connect(ip, port);
            if (connected) {
                printf("Connected to UDP %s:%u\n", ip, (unsigned)port);
            }
            sock.set_blocking(false);
        }
    }
    uint32_t nbytes;
    if (bcast == NULL) {
        bcast = get_ipv4_broadcast();
        if (bcast == NULL) {
            bcast = "255.255.255.255";
        }
        printf("Broadcasting to %s\n", bcast);
    }
    if (ardupilot_socket_check(buf, sizeof(buf), &nbytes) == 0) {
        if (!connected) {
            sock.sendto(buf, nbytes, bcast, 14550);
        } else {
            sock.send(buf, nbytes);
        }
    }
}

/*
  encode argv/argv as a sequence separated by \n
 */
static char *encode_argv(int argc, const char *argv[])
{
    uint32_t len = 0;
    for (int i=0; i<argc; i++) {
        len += strlen(argv[i]) + 1;
    }
    char *ret = (char *)malloc(len+1);
    char *p = ret;
    for (int i=0; i<argc; i++) {
        size_t slen = strlen(argv[i]);
        strcpy(p, argv[i]);
        p[slen] = '\n';
        p += slen + 1;
    }
    *p = 0;
    return ret;
}

/*
  main program
 */
int main(int argc, const char *argv[])
{
    sock.set_broadcast();

    printf("Starting DSP code\n");
    send_storage();

    char *cmdline = encode_argv(argc, argv);
    ardupilot_start(cmdline, strlen(cmdline));
    free(cmdline);
    while (true) {
        uint64_t now = micros64();
        if (now - last_get_storage_us > 1000*1000) {
            printf("tick t=%.6f\n", now*1.0e-6f);
            ardupilot_heartbeat();
            get_storage();
            last_get_storage_us = now;
        }
        socket_check();
        usleep(5000);
    }
}

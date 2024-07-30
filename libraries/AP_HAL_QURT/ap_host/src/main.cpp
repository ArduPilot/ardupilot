
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>

#include "slpi_link.h"
#include "protocol.h"
#include "getifaddrs.h"

volatile bool _running = false;
static bool enable_debug = false;
static bool enable_remote_debug = false;

static int gcs_socket_fd;
static int obd_socket_fd;
static bool gcs_connected;
static bool obd_connected;
static struct sockaddr_in gcs_addr;
static struct sockaddr_in obd_addr;

#define SO_NAME "ArduPilot.so"

// setup for mavlink to localhost
#define MAVLINK_UDP_LOCALHOST 1

// Ports for onboard stream
#define MAVLINK_OBD_UDP_PORT_LOCAL 14556
#define MAVLINK_OBD_UDP_PORT_REMOTE 14557

// Ports for external GCS stream
#define MAVLINK_GCS_UDP_PORT_LOCAL 14558
#define MAVLINK_GCS_UDP_PORT_REMOTE 14559


// directory for logs, parameters, terrain etc
#define DATA_DIRECTORY "/data/APM"

static void shutdown_signal_handler(int signo)
{
    switch (signo) {
    case SIGINT: // normal ctrl-c shutdown interrupt
        _running = false;
        fprintf(stderr, "\nreceived SIGINT Ctrl-C\n");
        break;
    case SIGTERM: // catchable terminate signal
        _running = false;
        fprintf(stderr, "\nreceived SIGTERM\n");
        break;
    case SIGHUP:
        // terminal closed or disconnected, carry on anyway
        fprintf(stderr, "\nreceived SIGHUP, continuing anyway\n");
        break;
    default:
        fprintf(stderr, "\nreceived signal %d\n", signo);
        break;
    }
    return;
}

static void slpi_init(void);

static uint32_t num_params = 0;
static uint32_t expected_seq[MAX_MAVLINK_INSTANCES] = {0, 0};

static void receive_callback(const uint8_t *data, uint32_t length_in_bytes)
{
    if (enable_debug) {
        printf("Got %u bytes in receive callback\n", length_in_bytes);
    }
    const auto *msg = (const struct qurt_rpc_msg *)data;
    if (length_in_bytes < QURT_RPC_MSG_HEADER_LEN) {
        printf("Invalid length_in_bytes %d", length_in_bytes);
        return;
    }
    if (msg->data_length + QURT_RPC_MSG_HEADER_LEN != length_in_bytes) {
        printf("Invalid lengths %d %d\n", msg->data_length, length_in_bytes);
        return;
    }
    if (msg->inst < MAX_MAVLINK_INSTANCES) {
        if (msg->seq != expected_seq[msg->inst]) {
            printf("Invalid seq %u %u\n", msg->seq, expected_seq[msg->inst]);
        }
    } else {
        printf("Invalid instance %u\n", msg->inst);
    }
    expected_seq[msg->inst] = msg->seq + 1;

    switch (msg->msg_id) {
    case QURT_MSG_ID_MAVLINK_MSG: {
        if (_running) {
            if (msg->inst == 0) {
                const auto bytes_sent = sendto(gcs_socket_fd, msg->data, msg->data_length, 0, (struct sockaddr *)&gcs_addr, sizeof(gcs_addr));
                if (bytes_sent <= 0) {
                    fprintf(stderr, "Send to GCS failed\n");
                }
            } else if (msg->inst == 1) {
                const auto bytes_sent = sendto(obd_socket_fd, msg->data, msg->data_length, 0, (struct sockaddr *)&obd_addr, sizeof(obd_addr));
                if (bytes_sent <= 0) {
                    fprintf(stderr, "Send to onboard failed\n");
                }
            }
        }
        break;
    }
    case QURT_MSG_ID_REBOOT: {
        printf("Rebooting\n");
        exit(0);
        break;
    }
    default:
        fprintf(stderr, "Got unknown message id %d\n", msg->msg_id);
        break;
    }
}

static void slpi_init(void)
{
    int r;
    while ((r=slpi_link_init(enable_remote_debug, &receive_callback, SO_NAME)) != 0) {
        fprintf(stderr, "slpi_link_init error %d %s, retrying\n", r, strerror(errno));
        sleep(1);
    }
}

/*
  setup directories for the hexagon code to use
 */
static void setup_directores(void)
{
    struct stat st;

    mkdir(DATA_DIRECTORY, 0777);
    chmod(DATA_DIRECTORY, 0777);
    if (stat(DATA_DIRECTORY, &st) != 0 || !S_ISDIR(st.st_mode)) {
        printf("Unable to create %s", DATA_DIRECTORY);
        exit(1);
    }
}

void *obd_recv_thread(void *)
{

    uint32_t next_seq = 0;

    printf("Waiting for OBD receive\n");

    while (_running) {
        struct qurt_rpc_msg msg {};
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        uint32_t bytes_received = recvfrom(obd_socket_fd, msg.data, sizeof(msg.data), 0,
                                           (struct sockaddr*)&from, &fromlen);
        if (bytes_received > 0 && !obd_connected) {
            obd_addr = from;
            obd_connected = true;
            printf("Connnected to OBD addr %s\n", inet_ntoa(from.sin_addr));
        }
        if (bytes_received < 0) {
            fprintf(stderr, "OBD receive failed");
            continue;
        }
        if (bytes_received > sizeof(msg.data)) {
            printf("Invalid bytes_received %d\n", bytes_received);
            continue;
        }
        msg.msg_id = QURT_MSG_ID_MAVLINK_MSG;
        msg.inst = 1;
        msg.data_length = bytes_received;
        msg.seq = next_seq++;
        // printf("Message received. %d bytes\n", bytes_received);
        if (slpi_link_send((const uint8_t*) &msg, bytes_received + QURT_RPC_MSG_HEADER_LEN)) {
            fprintf(stderr, "slpi_link_send_data failed for instance 1\n");
        }
    }

    return NULL;
}

int main()
{
    printf("Starting up\n");

    setup_directores();

    // make the sigaction struct for shutdown signals
    // sa_handler and sa_sigaction is a union, only set one
    struct sigaction action;
    action.sa_handler = shutdown_signal_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;

    // set actions
    if (sigaction(SIGINT, &action, NULL) < 0) {
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if (sigaction(SIGTERM, &action, NULL) < 0) {
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }
    if (sigaction(SIGHUP, &action, NULL) < 0) {
        fprintf(stderr, "ERROR: failed to set sigaction\n");
        return -1;
    }

    slpi_init();

    //initialize sockets and structures
    gcs_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (gcs_socket_fd == -1) {
        fprintf(stderr, "Could not create GCS socket");
        return -1;
    }
    obd_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (obd_socket_fd == -1) {
        fprintf(stderr, "Could not create OBD socket");
        return -1;
    }

#if MAVLINK_UDP_LOCALHOST
    // send to mavlink router on localhost for GCS
    gcs_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    gcs_addr.sin_family = AF_INET;
    gcs_addr.sin_port = htons(MAVLINK_GCS_UDP_PORT_REMOTE);

    struct sockaddr_in gcs_local {};
    gcs_local.sin_addr.s_addr = INADDR_ANY;
    gcs_local.sin_family = AF_INET;
    gcs_local.sin_port = htons(MAVLINK_GCS_UDP_PORT_LOCAL);

    if (bind(gcs_socket_fd, (struct sockaddr *)&gcs_local, sizeof(gcs_local)) == 0) {
        printf("Bind localhost GCS socket OK\n");
    } else {
        printf("Bind localhost GCS socket failed: %s", strerror(errno));
    }

    // send to mavlink router on localhost for onboard stream
    obd_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    obd_addr.sin_family = AF_INET;
    obd_addr.sin_port = htons(MAVLINK_OBD_UDP_PORT_REMOTE);

    struct sockaddr_in obd_local {};
    obd_local.sin_addr.s_addr = INADDR_ANY;
    obd_local.sin_family = AF_INET;
    obd_local.sin_port = htons(MAVLINK_OBD_UDP_PORT_LOCAL);

    if (bind(obd_socket_fd, (struct sockaddr *)&obd_local, sizeof(obd_local)) == 0) {
        printf("Bind localhost OBD socket OK\n");
    } else {
        printf("Bind localhost OBD socket failed: %s", strerror(errno));
    }
#else
    // broadcast directly to the local network broadcast address
    const char *bcast_address = get_ipv4_broadcast();
    printf("Broadcast address=%s\n", bcast_address);
    inet_aton(bcast_address, &gcs_addr.sin_addr);
    gcs_addr.sin_family = AF_INET;
    gcs_addr.sin_port = htons(UDP_OUT_PORT);

    int one = 1;
    setsockopt(socket_fd,SOL_SOCKET,SO_BROADCAST,(char *)&one,sizeof(one));

    struct sockaddr_in any {};
    any.sin_addr.s_addr = INADDR_ANY;
    any.sin_port = htons(15550);

    if (bind(socket_fd, (struct sockaddr *)&any, sizeof(any)) == 0) {
        printf("Bind OK\n");
    } else {
        printf("Bind failed: %s", strerror(errno));
    }
#endif

    printf("Enter ctrl-c to exit\n");
    _running = true;

    pthread_t obd_recv_thread_id;
    pthread_attr_t obd_recv_thread_attr;
    pthread_attr_init(&obd_recv_thread_attr);
    pthread_create(&obd_recv_thread_id, &obd_recv_thread_attr, obd_recv_thread, NULL);

    uint32_t next_seq = 0;

    printf("Waiting for GCS receive\n");

    while (_running) {
        struct qurt_rpc_msg msg {};
        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        uint32_t bytes_received = recvfrom(gcs_socket_fd, msg.data, sizeof(msg.data), 0,
                                           (struct sockaddr*)&from, &fromlen);
        if (bytes_received > 0 && !gcs_connected) {
            gcs_addr = from;
            gcs_connected = true;
            printf("Connnected to GCS at %s\n", inet_ntoa(from.sin_addr));
        }
        if (bytes_received < 0) {
            fprintf(stderr, "GCS receive failed");
            continue;
        }
        if (bytes_received > sizeof(msg.data)) {
            printf("Invalid bytes_received %d\n", bytes_received);
            continue;
        }
        msg.msg_id = QURT_MSG_ID_MAVLINK_MSG;
        msg.inst = 0;
        msg.data_length = bytes_received;
        msg.seq = next_seq++;
        // printf("Message received. %d bytes\n", bytes_received);
        if (slpi_link_send((const uint8_t*) &msg, bytes_received + QURT_RPC_MSG_HEADER_LEN)) {
            fprintf(stderr, "slpi_link_send_data failed for instance 0\n");
        }
    }

    printf("Reseting SLPI\n");
    // Send reset to SLPI
    slpi_link_reset();
    sleep(1);

    printf("Exiting\n");

    return 0;
}


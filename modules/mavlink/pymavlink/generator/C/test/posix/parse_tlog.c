/*
  parse a tlog, for performance testing
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_USE_MESSAGE_INFO
#define MAVLINK_COMM_NUM_BUFFERS 2

// this trick allows us to make mavlink_message_t as small as possible
// for this dialect, which saves some memory
#include <version.h>
#define MAVLINK_MAX_PAYLOAD_LEN MAVLINK_MAX_DIALECT_PAYLOAD_SIZE

#include <mavlink_types.h>
static mavlink_system_t mavlink_system = {42,11,};

#define MAVLINK_ASSERT(x) assert(x)
static void comm_send_ch(mavlink_channel_t chan, uint8_t c) {}

#include <mavlink.h>

int main(int argc, const char *argv[])
{
    if (argc < 2) {
	printf("Usage: parse_tlog FILENAME\n");
	exit(1);
    }
    const char *filename = argv[1];
    int fd = open(filename, O_RDONLY);
    if (fd == -1) {
	perror(filename);
	exit(1);
    }
    mavlink_channel_t chan = MAVLINK_COMM_0;
    char c;
    uint32_t msg_count = 0;
    mavlink_message_t last_msg;
    mavlink_status_t status;

    while (read(fd, &c, 1) == 1) {
	if (mavlink_parse_char(chan, c, &last_msg, &status)) {
	    msg_count++;
	}
    }
    printf("parsed %u messages\n", (unsigned)msg_count);

    return 0;
}


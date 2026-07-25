/*
  test specific issues
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

/*
  test for issue in https://github.com/ArduPilot/pymavlink/issues/1019
 */
static void test_issue_1019(void)
{
    uint8_t TestBuffer[]={/* 0xFD,0x1C, */ 0x00,0x00,0xFD,0x01,0x01,0x1E,0x00,0x00,0xE5,0x87,0x10,0x00,0x7c,0xdd,0x8c,0xba,0xfb,0xa6,0xa6,0xba,0x74,0x5a,0xdc,0xbc,0x98,0x97,0x99,0xb9};
    uint32_t msg_count = 0;
    mavlink_message_t last_msg, rmsg;
    mavlink_status_t status, rstatus;

    printf("Testing issue 1019\n");

    memset(&status, 0, sizeof(status));
    memset(&last_msg, 0, sizeof(last_msg));

    for (uint8_t i=0; i<sizeof(TestBuffer); i++) {
        uint8_t c = TestBuffer[i];
        uint8_t res = mavlink_frame_char_buffer(&last_msg, &status, c, &rmsg, &rstatus);
        if (res == 1) {
            printf("Got message %u\n", (unsigned)rmsg.msgid);
            msg_count++;
        }
    }
    // we should not have received a message
    if (msg_count > 0) {
        printf("Issue1019: incorrectly received corrupt message\n");
        exit(1);
    }
}


int main(int argc, const char *argv[])
{
    test_issue_1019();

    return 0;
}


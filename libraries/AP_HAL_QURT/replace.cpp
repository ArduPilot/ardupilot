#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <pthread.h>
#include "replace.h"
#include "interface.h"
#include "ap_host/src/protocol.h"

extern "C" {

    // this is not declared in qurt headers
    void HAP_debug(const char *msg, int level, const char *filename, int line);

    void HAP_printf(const char *file, int line, const char *format, ...)
    {
        va_list ap;
        char buf[300];

        va_start(ap, format);
        vsnprintf(buf, sizeof(buf), format, ap);
        va_end(ap);
        HAP_debug(buf, 0, file, line);
        //usleep(20000);
    }

    /**
       QURT doesn't have strnlen()
    **/
    size_t strnlen(const char *s, size_t max)
    {
        size_t len;

        for (len = 0; len < max; len++) {
            if (s[len] == '\0') {
                break;
            }
        }
        return len;
    }

    int vasprintf(char **ptr, const char *format, va_list ap)
    {
        int ret;
        va_list ap2;

        va_copy(ap2, ap);
        ret = vsnprintf(nullptr, 0, format, ap2);
        va_end(ap2);
        if (ret < 0) {
            return ret;
        }

        (*ptr) = (char *)malloc(ret+1);
        if (!*ptr) {
            return -1;
        }

        va_copy(ap2, ap);
        ret = vsnprintf(*ptr, ret+1, format, ap2);
        va_end(ap2);

        return ret;
    }

    int asprintf(char **ptr, const char *format, ...)
    {
        va_list ap;
        int ret;

        *ptr = nullptr;
        va_start(ap, format);
        ret = vasprintf(ptr, format, ap);
        va_end(ap);

        return ret;
    }

    void *memmem(const void *haystack, size_t haystacklen,
                 const void *needle, size_t needlelen)
    {
        if (needlelen == 0) {
            return const_cast<void*>(haystack);
        }
        while (haystacklen >= needlelen) {
            char *p = (char *)memchr(haystack, *(const char *)needle,
                                     haystacklen-(needlelen-1));
            if (!p) {
                return NULL;
            }
            if (memcmp(p, needle, needlelen) == 0) {
                return p;
            }
            haystack = p+1;
            haystacklen -= (p - (const char *)haystack) + 1;
        }
        return NULL;
    }

    char *strndup(const char *s, size_t n)
    {
        char *ret;

        n = strnlen(s, n);
        ret = (char*)malloc(n+1);
        if (!ret) {
            return NULL;
        }
        memcpy(ret, s, n);
        ret[n] = 0;

        return ret;
    }

    int pthread_cond_init(pthread_cond_t *cond, pthread_condattr_t *attr)
    {
        return 0;
    }

    // INVESTIGATE: What is this needed on QURT?
    int apfs_rename(const char *oldpath, const char *newpath)
    {
        return 0;
    }

    // INVESTIGATE: Seems important :-)
    int ArduPilot_main(int argc, const char *argv[])
    {
        return 0;
    }

    char *__wrap_strdup(const char *s);
}

extern "C" int qurt_ardupilot_main(int argc, char* const argv[]);

int slpi_link_client_init(void)
{
    HAP_PRINTF("About to call qurt_ardupilot_main %p", &qurt_ardupilot_main);

    qurt_ardupilot_main(0, NULL);

    return 0;
}

typedef void (*mavlink_data_callback_t)(const struct qurt_rpc_msg *msg, void* p);

static mavlink_data_callback_t mav_cb[MAX_MAVLINK_INSTANCES];
static void *mav_cb_ptr[MAX_MAVLINK_INSTANCES];

void register_mavlink_data_callback(uint8_t instance, mavlink_data_callback_t func, void *p)
{
	if (instance < MAX_MAVLINK_INSTANCES) {
	    mav_cb[instance] = func;
	    mav_cb_ptr[instance] = p;
	} else {
        HAP_PRINTF("Error: Invalid mavlink instance %u", instance);
	}
}

int slpi_link_client_receive(const uint8_t *data, int data_len_in_bytes)
{
    if (data_len_in_bytes < QURT_RPC_MSG_HEADER_LEN) {
        return 0;
    }
    const auto *msg = (struct qurt_rpc_msg *)data;
    if (msg->data_length + QURT_RPC_MSG_HEADER_LEN != data_len_in_bytes) {
        return 0;
    }

    switch (msg->msg_id) {
    case QURT_MSG_ID_MAVLINK_MSG: {
        if ((msg->inst < MAX_MAVLINK_INSTANCES) && (mav_cb[msg->inst])) {
            mav_cb[msg->inst](msg, mav_cb_ptr[msg->inst]);
        }
        break;
    }
    default:
        HAP_PRINTF("Got unknown message id %d", msg->msg_id);
        break;
    }

    return 0;
}

int __wrap_printf(const char *fmt, ...)
{
    va_list ap;

    char buf[300];
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    HAP_PRINTF(buf);

    return 0;
}

/*
  free() on strdup return on QURT causes a memory fault
 */
char *__wrap_strdup(const char *s)
{
    return strndup(s, strlen(s));
}

/*
  send a RPC message to the host
 */
bool qurt_rpc_send(struct qurt_rpc_msg &msg)
{
    if (msg.data_length > sizeof(msg.data)) {
        return false;
    }
    return sl_client_send_data((const uint8_t*)&msg, msg.data_length + QURT_RPC_MSG_HEADER_LEN) >= 0;
}

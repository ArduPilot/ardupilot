/*
  a set of tests for networking
 */

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_TESTS_ENABLED
#include "AP_Networking.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/Socket.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
  start tests
 */
void AP_Networking::start_tests(void)
{
    if (param.tests & TEST_UDP_CLIENT) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_UDP_client, void),
                                     "UDP_client",
                                     8192, AP_HAL::Scheduler::PRIORITY_IO, -1);
    }
    if (param.tests & TEST_TCP_CLIENT) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_TCP_client, void),
                                     "TCP_client",
                                     8192, AP_HAL::Scheduler::PRIORITY_IO, -1);
    }
}

/*
  start UDP client test
 */
void AP_Networking::test_UDP_client(void)
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(1000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP_client: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = new SocketAPM(true);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UDP_client: failed to create socket");
        return;
    }
    // connect to the echo service, which is port 7
    if (!sock->connect(dest, 7)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "UDP_client: connect failed");
        return;
    }
    while (true) {
        hal.scheduler->delay(100);
        char *s = nullptr;
        IGNORE_RETURN(asprintf(&s, "hello %u", unsigned(AP_HAL::millis())));
        sock->send((const void*)s, strlen(s));
        free(s);
        uint8_t buf[128] {};
        const ssize_t ret = sock->recv(buf, sizeof(buf), 10);
        if (ret > 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP_client: reply '%s'", (const char *)buf);
        }
    }
}

/*
  start TCP client test
 */
void AP_Networking::test_TCP_client(void)
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay(100);
    }
    hal.scheduler->delay(1000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_client: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = new SocketAPM(false);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TCP_client: failed to create socket");
        return;
    }
    // connect to the echo service, which is port 7
    if (!sock->connect(dest, 7)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TCP_client: connect failed");
        return;
    }
    while (true) {
        hal.scheduler->delay(100);
        char *s = nullptr;
        IGNORE_RETURN(asprintf(&s, "hello %u", unsigned(AP_HAL::millis())));
        sock->send((const void*)s, strlen(s));
        free(s);
        uint8_t buf[128] {};
        const ssize_t ret = sock->recv(buf, sizeof(buf), 10);
        if (ret > 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_client: reply '%s'", (const char *)buf);
        }
    }
}

#endif // AP_NETWORKING_ENABLED && AP_NETWORKING_TESTS_ENABLED

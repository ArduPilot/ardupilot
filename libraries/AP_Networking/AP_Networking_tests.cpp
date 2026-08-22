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
    const uint32_t enabled_tests = uint32_t(param.tests.get());
    tests_started &= enabled_tests;
    const uint32_t new_tests = enabled_tests & ~tests_started;
    tests_started |= new_tests;

    if (new_tests & TEST_UDP_CLIENT) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_UDP_client, void),
                                     "UDP_client",
                                     8192, AP_HAL::Scheduler::PRIORITY_IO, -1);
    }
    if (new_tests & TEST_TCP_CLIENT) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_TCP_client, void),
                                     "TCP_client",
                                     8192, AP_HAL::Scheduler::PRIORITY_IO, -1);
    }
    if (new_tests & TEST_TCP_DISCARD) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_TCP_discard, void),
                                     "TCP_discard",
                                     8192, AP_HAL::Scheduler::PRIORITY_UART, -1);
    }
    if (new_tests & TEST_TCP_DISCARD_ONESHOT) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_TCP_discard_oneshot, void),
                                     "TCP_discard_once",
                                     8192, AP_HAL::Scheduler::PRIORITY_UART, -1);
    }
    if (new_tests & TEST_TCP_REFLECT) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_TCP_reflect, void),
                                     "TCP_reflect",
                                     8192, AP_HAL::Scheduler::PRIORITY_UART, -1);
    }
    if (new_tests & TEST_TCP_DISCARD_SERVER) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_TCP_discard_server, void),
                                     "TCP_discard_server",
                                     8192, AP_HAL::Scheduler::PRIORITY_UART, -1);
    }
    if (new_tests & TEST_CONNECTOR_LOOPBACK) {
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking::test_connector_loopback, void),
                                     "connector_loopback",
                                     8192, AP_HAL::Scheduler::PRIORITY_IO, -1);
    }
}

/*
  start UDP client test
 */
void AP_Networking::test_UDP_client(void)
{
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "UDP_client: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = NEW_NOTHROW SocketAPM(true);
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
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_client: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = NEW_NOTHROW SocketAPM(false);
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

/*
  start TCP discard (throughput) test
 */
void AP_Networking::test_TCP_discard(void)
{
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_discard: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = NEW_NOTHROW SocketAPM(false);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TCP_discard: failed to create socket");
        return;
    }
    // connect to the discard service, which is port 9
    while (!sock->connect(dest, 9)) {
        hal.scheduler->delay(10);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_discard: connected");
    const uint32_t bufsize = 1024;
    uint8_t *buf = (uint8_t*)malloc(bufsize);
    for (uint32_t i=0; i<bufsize; i++) {
        buf[i] = i & 0xFF;
    }
    uint32_t last_report_ms = AP_HAL::millis();
    uint32_t total_sent = 0;
    while (true) {
        if ((param.tests & TEST_TCP_DISCARD) == 0) {
            hal.scheduler->delay(1);
            continue;
        }
        total_sent += sock->send(buf, bufsize);
        const uint32_t now = AP_HAL::millis();
        if (now - last_report_ms >= 1000) {
            float dt = (now - last_report_ms)*0.001;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Discard throughput %.3f kbyte/sec", (total_sent/dt)*1.0e-3);
            total_sent = 0;
            last_report_ms = now;
        }
    }
}

/*
  run a bounded TCP discard test for integration tests
 */
void AP_Networking::test_TCP_discard_oneshot(void)
{
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_discard: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = NEW_NOTHROW SocketAPM(false);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "TCP_discard: failed to create socket");
        return;
    }
    while (!sock->connect(dest, 9)) {
        hal.scheduler->delay(10);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_discard: connected");

    uint8_t buf[1024];
    for (uint32_t i = 0; i < sizeof(buf); i++) {
        buf[i] = i & 0xFF;
    }
    const uint32_t start_ms = AP_HAL::millis();
    uint32_t last_report_ms = start_ms;
    uint32_t total_sent = 0;
    while (AP_HAL::millis() - start_ms < 15000U) {
        if (!sock->pollout(100)) {
            continue;
        }
        const ssize_t ret = sock->send(buf, sizeof(buf));
        if (ret <= 0) {
            break;
        }
        total_sent += ret;
        const uint32_t now = AP_HAL::millis();
        if (now - last_report_ms >= 1000) {
            const float dt = (now - last_report_ms) * 0.001;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "Discard throughput %.3f kbyte/sec",
                          (total_sent / dt) * 1.0e-3);
            total_sent = 0;
            last_report_ms = now;
        }
    }
    delete sock;
}

/*
  start TCP discard server for throughput tests
 */
void AP_Networking::test_TCP_discard_server(void)
{
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_discard_server: starting");

    auto *listen_sock = NEW_NOTHROW SocketAPM(false);
    if (listen_sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "TCP_discard_server: failed to create socket");
        return;
    }
    listen_sock->reuseaddress();
    if (!listen_sock->bind("0.0.0.0", 9)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "TCP_discard_server: failed to bind");
        delete listen_sock;
        return;
    }
    if (!listen_sock->listen(1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                      "TCP_discard_server: failed to listen");
        delete listen_sock;
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_discard_server: listening");

    uint8_t buf[1024];
    while (param.tests & TEST_TCP_DISCARD_SERVER) {
        auto *sock = listen_sock->accept(1000);
        if (sock == nullptr) {
            continue;
        }

        uint32_t last_report_ms = AP_HAL::millis();
        uint32_t total_received = 0;
        while (param.tests & TEST_TCP_DISCARD_SERVER) {
            if (!sock->pollin(100)) {
                continue;
            }
            const ssize_t ret = sock->recv(buf, sizeof(buf), 0);
            if (ret <= 0) {
                break;
            }
            total_received += ret;
            const uint32_t now = AP_HAL::millis();
            if (now - last_report_ms >= 1000) {
                const float dt = (now - last_report_ms) * 0.001;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "Discard server %.3f kbyte/sec",
                              (total_received / dt) * 1.0e-3);
                total_received = 0;
                last_report_ms = now;
            }
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "Discard server: disconnected");
        delete sock;
    }
    delete listen_sock;
}

/*
  start TCP reflect (TCP echo throughput) test
 */
void AP_Networking::test_TCP_reflect(void)
{
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_reflect: starting");
    const char *dest = param.test_ipaddr.get_str();
    auto *sock = new SocketAPM(false);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TCP_reflect: failed to create socket");
        return;
    }
    // connect to the echo service, which is port 7
    while (!sock->connect(dest, 7)) {
        hal.scheduler->delay(10);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TCP_reflect: connected");
    const uint32_t bufsize = 4096;
    uint8_t *buf = (uint8_t*)malloc(bufsize);
    for (uint32_t i=0; i<bufsize; i++) {
        buf[i] = i & 0xFF;
    }
    uint32_t last_report_ms = AP_HAL::millis();
    uint32_t total_sent = 0;
    uint32_t total_recv = 0;
    uint32_t last_recv = 0;
    const uint32_t max_disparity = 100*1024;
    while (true) {
        if ((param.tests & TEST_TCP_REFLECT) == 0) {
            hal.scheduler->delay(1);
            continue;
        }
        const bool will_send = total_sent < total_recv + max_disparity;
        if (will_send) {
            total_sent += sock->send(buf, bufsize);
        }
        if (sock->pollin(0)) {
            uint32_t n = sock->recv(buf, bufsize, 0);
            if (n == 0 && !will_send) {
                hal.scheduler->delay_microseconds(100);
            }
            total_recv += n;
        }
        const uint32_t now = AP_HAL::millis();

        if (now - last_report_ms >= 1000) {
            float dt = (now - last_report_ms)*0.001;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reflect throughput %.3f kbyte/sec (disparity %u)", ((total_recv-last_recv)/dt)*1.0e-3, unsigned(total_sent-total_recv));
            last_recv = total_recv;
            last_report_ms = now;
        }
    }
}

void AP_Networking::test_connector_loopback(void)
{
    startup_wait();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Connector Loopback: starting");

    // start tcp discard server
    auto *listen_sock = new SocketAPM(false);
    if (listen_sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "connector_loopback: failed to create socket");
        return;
    }
    listen_sock->reuseaddress();

    // use netif ip address
    char ipstr[16];
    SocketAPM::inet_addr_to_str(get_ip_active(), ipstr, sizeof(ipstr));
    if (!listen_sock->bind(ipstr, 9)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "connector_loopback: failed to bind");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "connector_loopback: bound");

    if (!listen_sock->listen(1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "connector_loopback: failed to listen");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "connector_loopback: listening");
    // create discard client
    auto *client = new SocketAPM(false);
    if (client == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "connector_loopback: failed to create client");
        return;
    }
    while (!client->connect(ipstr, 9)) {
        hal.scheduler->delay(10);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "connector_loopback: connected");

    // accept client
    auto *sock = listen_sock->accept(100);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "connector_loopback: failed to accept");
        return;
    }

    uint8_t buf[1024];
    uint32_t last_report_ms = AP_HAL::millis();
    uint32_t total_rx = 0;
    while (true) {
        if ((param.tests & TEST_CONNECTOR_LOOPBACK) == 0) {
            hal.scheduler->delay(1);
            continue;
        }
        client->send(buf, 1024);
        if (sock->pollin(0)) {
            const ssize_t ret = sock->recv(buf, sizeof(buf), 0);
            if (ret > 0) {
                total_rx += ret;
            }
        }
        if (AP_HAL::millis() - last_report_ms >= 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "connector_loopback throughput %u kbytes/sec", unsigned(total_rx/1024));
            total_rx = 0;
            last_report_ms = AP_HAL::millis();
        }
    }
}

#endif // AP_NETWORKING_ENABLED && AP_NETWORKING_TESTS_ENABLED

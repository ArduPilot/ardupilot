/*
  support for networking enabled bootloader
 */

#include "AP_Bootloader_config.h"

#if AP_BOOTLOADER_NETWORK_ENABLED

#include <hal.h>

class SocketAPM;

class BL_Network {
public:
    void init(void);
    void save_comms_ip(void);

private:
    struct netif *thisif;
    thread_t *net_thread_ctx;

    HAL_Semaphore web_delete_mtx;
    thread_t *web_delete_list;

    static void net_thread_trampoline(void*);
    static void web_server_trampoline(void*);

    void net_thread(void);
    void web_server(void);
    static void net_request_trampoline(void *);
    void handle_request(SocketAPM *);
    void handle_post(SocketAPM *, uint32_t content_length);
    char *read_headers(SocketAPM *);

    static void link_up_cb(void *p);
    static void link_down_cb(void *p);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);
    static bool low_level_input(struct netif *netif, struct pbuf **pbuf);
    static int8_t ethernetif_init(struct netif *netif);

    static char *substitute_vars(const char *msg, uint32_t size);
    static struct web_var {
        const char *name;
        const char *value;
    } variables[];

    struct {
        uint32_t ip, gateway, netmask;
    } addr;

    bool need_reboot;
    bool need_launch;
    HAL_Semaphore status_mtx;
    char bl_status[256];

    void status_printf(const char *fmt, ...);
};

#endif // AP_BOOTLOADER_NETWORK_ENABLED


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
    void restore_comms_ip(void);
    void status_printf(const char *fmt, ...);

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
    void wait_sender_blocked(SocketAPM *, uint32_t remaining);
    char *read_headers(SocketAPM *);

    static void link_up_cb(void *p);
    static void link_down_cb(void *p);
#if AP_BOOTLOADER_NETWORK_PPP_ENABLED
    struct ppp_pcb_s *ppp;
    volatile bool ppp_need_restart;
    void ppp_start_serial(void);
    static uint32_t ppp_output_cb(struct ppp_pcb_s *pcb, const void *data, uint32_t len, void *ctx);
    static void ppp_status_cb(struct ppp_pcb_s *pcb, int code, void *ctx);
#else
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);
    static bool low_level_input(struct netif *netif, struct pbuf **pbuf);
    static int8_t ethernetif_init(struct netif *netif);
#endif  // AP_BOOTLOADER_NETWORK_PPP_ENABLED

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

};

#endif // AP_BOOTLOADER_NETWORK_ENABLED


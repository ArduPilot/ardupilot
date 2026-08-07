/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "hwdef.h"
#endif

#ifdef   __cplusplus
extern "C"
{
#endif

/* macOS specific */
#if defined(__APPLE__)
/* lwip/contrib/ports/unix/port/netif/sio.c */
#include <util.h>

/* lwip/src/apps/http/makefsdata/makefsdata.c  */
#define GETCWD(path, len)             getcwd(path, len)
#define GETCWD_SUCCEEDED(ret)         (ret != NULL)
#define CHDIR(path)                   chdir(path)
#define CHDIR_SUCCEEDED(ret)          (ret == 0)

/* lwip/src/netif/ppp/fsm.c */
/* lwip/src/netif/ppp/pppos.c */
/* lwip/src/netif/ppp/vj.c */
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
#endif

#ifdef LWIP_OPTTEST_FILE
#include "lwipopts_test.h"
#else /* LWIP_OPTTEST_FILE */

#define LWIP_IPV4                  1
#define LWIP_IPV6                  0

#define MEM_LIBC_MALLOC 1
#define MEMP_MEM_MALLOC 1
#define LWIP_NETCONN_SEM_PER_THREAD 0

/**
 * SO_REUSE==1: Enable SO_REUSEADDR option.
 */
#ifndef SO_REUSE
#define SO_REUSE                   1
#endif

#define NO_SYS                     0
#define LWIP_SOCKET                (NO_SYS==0)
#define LWIP_NETCONN               (NO_SYS==0)
#define LWIP_NETIF_API             (NO_SYS==0)

#ifndef LWIP_IGMP
#define LWIP_IGMP                  LWIP_IPV4
#endif
#define LWIP_ICMP                  LWIP_IPV4

#define LWIP_SNMP                  LWIP_UDP
#ifdef LWIP_HAVE_MBEDTLS
#define LWIP_SNMP_V3               (LWIP_SNMP)
#endif

#define LWIP_DNS                   0
#define LWIP_MDNS_RESPONDER        0

#define LWIP_NUM_NETIF_CLIENT_DATA (LWIP_MDNS_RESPONDER)

#ifndef LWIP_HAVE_LOOPIF
#define LWIP_HAVE_LOOPIF           1
#endif
#ifndef LWIP_NETIF_LOOPBACK
#define LWIP_NETIF_LOOPBACK        1
#endif
#define LWIP_LOOPBACK_MAX_PBUFS    10

#define TCP_LISTEN_BACKLOG         1

#define LWIP_COMPAT_SOCKETS        0
#define LWIP_SO_RCVTIMEO           1
#define LWIP_SO_RCVBUF             1

#define LWIP_TCPIP_CORE_LOCKING    1

#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_EXT_STATUS_CALLBACK  1

#define USE_PPP 1

// generate a regular LCP echo to keep link up
#define LCP_ECHOINTERVAL                1

// on 5 failed echos terminate link
#define LCP_MAXECHOFAILS                5

#define LWIP_TIMEVAL_PRIVATE 0
#define LWIP_FD_SET_PRIVATE 0

#define TCP_WND 12000
#define TCP_SND_BUF 12000
#define DEFAULT_ACCEPTMBOX_SIZE         20

    
// #define LWIP_DEBUG
#ifdef LWIP_DEBUG
#define LWIP_DBG_MIN_LEVEL         0
#define PPP_DEBUG                  LWIP_DBG_ON
#define MEM_DEBUG                  LWIP_DBG_ON
#define MEMP_DEBUG                 LWIP_DBG_ON
#define PBUF_DEBUG                 LWIP_DBG_OFF
#define API_LIB_DEBUG              LWIP_DBG_OFF
#define API_MSG_DEBUG              LWIP_DBG_OFF
#define TCPIP_DEBUG                LWIP_DBG_ON
#define NETIF_DEBUG                LWIP_DBG_ON
#define SOCKETS_DEBUG              LWIP_DBG_OFF
#define DNS_DEBUG                  LWIP_DBG_OFF
#define AUTOIP_DEBUG               LWIP_DBG_OFF
#define DHCP_DEBUG                 LWIP_DBG_OFF
#define IP_DEBUG                   LWIP_DBG_OFF
#define IP_REASS_DEBUG             LWIP_DBG_OFF
#define ICMP_DEBUG                 LWIP_DBG_OFF
#define IGMP_DEBUG                 LWIP_DBG_OFF
#define UDP_DEBUG                  LWIP_DBG_OFF
#define TCP_DEBUG                  LWIP_DBG_ON
#define TCP_INPUT_DEBUG            LWIP_DBG_ON
#define TCP_OUTPUT_DEBUG           LWIP_DBG_ON
#define TCP_RTO_DEBUG              LWIP_DBG_OFF
#define TCP_CWND_DEBUG             LWIP_DBG_OFF
#define TCP_WND_DEBUG              LWIP_DBG_OFF
#define TCP_FR_DEBUG               LWIP_DBG_OFF
#define TCP_QLEN_DEBUG             LWIP_DBG_OFF
#define TCP_RST_DEBUG              LWIP_DBG_OFF
#endif

#define LWIP_DBG_TYPES_ON         (LWIP_DBG_ON|LWIP_DBG_TRACE|LWIP_DBG_STATE|LWIP_DBG_FRESH|LWIP_DBG_HALT)


/* ---------- Memory options ---------- */
/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
   lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
   byte alignment -> define MEM_ALIGNMENT to 2. */
/* MSVC port: intel processors don't need 4-byte alignment,
   but are faster that way! */
#define MEM_ALIGNMENT           4U

/* MEM_SIZE: the size of the heap memory. If the application will send
a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE               10240

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
   sends a lot of data out of ROM (or other static memory), this
   should be set high. */
#define MEMP_NUM_PBUF           64
/* MEMP_NUM_RAW_PCB: the number of UDP protocol control blocks. One
   per active RAW "connection". */
#define MEMP_NUM_RAW_PCB        3
/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
   per active UDP "connection". */
#define MEMP_NUM_UDP_PCB        8
/* MEMP_NUM_TCP_PCB: the number of simultaneously active TCP
   connections. */
#define MEMP_NUM_TCP_PCB        5
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP
   connections. */
#define MEMP_NUM_TCP_PCB_LISTEN 8
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP
   segments. */
#define MEMP_NUM_TCP_SEG        16
/* MEMP_NUM_SYS_TIMEOUT: the number of simultaneously active
   timeouts. */
#define MEMP_NUM_SYS_TIMEOUT    17

/* The following four are used only with the sequential API and can be
   set to 0 if the application only will use the raw API. */
/* MEMP_NUM_NETBUF: the number of struct netbufs. */
#define MEMP_NUM_NETBUF         100
/* MEMP_NUM_NETCONN: the number of struct netconns. */
#define MEMP_NUM_NETCONN        64
/* MEMP_NUM_TCPIP_MSG_*: the number of struct tcpip_msg, which is used
   for sequential API communication and incoming packets. Used in
   src/api/tcpip.c. */
#define MEMP_NUM_TCPIP_MSG_API   16
#define MEMP_NUM_TCPIP_MSG_INPKT 16


/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */
#define PBUF_POOL_SIZE          120

/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */
#define PBUF_POOL_BUFSIZE       256

/** SYS_LIGHTWEIGHT_PROT
 * define SYS_LIGHTWEIGHT_PROT in lwipopts.h if you want inter-task protection
 * for certain critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT    (NO_SYS==0)


/* ---------- TCP options ---------- */
#define LWIP_TCP                1
#define TCP_TTL                 255

#ifndef LWIP_ALTCP
#define LWIP_ALTCP              (LWIP_TCP)
#endif
#ifdef LWIP_HAVE_MBEDTLS
#define LWIP_ALTCP_TLS          (LWIP_TCP)
#define LWIP_ALTCP_TLS_MBEDTLS  (LWIP_TCP)
#endif


/* Controls if TCP should queue segments that arrive out of
   order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ         1

/* TCP Maximum segment size. */
#define TCP_MSS                 1024

/* TCP sender buffer space (bytes). */
#ifndef TCP_SND_BUF
#define TCP_SND_BUF             2048
#endif

/* TCP sender buffer space (pbufs). This must be at least = 2 *
   TCP_SND_BUF/TCP_MSS for things to work. */
#define TCP_SND_QUEUELEN       (4 * TCP_SND_BUF/TCP_MSS)

/* TCP writable space (bytes). This must be less than or equal
   to TCP_SND_BUF. It is the amount of space which must be
   available in the tcp snd_buf for select to return writable */
#define TCP_SNDLOWAT           (TCP_SND_BUF/2)

/* TCP receive window. */
#ifndef TCP_WND
#define TCP_WND                 (20 * 1024)
#endif

/* Maximum number of retransmissions of data segments. */
#define TCP_MAXRTX              12

/* Maximum number of retransmissions of SYN segments. */
#define TCP_SYNMAXRTX           4


/* ---------- ARP options ---------- */
#define LWIP_ARP                1
#define ARP_TABLE_SIZE          10
#define ARP_QUEUEING            1
#ifndef ARP_PROXYARP_SUPPORT
#define ARP_PROXYARP_SUPPORT    1
#endif


/* ---------- IP options ---------- */
/* Define IP_FORWARD to 1 if you wish to have the ability to forward
   IP packets across network interfaces. If you are going to run lwIP
   on a device with only one network interface, define this to 0. */
#ifndef IP_FORWARD
#define IP_FORWARD              1
#endif

/*
  extra header space when forwarding for adding the ethernet header
*/
#define PBUF_LINK_HLEN          16


/* IP reassembly and segmentation.These are orthogonal even
 * if they both deal with IP fragments */
#define IP_REASSEMBLY           1
#define IP_REASS_MAX_PBUFS      (10 * ((1500 + PBUF_POOL_BUFSIZE - 1) / PBUF_POOL_BUFSIZE))
#define MEMP_NUM_REASSDATA      IP_REASS_MAX_PBUFS
#define IP_FRAG                 1
#define IPV6_FRAG_COPYHEADER    1

/* ---------- ICMP options ---------- */
#define ICMP_TTL                255


/* ---------- DHCP options ---------- */
/* Define LWIP_DHCP to 1 if you want DHCP configuration of
   interfaces. */
#ifndef LWIP_DHCP
#define LWIP_DHCP               1
#endif

/* 1 if you want to do an ARP check on the offered address
   (recommended). */
#define DHCP_DOES_ARP_CHECK    (LWIP_DHCP)


/* ---------- AUTOIP options ------- */
#define LWIP_AUTOIP            (LWIP_DHCP)
#define LWIP_DHCP_AUTOIP_COOP  (LWIP_DHCP && LWIP_AUTOIP)


/* ---------- UDP options ---------- */
#ifndef LWIP_UDP
#define LWIP_UDP                1
#endif
#define LWIP_UDPLITE            LWIP_UDP
#define UDP_TTL                 255


/* ---------- RAW options ---------- */
#define LWIP_RAW                0


/* ---------- Statistics options ---------- */

#define LWIP_STATS              0
#define LWIP_STATS_DISPLAY      0

/* ---------- NETBIOS options ---------- */
#define LWIP_NETBIOS_RESPOND_NAME_QUERY 0

/* routing hook */
#define LWIP_HOOK_IP4_ROUTE ap_networking_routing_hook
struct ip4_addr;
struct netif *ap_networking_routing_hook(const struct ip4_addr *dest);

/* ---------- PPP options ---------- */

#ifndef PPP_SUPPORT
#define PPP_SUPPORT             1      /* Set > 0 for PPP */
#endif

#if PPP_SUPPORT

// support PPP network capture
struct pbuf;
struct ppp_pcb_s;
void ap_ppp_capture_hook(const struct ppp_pcb_s *pcb, const struct pbuf *pb);
#define LWIP_PPP_CAPTURE_HOOK ap_ppp_capture_hook

#define NUM_PPP                 1      /* Max PPP sessions. */


/* Select modules to enable.  Ideally these would be set in the makefile but
 * we're limited by the command line length so you need to modify the settings
 * in this file.
 */
#define PPPOE_SUPPORT           0
#define PPPOS_SUPPORT           PPP_SUPPORT

#define PAP_SUPPORT             0      /* Set > 0 for PAP. */
#define CHAP_SUPPORT            0      /* Set > 0 for CHAP. */
#define MSCHAP_SUPPORT          0      /* Set > 0 for MSCHAP */
#define CBCP_SUPPORT            0      /* Set > 0 for CBCP (NOT FUNCTIONAL!) */
#define CCP_SUPPORT             0      /* Set > 0 for CCP */
/*
  VJ support disabled due to bugs with IP forwarding
 */
#define VJ_SUPPORT              0      /* Set > 0 for VJ header compression. */
#define MD5_SUPPORT             0      /* Set > 0 for MD5 (see also CHAP) */

#endif /* PPP_SUPPORT */

#endif /* LWIP_OPTTEST_FILE */

/* The following defines must be done even in OPTTEST mode: */

#if !defined(NO_SYS) || !NO_SYS /* default is 0 */
void sys_check_core_locking(void);
#define LWIP_ASSERT_CORE_LOCKED()  sys_check_core_locking()
#endif

#ifndef LWIP_PLATFORM_ASSERT
/* Define LWIP_PLATFORM_ASSERT to something to catch missing stdio.h includes */
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || defined(HAL_DEBUG_BUILD)
void ap_networking_platform_assert(const char *msg, int line);
#define LWIP_PLATFORM_ASSERT(x) ap_networking_platform_assert(x, __LINE__)
#else
void ap_networking_platform_assert(int line);
#define LWIP_PLATFORM_ASSERT(x) ap_networking_platform_assert(__LINE__)
#endif
#endif

/*
  map LWIP debugging onto ap_networking_printf to allow for easier
  redirection to a file or dedicated serial port
 */
#ifdef __cplusplus
extern "C" {
#endif
int ap_networking_printf(const char *fmt, ...);
#ifdef __cplusplus
}
#endif
#define LWIP_PLATFORM_DIAG(x)    do {ap_networking_printf x; } while(0)

// #define AP_NETWORKING_LWIP_DEBUG_FILE "lwip.log"

#ifdef   __cplusplus
}
#endif
    

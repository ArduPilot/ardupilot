#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdlib.h>
#include <stdio.h>
#include <arpa/inet.h>

/*
  address of the local interface SITL should send and receive its
  multicast traffic on, in network byte order, or zero to leave the
  choice to the routing table.

  Multicasting on whichever interface the routing table picks is
  normally what you want; it is how an AP_Periph, or a second machine on
  the network, gets at the simulation.  It does mean the traffic follows
  the default route, so it stops working when that route is not up, or
  not multicast-capable.  Set SITL_MULTICAST_IF_ADDR (e.g. to 127.0.0.1)
  to pin the traffic to one interface where a run has to be insulated
  from the state of the machine's network - the autotest suite does
  this, so that a interface coming or going does not decide whether a
  test passes.
 */
static inline uint32_t sitl_multicast_interface_address(void)
{
    const char *addr = getenv("SITL_MULTICAST_IF_ADDR");
    if (addr == nullptr || *addr == 0) {
        // unset, or set empty to get the default back in an environment
        // which sets it (the autotest suite does)
        return 0;
    }
    const uint32_t ret = inet_addr(addr);
    if (ret == INADDR_NONE) {
        ::fprintf(stderr, "Bad SITL_MULTICAST_IF_ADDR (%s); using any interface\n", addr);
        return 0;
    }
    return ret;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL

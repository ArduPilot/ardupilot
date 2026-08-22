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

/*
  return the port for the simulation-state multicast, taking any
  override from the environment.  The compiled-in default is a fixed
  constant with no per-instance offset - unlike every other SITL port -
  so simulations run concurrently on one machine share a state bus and
  each peripheral answers a vehicle which is not its own.  A test
  framework running vehicles in parallel exports a per-instance
  SITL_MCAST_STATE_PORT to the vehicle and its peripherals (environment
  inheritance reaches both) to give each simulation its own bus.
 */
static inline uint16_t sitl_multicast_state_port(uint16_t default_port)
{
    const char *port_str = getenv("SITL_MCAST_STATE_PORT");
    if (port_str == nullptr || *port_str == 0) {
        return default_port;
    }
    const long port = strtol(port_str, nullptr, 10);
    if (port <= 0 || port > 65535) {
        ::fprintf(stderr, "Bad SITL_MCAST_STATE_PORT (%s); using default\n", port_str);
        return default_port;
    }
    return (uint16_t)port;
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL

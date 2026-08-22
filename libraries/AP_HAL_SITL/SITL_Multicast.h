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
  return a port taken from the named environment variable, or the
  supplied default if it is unset, empty or invalid
 */
static inline uint16_t sitl_port_from_env(const char *name, uint16_t default_port)
{
    const char *port_str = getenv(name);
    if (port_str == nullptr || *port_str == 0) {
        return default_port;
    }
    const long port = strtol(port_str, nullptr, 10);
    if (port <= 0 || port > 65535) {
        ::fprintf(stderr, "Bad %s (%s); using default\n", name, port_str);
        return default_port;
    }
    return (uint16_t)port;
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
    return sitl_port_from_env("SITL_MCAST_STATE_PORT", default_port);
}

/*
  return the port for the simulated CAN bus multicast, taking any
  override from the environment.  The multicast group already varies
  with the CAN bus number, keeping one simulation's buses apart from
  each other, but the port is a fixed constant, so concurrent
  simulations on one machine share their CAN buses: every peripheral
  hears (and answers) every vehicle.  A per-instance
  SITL_CAN_MCAST_PORT, exported to the vehicle and its peripherals,
  gives each simulation a private set of buses.
 */
static inline uint16_t sitl_can_multicast_port(uint16_t default_port)
{
    return sitl_port_from_env("SITL_CAN_MCAST_PORT", default_port);
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL

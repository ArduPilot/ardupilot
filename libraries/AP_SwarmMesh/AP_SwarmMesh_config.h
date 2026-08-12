#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SWARMMESH_ENABLED
#define AP_SWARMMESH_ENABLED HAL_PROGRAM_SIZE_LIMIT_KB > 2048
#endif

#ifndef AP_SWARMMESH_TIMEOUT_MS
#define AP_SWARMMESH_TIMEOUT_MS 300
#endif

#ifndef AP_SWARMMESH_SERIAL_ENABLED
#define AP_SWARMMESH_SERIAL_ENABLED AP_SWARMMESH_ENABLED
#endif

#ifndef AP_SWARMMESH_SITL_ENABLED
#define AP_SWARMMESH_SITL_ENABLED (AP_SWARMMESH_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef AP_SWARMMESH_POSCONTROL_ENABLED
#define AP_SWARMMESH_POSCONTROL_ENABLED AP_SWARMMESH_ENABLED
#endif

// peer_state[AP_SWARMMESH_MAX_PEERS], sizeof(PeerState) is ~168 bytes currently. Boards can override this directly in hwdef
#ifndef AP_SWARMMESH_MAX_PEERS
#if HAL_MEM_CLASS >= HAL_MEM_CLASS_1000
#define AP_SWARMMESH_MAX_PEERS 255  // full sysid range (0 is reserved for broadcast): H7, SITL, Linux, QURT
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_500
#define AP_SWARMMESH_MAX_PEERS 128  // ~17KB
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_300
#define AP_SWARMMESH_MAX_PEERS 64   // ~8.5KB
#elif HAL_MEM_CLASS >= HAL_MEM_CLASS_192
#define AP_SWARMMESH_MAX_PEERS 18   // ~2.5KB
#else
#define AP_SWARMMESH_MAX_PEERS 8    // ~1KB fallback
#endif
#endif

// optional filter peer filter (mandatory on small boards)
// (which fill their small peer table on a first come first serve basis otherwise).
#ifndef AP_SWARMMESH_MAX_PEER_FILTERS
#define AP_SWARMMESH_MAX_PEER_FILTERS 16
#endif

// Max RX bytes drained through the parser per update tick. Must exceed the max arrival rate (~N_peers x stream-rate x ~90B) divided by the update rate,
// or the transport's buffers overflow and the peer table starves. SITL gets a large budget to cope with 254 node swarms.
#ifndef AP_SWARMMESH_RX_BUDGET_BYTES
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define AP_SWARMMESH_RX_BUDGET_BYTES 16384
#else
#define AP_SWARMMESH_RX_BUDGET_BYTES 1024
#endif
#endif

// TX stream rates (Hz) for each hardware profile.
#ifndef AP_SWARMMESH_FULL_HZ
#define AP_SWARMMESH_FULL_HZ 200
#endif

#ifndef AP_SWARMMESH_LITE_HZ
#define AP_SWARMMESH_LITE_HZ 10
#endif
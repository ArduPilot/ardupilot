#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>
#include <pthread.h>

/*
  shared memory segment allowing multiple SITL instances to share
  simulated clock state. Each instance writes its own sim_time_us into
  a slot indexed by instance number; readers check peer times to
  determine relative clock positions.

  Clustering is strictly opt-in: nothing here runs unless --cluster was
  given on the command line. An instance number alone says nothing about
  whether the user wants a swarm, so "--instance 1" on its own never
  touches shared memory.

  Membership is discovered, never declared. There is no fleet size to
  state up front: an instance barriers against whichever peers have
  actually registered in the segment, so a peer that is starting late
  simply joins when it appears, and one that never starts costs nothing.

  The segment name is built at runtime (see _build_name()) from the uid
  and the cluster id, so separate swarms - and different users on a
  shared host - never land in the same object.

  The clock field is a single-writer 64-bit value, accessed atomically
  where the target supports lock-free 64-bit atomics so it cannot tear.
  The payload[] block is control-critical (e.g. shared vehicle telemetry)
  so it's protected by a process-shared, robust pthread mutex instead -
  see write_payload()/read_payload().
*/

#define AP_SITL_SHMEM_MAX_INSTANCES 16
#define AP_SITL_SHMEM_MAGIC         0x41505351  // "APSQ" (v5 layout: shared slave epoch)
#define AP_SITL_SHMEM_VERSION       3

// segment name prefix; the uid and cluster id are appended
#define AP_SITL_SHMEM_NAME_PREFIX   "/ardupilot_sitl_cluster"
#define AP_SITL_SHMEM_NAME_LEN      64

// how long init() will wait for a concurrently-starting instance to
// finish building the segment before deciding it is stale garbage
#define AP_SITL_SHMEM_INIT_TIMEOUT_US 2000000

// How often a peer's pid is probed with kill(pid, 0) while spinning in the
// barrier. The spin runs every ~100us, so probing every pass costs tens of
// thousands of syscalls per second per instance - and every instance is
// doing it about every other peer. Peers do not die often enough to need
// microsecond detection; 100ms is still effectively instant.
#define AP_SITL_SHMEM_LIVENESS_CHECK_US 100000

// size of the arbitrary per-instance data block, in addition to the clock.
// e.g. vehicle position/attitude or other user-defined telemetry.
#define AP_SITL_SHMEM_PAYLOAD_SIZE  4096

/*
  the payload block is partitioned between its tenants at fixed offsets:
    [0, AP_SITL_SHMEM_RIDE_OFFSET)    AP_SITL_SwarmInfo, published from
                                      Aircraft::sync_frame_time()
    [AP_SITL_SHMEM_RIDE_OFFSET, end)  ride-along JSON transport channel,
                                      used by SIM_JSON_Master (master) and
                                      SIM_JSON (slave) instead of the UDP
                                      sockets when the ride-along pair was
                                      started with --cluster
  Both tenants are guarded by the same per-slot robust mutex.
*/
#define AP_SITL_SHMEM_RIDE_OFFSET   512
#define AP_SITL_SHMEM_RIDE_SIZE     (AP_SITL_SHMEM_PAYLOAD_SIZE - AP_SITL_SHMEM_RIDE_OFFSET)

/*
  one-directional message channel living in the ride-along region of a
  slot: the master publishes its fdm JSON line in its own slot, each
  slave publishes its servo packet in its own slot. A publish rewrites
  the whole record under the slot mutex and bumps seq last, so a single
  read_payload() of the channel always yields an internally-consistent
  record; readers poll seq for change.
*/
struct AP_SITL_RideAlongChannel {
    uint32_t magic;     // which endpoint writes this slot's channel (below)
    uint32_t seq;       // bumped once per publish
    uint32_t len;       // valid bytes in data[]
    uint8_t  data[AP_SITL_SHMEM_RIDE_SIZE - 12];
};
static_assert(sizeof(AP_SITL_RideAlongChannel) == AP_SITL_SHMEM_RIDE_SIZE,
              "ride-along channel must exactly fill its payload region");
#define AP_SITL_RIDE_FDM_MAGIC    0x52464D31  // "RFM1": master -> slaves fdm JSON
#define AP_SITL_RIDE_SERVO_MAGIC  0x52535631  // "RSV1": slave -> master servo packet

/*
  Robust process-shared mutexes make a peer that is killed mid-write hand
  the next locker EOWNERDEAD instead of deadlocking every other instance
  forever.

  Do NOT write "#ifdef PTHREAD_MUTEX_ROBUST" here: glibc declares it as a
  member of an anonymous enum in pthread.h, not as a macro, so the #ifdef
  is always false and silently compiles the whole recovery path out while
  the symbol still works perfectly as a value. Test the platform instead.
*/
#if defined(__linux__)
#define AP_SITL_SHMEM_ROBUST_MUTEX 1
#else
#define AP_SITL_SHMEM_ROBUST_MUTEX 0
#endif

/*
  Use real atomics for the cross-process 64-bit clock only where they are
  lock-free. On targets where a 64-bit atomic degrades to a libatomic call
  (e.g. armv6) fall back to a plain access rather than adding a link-time
  dependency; such builds are 32-bit hosts running a single instance.
*/
#if defined(__GCC_ATOMIC_LLONG_LOCK_FREE) && __GCC_ATOMIC_LLONG_LOCK_FREE == 2
#define AP_SITL_SHMEM_ATOMIC64 1
#else
#define AP_SITL_SHMEM_ATOMIC64 0
#endif

/*
  layout of the shared memory segment
*/
struct AP_SITL_ShmData {
    uint32_t magic;         // written last by the creator; see _init_created_segment()
    uint32_t version;
    // the cluster-wide wall-slaved schedule epoch: written once (CAS)
    // by the first member to observe the whole cluster armed; every
    // member slaves its simulated time to this single shared timeline
    // so pinned schedules cannot drift apart between members
    uint64_t slave_epoch_wall_us;
    uint64_t slave_epoch_sim_us;
    struct {
        uint64_t sim_time_us;   // simulated time in microseconds
        pid_t    pid;           // pid of owning process (0 = never used,
                                // -1 = peer registered but has since died)
        uint32_t armed;         // nonzero once this member's vehicle is
                                // armed; the wall-slaved schedule engages
                                // cluster-wide only when every active
                                // member is armed
        // process-shared robust mutex guarding payload[]. Robust means
        // a locker that dies mid-hold gives the next locker EOWNERDEAD
        // instead of hanging - see write_payload()/read_payload().
        pthread_mutex_t payload_mutex;
        uint8_t  payload[AP_SITL_SHMEM_PAYLOAD_SIZE];
    } instance[AP_SITL_SHMEM_MAX_INSTANCES];
};

class AP_SITL_SharedMem {
public:
    AP_SITL_SharedMem();
    ~AP_SITL_SharedMem();

    /*
      join cluster cluster_id as the given instance number. Called only
      when --cluster was passed; until then this object stays inert and
      every method below is a no-op.
    */
    bool init(uint8_t instance_id, uint8_t cluster_id);

    /*
      update this instance's clock in the shared segment.
      Call this periodically (e.g. every scheduler tick).
    */
    void update(uint64_t time_us);

    /*
      return the last published sim_time_us for the given instance.
      Returns 0 if the instance has no active entry or shm is not initialised.
    */
    uint64_t get_time_us(uint8_t instance_id) const;

    /*
      return true if the given instance slot appears active
      (pid is alive and time is non-zero)
    */
    bool instance_active(uint8_t instance_id) const;

    /*
      return the number of instances currently in the cluster, ourselves
      included. 0 when clustering is not enabled.
    */
    uint8_t get_instance_count() const;

    /*
      write up to AP_SITL_SHMEM_PAYLOAD_SIZE bytes into this instance's
      payload block, for peers to read via read_payload(). Takes the
      slot's robust process-shared mutex for the duration of the copy so
      readers never observe a torn write. offset positions the write
      within the block, for tenants above the first (see the payload
      partition map by AP_SITL_SHMEM_RIDE_OFFSET).
    */
    void write_payload(const void *data, uint32_t len, uint32_t offset = 0);

    /*
      read the given instance's payload block into data (up to len bytes,
      capped at AP_SITL_SHMEM_PAYLOAD_SIZE, starting offset bytes into
      the block). Takes the slot's robust process-shared mutex for the
      duration of the copy. Returns true on success, false if shm is not
      initialised or instance_id is out of range.
    */
    bool read_payload(uint8_t instance_id, void *data, uint32_t len, uint32_t offset = 0) const;

    /*
      barrier sync: spin-wait until every peer that has joined the cluster
      has published a sim_time_us >= (sim_time_us - max_skew_us). Returns
      false on timeout_us (wall-clock us) expiry, e.g. a peer crashed.

      Slots nobody has claimed are simply not peers, so this returns
      immediately when we are the only instance in the cluster.

      A peer whose sim_time_us stalls (e.g. mid-reboot) but is still
      alive is excluded from the barrier after stall_grace_us, so it
      doesn't block the rest of the swarm for the full timeout_us.
    */
    bool sync_with_peers(uint64_t sim_time_us,
                         uint64_t max_skew_us = 5000,
                         uint64_t timeout_us = 5000000,
                         uint64_t stall_grace_us = 300000);

    /*
      return true if our own sim_time_us is more than max_skew_us behind
      the most-advanced live peer - used to trigger a ~2x-speedup
      catch-up sprint (see Aircraft::sync_frame_time()) so a rebooted
      instance fast-forwards back into lock-step instead of remaining
      permanently excluded from the barrier.

      The sprint is capped at max_catchup_us of wall-clock time; once
      exceeded we give up and return false, running at normal speed
      while sync_with_peers() still excludes us from the barrier.
    */
    bool is_behind_peers(uint64_t sim_time_us, uint64_t max_skew_us = 5000,
                          uint64_t max_catchup_us = 60000000);

    /*
      one-shot: if this process started less than max_uptime_us ago and
      is behind live peers, snap sim_time_us to match them instantly
      (no sprint) and return true. Meant for an instance joining an
      already-running cluster - or a just-rebooted one - to reach
      lock-step immediately rather than sprinting to catch up.
    */
    bool instant_catchup_if_new(uint64_t &sim_time_us, uint64_t max_uptime_us = 10000000);

    bool is_initialised() const { return _data != nullptr; }

    // publish this member's armed state (called with the sim time update)
    void set_armed(bool armed);

    // true when every ACTIVE cluster member reports armed. False when
    // uninitialised.
    bool all_members_armed() const;

    // fetch the cluster-wide slave-schedule epoch, publishing the
    // caller's proposal if none exists yet (first-writer wins).
    // Returns false when uninitialised.
    bool get_or_set_slave_epoch(uint64_t &wall_us, uint64_t &sim_us);

    // read the epoch without ever publishing: true once one exists.
    bool peek_slave_epoch(uint64_t &wall_us, uint64_t &sim_us) const;
    bool is_multi_instance() const { return get_instance_count() > 1; }

private:
    AP_SITL_ShmData *_data;
    int              _fd;
    uint8_t          _instance_id;
    uint8_t          _cluster_id;
    bool             _created;  // true if we created the shm segment
    bool             _sync_announced;     // printed "in lock-step" once

    char             _name[AP_SITL_SHMEM_NAME_LEN];  // resolved segment name

    // per-peer stall tracking for sync_with_peers(): last sim_time_us
    // observed per peer, when it last changed, and stall/resume state.
    uint64_t         _peer_seen_time_us[AP_SITL_SHMEM_MAX_INSTANCES];
    uint64_t         _peer_seen_wall_us[AP_SITL_SHMEM_MAX_INSTANCES];
    bool             _peer_stalled[AP_SITL_SHMEM_MAX_INSTANCES];
    bool             _peer_resume_announced[AP_SITL_SHMEM_MAX_INSTANCES];  // printed "resumed, still catching up" once per stall
    bool             _peer_joined_announced[AP_SITL_SHMEM_MAX_INSTANCES];  // printed "joined the cluster" once

    // wall-clock time each peer's pid was last probed for liveness, so the
    // barrier spin does not issue a kill() per peer per pass
    uint64_t         _peer_checked_wall_us[AP_SITL_SHMEM_MAX_INSTANCES];

    // wall-clock time before which a just-rejoined peer may not be
    // re-marked stalled; prevents flapping right at the min_peer_time
    // boundary under CPU contention from sprinting instances.
    uint64_t         _peer_cooldown_until_us[AP_SITL_SHMEM_MAX_INSTANCES];

    // is_behind_peers() catch-up-sprint tracking: the wall-clock time we
    // first noticed we were behind, and whether we've since given up
    // sprinting (having hit max_catchup_us) until we catch up naturally.
    uint64_t         _catchup_started_wall_us;
    bool             _catchup_gave_up;

    // instant_catchup_if_new(): wall-clock time this process started, and
    // whether we've already taken (or forfeited) our one-shot clock jump.
    uint64_t         _process_start_wall_us;
    bool             _instant_catchup_done;

#ifdef __CYGWIN__
    // Win32 file-mapping handle backing the segment. Stored as void* so
    // this header never needs <windows.h>; only SITL_SharedMem.cpp casts
    // it to HANDLE.
    void            *_win_map_handle;
#endif

    static uint64_t _now_us();
    uint64_t _max_peer_time() const;

    // cross-process access to a slot's 64-bit clock
    static uint64_t _load_time(const uint64_t &v);
    static void     _store_time(uint64_t &v, uint64_t t);

    void _build_name();
    bool _open_segment();
    bool _init_created_segment();
    bool _init_segment_contents();
    bool _map_when_ready(uint64_t deadline_us);
    bool _wait_for_magic(uint64_t deadline_us);
    bool _map();
    void _unmap();

    void _clear_slot();
    void _cleanup();
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

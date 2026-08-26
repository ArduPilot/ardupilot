#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SITL_SharedMem.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#ifdef __CYGWIN__
// Win32 API for the native shared-memory path - see _open_segment().
// Included after everything else, and lean, so its macro spill cannot
// reach the ArduPilot headers above.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

AP_SITL_SharedMem::AP_SITL_SharedMem() :
    _data(nullptr),
    _fd(-1),
    _instance_id(0),
    _cluster_id(0),
    _created(false),
    _sync_announced(false)
{
    _name[0] = 0;

    memset(_peer_seen_time_us, 0, sizeof(_peer_seen_time_us));
    memset(_peer_seen_wall_us, 0, sizeof(_peer_seen_wall_us));
    memset(_peer_stalled, 0, sizeof(_peer_stalled));
    memset(_peer_resume_announced, 0, sizeof(_peer_resume_announced));
    memset(_peer_joined_announced, 0, sizeof(_peer_joined_announced));
    memset(_peer_checked_wall_us, 0, sizeof(_peer_checked_wall_us));
    memset(_peer_cooldown_until_us, 0, sizeof(_peer_cooldown_until_us));
    _catchup_started_wall_us = 0;
    _catchup_gave_up = false;

    _process_start_wall_us = _now_us();
    _instant_catchup_done = false;

#ifdef __CYGWIN__
    _win_map_handle = nullptr;
#endif
}

AP_SITL_SharedMem::~AP_SITL_SharedMem()
{
    _cleanup();
}

uint64_t AP_SITL_SharedMem::_now_us()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

/*
  the clock word is written by one process and read by all the others, so
  a plain 64-bit access can be seen half-updated on a 32-bit build.
*/
uint64_t AP_SITL_SharedMem::_load_time(const uint64_t &v)
{
#if AP_SITL_SHMEM_ATOMIC64
    return __atomic_load_n(&v, __ATOMIC_RELAXED);
#else
    return v;
#endif
}

void AP_SITL_SharedMem::_store_time(uint64_t &v, uint64_t t)
{
#if AP_SITL_SHMEM_ATOMIC64
    __atomic_store_n(&v, t, __ATOMIC_RELAXED);
#else
    v = t;
#endif
}

// highest sim_time_us reported by any live peer in the cluster
uint64_t AP_SITL_SharedMem::_max_peer_time() const
{
    uint64_t max_peer_time = 0;
    for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
        if (i == _instance_id) {
            continue;
        }
        const pid_t pid = _data->instance[i].pid;
        if (pid <= 0 || kill(pid, 0) != 0) {
            continue;
        }
        const uint64_t peer_time = _load_time(_data->instance[i].sim_time_us);
        if (peer_time > max_peer_time) {
            max_peer_time = peer_time;
        }
    }
    return max_peer_time;
}

/*
  build the segment name.

  A single fixed name would mean every SITL run by every user on the host
  shared one object: unrelated sessions claiming each other's slots, and a
  segment left behind by a killed swarm confusing the next run. The uid
  separates users; the cluster id separates concurrent swarms.
*/
void AP_SITL_SharedMem::_build_name()
{
#ifdef __CYGWIN__
    // A Win32 kernel object name, not a filesystem path: no leading
    // slash. Unprefixed names live in the caller's session-local
    // namespace, and Windows sessions already separate users from each
    // other, so the uid the POSIX name carries would be redundant here -
    // and the harness on native Windows Python could not compute the
    // Cygwin uid anyway.
    snprintf(_name, sizeof(_name), "%s_%u",
             AP_SITL_SHMEM_NAME_PREFIX + 1, (unsigned)_cluster_id);
#else
    snprintf(_name, sizeof(_name), "%s_%lu_%u",
             AP_SITL_SHMEM_NAME_PREFIX, (unsigned long)getuid(),
             (unsigned)_cluster_id);
#endif
}

bool AP_SITL_SharedMem::_map()
{
    _data = static_cast<AP_SITL_ShmData *>(
        mmap(nullptr, sizeof(AP_SITL_ShmData),
             PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0));

    if (_data == MAP_FAILED) {
        perror("SITL_SharedMem: mmap");
        _data = nullptr;
        return false;
    }
    return true;
}

void AP_SITL_SharedMem::_unmap()
{
    if (_data != nullptr) {
#ifdef __CYGWIN__
        UnmapViewOfFile(_data);
#else
        munmap(_data, sizeof(AP_SITL_ShmData));
#endif
        _data = nullptr;
    }
}

/*
  we won the O_EXCL race, so build the segment.

  The magic is written last, after the size is set and the payload
  mutexes are constructed, because a peer that opens the name spins on
  the magic before touching anything else - see _map_when_ready().
*/
bool AP_SITL_SharedMem::_init_created_segment()
{
    if (ftruncate(_fd, sizeof(AP_SITL_ShmData)) < 0) {
        perror("SITL_SharedMem: ftruncate");
        return false;
    }
    if (!_map()) {
        return false;
    }
    return _init_segment_contents();
}

/*
  fill in a freshly created, zero-filled, mapped segment and publish it.
  Shared by the POSIX and Win32 creation paths - the layout protocol
  must have exactly one author.
*/
bool AP_SITL_SharedMem::_init_segment_contents()
{
    memset(_data, 0, sizeof(AP_SITL_ShmData));
    _data->version = AP_SITL_SHMEM_VERSION;

#ifdef __CYGWIN__
    /*
      No payload mutex on this platform, at all. Cygwin's pthread_mutex_t
      is a POINTER: pthread_mutex_init() here would allocate the real
      mutex in the CREATOR's private heap and store only that pointer in
      the shared segment - meaningless in every other process, whose
      locks then fail EINVAL on every single call (confirmed on Windows:
      one error line per physics frame). Cygwin has no process-shared
      mutexes to reach for either, so the payload block is simply
      unlocked here, announced once below. The clock protocol never
      needed a lock in the first place.
    */
    fprintf(stderr, "SITL_SharedMem: no process-shared mutexes on this "
                    "platform; payload access is unlocked between instances\n");
#else
    // initialise each slot's payload mutex as a process-shared, robust
    // mutex - memset() alone is not a valid pthread_mutex_t.
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    // Not every platform that compiles this actually supports sharing a
    // mutex between processes - Cygwin's pthread implementation rejects
    // PTHREAD_PROCESS_SHARED. Say so rather than silently handing out a
    // process-private mutex that guards nothing across the cluster.
    const int pret = pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    if (pret != 0) {
        fprintf(stderr, "SITL_SharedMem: process-shared mutexes unsupported "
                        "(%d); clock sync will work but payload access is "
                        "NOT safely locked between instances\n", pret);
    }
#if AP_SITL_SHMEM_ROBUST_MUTEX
    const int rret = pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    if (rret != 0) {
        fprintf(stderr, "SITL_SharedMem: robust mutexes unavailable (%d); an "
                        "instance killed mid-write will block its peers\n", rret);
    }
#else
    fprintf(stderr, "SITL_SharedMem: no robust-mutex support on this platform; "
                    "an instance killed mid-write will block its peers\n");
#endif
    for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
        pthread_mutex_init(&_data->instance[i].payload_mutex, &attr);
    }
    pthread_mutexattr_destroy(&attr);
#endif  // __CYGWIN__

    // publish: everything above must be visible before the magic is
    __atomic_store_n(&_data->magic, (uint32_t)AP_SITL_SHMEM_MAGIC, __ATOMIC_RELEASE);
    return true;
}

/*
  an object with our name already exists. Wait for its creator to finish
  building it, then map it.

  Creation is not atomic - shm_open() publishes the name before
  ftruncate() and the mutex setup have run - so an opener can legitimately
  find a zero-length, zero-magic object for a moment. Treating that as
  stale (and unlinking it) would pull the segment out from under the very
  instance still initialising it, so wait instead, and only give up when
  the deadline passes.

  Returns false if the segment never became usable, i.e. it really is a
  leftover and the caller should replace it.
*/
bool AP_SITL_SharedMem::_map_when_ready(uint64_t deadline_us)
{
    // wait for the creator's ftruncate(), so that mapping and touching
    // the pages cannot SIGBUS us on a short object
    while (true) {
        struct stat st;
        if (fstat(_fd, &st) < 0) {
            perror("SITL_SharedMem: fstat");
            return false;
        }
        if ((size_t)st.st_size >= sizeof(AP_SITL_ShmData)) {
            break;
        }
        if (_now_us() >= deadline_us) {
            fprintf(stderr, "SITL_SharedMem: %s still %lld bytes, need %zu\n",
                    _name, (long long)st.st_size, sizeof(AP_SITL_ShmData));
            return false;
        }
        usleep(1000);
    }

    if (!_map()) {
        return false;
    }
    return _wait_for_magic(deadline_us);
}

/*
  wait for the creator to publish the magic, which it writes last.
  Shared by the POSIX and Win32 attach paths.
*/
bool AP_SITL_SharedMem::_wait_for_magic(uint64_t deadline_us)
{
    while (true) {
        const uint32_t magic = __atomic_load_n(&_data->magic, __ATOMIC_ACQUIRE);
        if (magic == AP_SITL_SHMEM_MAGIC) {
            return true;
        }
        if (magic != 0) {
            // a different layout version, not a half-built segment
            fprintf(stderr, "SITL_SharedMem: bad magic 0x%08X in %s\n",
                    magic, _name);
            _unmap();
            return false;
        }
        if (_now_us() >= deadline_us) {
            fprintf(stderr, "SITL_SharedMem: %s never initialised\n", _name);
            _unmap();
            return false;
        }
        usleep(1000);
    }
}

#ifdef __CYGWIN__
/*
  Windows path: a named, pagefile-backed Win32 file mapping instead of
  POSIX shm.

  Cygwin's shm_open() needs a real /dev/shm directory with POSIX
  permissions under the Cygwin root - and with a standalone cygwin1.dll
  beside the binaries the root is wherever the DLL happens to live, so
  that directory reliably does not exist. The native kernel object needs
  no filesystem presence at all and gives the same zero-copy mapping
  semantics once mapped.

  Two things actually improve on the POSIX path here:

    - the creation race is resolved by the kernel: every instance calls
      CreateFileMapping, one caller wins, the rest see
      ERROR_ALREADY_EXISTS - no O_EXCL retry dance

    - the object is reference-counted and vanishes with its last handle,
      so a stale segment left by a killed swarm cannot exist and no
      unlink/recreate logic is needed
*/
bool AP_SITL_SharedMem::_open_segment()
{
    const uint64_t deadline_us = _now_us() + AP_SITL_SHMEM_INIT_TIMEOUT_US;

    // The creator/attacher decision below reads GetLastError() after a
    // SUCCESSFUL CreateFileMapping: documented to be ERROR_ALREADY_EXISTS
    // when the object existed, but not explicitly documented to be
    // cleared on a fresh create. Clear it first, so a stale value from
    // an earlier API call cannot make the genuine creator think it is an
    // attacher and wait forever for a magic only it was going to write.
    SetLastError(0);

    HANDLE handle = CreateFileMappingA(
        INVALID_HANDLE_VALUE,       // pagefile-backed: RAM only, no file
        nullptr,
        PAGE_READWRITE,
        0, (DWORD)sizeof(AP_SITL_ShmData),
        _name);
    if (handle == nullptr) {
        fprintf(stderr,
                "SITL_SharedMem: CreateFileMapping(%s) failed: error %lu\n",
                _name, (unsigned long)GetLastError());
        return false;
    }
    // set even on success, specifically so the loser of a creation race
    // can tell it attached to an existing object
    _created = (GetLastError() != ERROR_ALREADY_EXISTS);
    _win_map_handle = handle;

    _data = static_cast<AP_SITL_ShmData *>(
        MapViewOfFile(handle, FILE_MAP_ALL_ACCESS, 0, 0,
                      sizeof(AP_SITL_ShmData)));
    if (_data == nullptr) {
        fprintf(stderr,
                "SITL_SharedMem: MapViewOfFile(%s) failed: error %lu\n",
                _name, (unsigned long)GetLastError());
        _cleanup();
        return false;
    }

    if (_created) {
        // a fresh mapping arrives zero-filled, exactly like a freshly
        // ftruncate'd shm object, so the same initialisation applies
        return _init_segment_contents();
    }

    // someone else created it; wait for their magic
    if (!_wait_for_magic(deadline_us)) {
        // nothing to unlink and recreate here: the object lives exactly
        // as long as its handles, so a creator that died before
        // publishing the magic takes the segment with it, and simply
        // failing now and retrying at the next startup is correct
        _cleanup();
        return false;
    }
    return true;
}

#else  // !__CYGWIN__

/*
  create the segment, or attach to the one another instance is creating.
*/
bool AP_SITL_SharedMem::_open_segment()
{
    const uint64_t deadline_us = _now_us() + AP_SITL_SHMEM_INIT_TIMEOUT_US;
    bool replaced_stale = false;

    while (true) {
        // try to be the creator
        _fd = shm_open(_name, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
        if (_fd >= 0) {
            _created = true;
            if (_init_created_segment()) {
                return true;
            }
            _cleanup();
            return false;
        }
        if (errno != EEXIST) {
            perror("SITL_SharedMem: shm_open (create)");
            return false;
        }

        // someone else owns the name; attach to it
        _fd = shm_open(_name, O_RDWR, S_IRUSR | S_IWUSR);
        if (_fd < 0) {
            if (errno == ENOENT && _now_us() < deadline_us) {
                // it was unlinked between our two calls - go round again
                usleep(1000);
                continue;
            }
            perror("SITL_SharedMem: shm_open");
            return false;
        }
        _created = false;

        if (_map_when_ready(deadline_us)) {
            return true;
        }

        // never became usable: a leftover from a killed swarm, or a
        // segment from an incompatible build. Replace it - but only once,
        // so two instances cannot ping-pong unlinking each other's fresh
        // segments forever.
        close(_fd);
        _fd = -1;
        if (replaced_stale) {
            fprintf(stderr, "SITL_SharedMem: %s unusable; giving up\n", _name);
            return false;
        }
        replaced_stale = true;
        fprintf(stderr, "SITL_SharedMem: stale segment %s - recreating\n", _name);
        shm_unlink(_name);
    }
}

#endif  // __CYGWIN__

#ifdef __CYGWIN__
/*
  Windows wakes sleeping threads on its timer tick, which defaults to
  15.6ms - so every short sleep in the pacing and barrier paths rounds up
  to that, capping the whole cluster near realtime. timeBeginPeriod(1)
  drops the tick to 1ms process-wide. It lives in winmm.dll, which the
  toolchain does not link by default, so resolve it at runtime rather
  than adding a link dependency.
*/
static void _request_1ms_timer_resolution(void)
{
    HMODULE winmm = LoadLibraryA("winmm.dll");
    if (winmm == nullptr) {
        return;
    }
    typedef UINT (WINAPI *time_begin_period_t)(UINT);
    time_begin_period_t tbp = (time_begin_period_t)
        GetProcAddress(winmm, "timeBeginPeriod");
    if (tbp != nullptr) {
        tbp(1);
    }
    // deliberately never FreeLibrary/timeEndPeriod: the resolution should
    // hold for the life of the process
}

/*
  Windows 11 quietly re-throttles a windowless console process in two
  ways that cap SITL near a quarter of a core: timer-resolution
  coalescing IGNORES timeBeginPeriod for processes without a foreground
  window (every 1ms sleep silently becomes the 15.6ms tick again), and
  EcoQoS "efficiency mode" clamps the execution speed of anything it
  classifies as background work. SetProcessInformation with
  ProcessPowerThrottling opts out of both. The struct and constants are
  declared locally because Cygwin's headers predate them; the call is
  resolved at runtime and quietly does nothing on older Windows.
*/
static void _request_full_speed_scheduling(void)
{
    typedef struct {
        ULONG Version;
        ULONG ControlMask;
        ULONG StateMask;
    } ap_power_throttling_state_t;
    static const ULONG AP_POWER_THROTTLING_VERSION = 1;
    static const ULONG AP_THROTTLE_EXECUTION_SPEED = 0x1;
    static const ULONG AP_THROTTLE_IGNORE_TIMER_RESOLUTION = 0x4;
    static const DWORD AP_PROCESS_POWER_THROTTLING = 4;  // PROCESS_INFORMATION_CLASS

    HMODULE k32 = GetModuleHandleA("kernel32.dll");
    if (k32 == nullptr) {
        return;
    }
    typedef BOOL (WINAPI *set_process_information_t)(HANDLE, DWORD, LPVOID, DWORD);
    set_process_information_t spi = (set_process_information_t)
        GetProcAddress(k32, "SetProcessInformation");
    if (spi != nullptr) {
        ap_power_throttling_state_t state;
        state.Version = AP_POWER_THROTTLING_VERSION;
        // control both throttles, set neither: never throttle execution
        // speed, never ignore our timer-resolution request
        state.ControlMask = AP_THROTTLE_EXECUTION_SPEED |
                            AP_THROTTLE_IGNORE_TIMER_RESOLUTION;
        state.StateMask = 0;
        spi(GetCurrentProcess(), AP_PROCESS_POWER_THROTTLING,
            &state, sizeof(state));
    }
    // a simulation the user is actively watching should not lose the CPU
    // to background desktop churn
    SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS);

    /*
      self-diagnose: measure the sleep quantum actually in force. When
      Windows honors the 1ms request a 1ms sleep returns in 1-2ms; when
      the request is being ignored (observed: every thread of every
      vehicle pinned at ~26% duty cycle, all sleeps rounding to the
      15.6ms tick) it returns in ~15ms. Print the verdict so a throttled
      run is visible in the log instead of a mystery.
    */
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (uint8_t i = 0; i < 4; i++) {
        usleep(1000);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    const uint64_t elapsed_us =
        (t1.tv_sec - t0.tv_sec) * 1000000ULL +
        (t1.tv_nsec - t0.tv_nsec) / 1000;
    const uint32_t quantum_us = (uint32_t)(elapsed_us / 4);
    if (quantum_us > 5000) {
        fprintf(stderr,
                "SITL_SharedMem: WARNING: Windows is ignoring the 1ms "
                "timer request (measured %ums sleep quantum); expect the "
                "simulation to be capped near a quarter of a core\n",
                (unsigned)(quantum_us / 1000));
    } else {
        printf("SITL_SharedMem: Windows 1ms timer resolution in force "
               "(measured %uus sleep quantum)\n", (unsigned)quantum_us);
    }
}
#endif

bool AP_SITL_SharedMem::init(uint8_t instance_id, uint8_t cluster_id)
{
    if (instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        fprintf(stderr, "SITL_SharedMem: instance %u cannot join a cluster "
                        "(max instance is %u)\n",
                instance_id, AP_SITL_SHMEM_MAX_INSTANCES - 1);
        return false;
    }

    _instance_id = instance_id;
    _cluster_id  = cluster_id;

#ifdef __CYGWIN__
    _request_1ms_timer_resolution();
    _request_full_speed_scheduling();
#endif

    _build_name();

    if (!_open_segment()) {
        return false;
    }

    // claim our slot
    _data->instance[_instance_id].pid = getpid();
    _store_time(_data->instance[_instance_id].sim_time_us, 0);

    fprintf(stderr, "SITL_SharedMem: instance %u joined cluster %u (%s)\n",
            (unsigned)_instance_id, (unsigned)_cluster_id, _name);

    return true;
}

void AP_SITL_SharedMem::update(uint64_t time_us)
{
    if (_data == nullptr) {
        return;
    }
    _store_time(_data->instance[_instance_id].sim_time_us, time_us);
}

void AP_SITL_SharedMem::set_armed(bool armed)
{
    if (_data == nullptr) {
        return;
    }
    __atomic_store_n(&_data->instance[_instance_id].armed,
                     armed ? 1U : 0U, __ATOMIC_RELAXED);
}

bool AP_SITL_SharedMem::get_or_set_slave_epoch(uint64_t &wall_us,
                                               uint64_t &sim_us)
{
    if (_data == nullptr) {
        return false;
    }
    uint64_t expected = 0;
    // first-writer wins: publish the caller's proposal only if no
    // epoch exists yet, then read back whatever won
    __atomic_compare_exchange_n(&_data->slave_epoch_wall_us, &expected,
                                wall_us, false,
                                __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
    if (expected == 0) {
        // our proposal won; publish the sim half before anyone uses it
        __atomic_store_n(&_data->slave_epoch_sim_us, sim_us,
                         __ATOMIC_SEQ_CST);
    }
    const uint64_t w = __atomic_load_n(&_data->slave_epoch_wall_us,
                                       __ATOMIC_SEQ_CST);
    const uint64_t t = __atomic_load_n(&_data->slave_epoch_sim_us,
                                       __ATOMIC_SEQ_CST);
    if (w == 0 || t == 0) {
        return false;
    }
    wall_us = w;
    sim_us = t;
    return true;
}

bool AP_SITL_SharedMem::peek_slave_epoch(uint64_t &wall_us,
                                         uint64_t &sim_us) const
{
    if (_data == nullptr) {
        return false;
    }
    const uint64_t w = __atomic_load_n(&_data->slave_epoch_wall_us,
                                       __ATOMIC_SEQ_CST);
    const uint64_t t = __atomic_load_n(&_data->slave_epoch_sim_us,
                                       __ATOMIC_SEQ_CST);
    if (w == 0 || t == 0) {
        return false;
    }
    wall_us = w;
    sim_us = t;
    return true;
}

bool AP_SITL_SharedMem::all_members_armed() const
{
    if (_data == nullptr) {
        return false;
    }
    bool any = false;
    for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
        if (!instance_active(i)) {
            continue;
        }
        any = true;
        if (__atomic_load_n(&_data->instance[i].armed,
                            __ATOMIC_RELAXED) == 0) {
            return false;
        }
    }
    return any;
}

uint64_t AP_SITL_SharedMem::get_time_us(uint8_t instance_id) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return 0;
    }
    return _load_time(_data->instance[instance_id].sim_time_us);
}

bool AP_SITL_SharedMem::instance_active(uint8_t instance_id) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }
    const pid_t pid = _data->instance[instance_id].pid;
    if (pid <= 0) {
        return false;
    }
    // check pid is alive by sending signal 0
    return kill(pid, 0) == 0;
}

uint8_t AP_SITL_SharedMem::get_instance_count() const
{
    if (_data == nullptr) {
        return 0;
    }
    uint8_t count = 0;
    for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
        if (instance_active(i)) {
            count++;
        }
    }
    return count;
}

/*
  write our payload block while holding the slot's robust process-shared
  mutex. EOWNERDEAD (a prior crashed run reusing this slot) is marked
  consistent and we proceed, since we are the sole writer for our slot.
*/
void AP_SITL_SharedMem::write_payload(const void *data, uint32_t len)
{
    if (_data == nullptr) {
        return;
    }
    if (len > AP_SITL_SHMEM_PAYLOAD_SIZE) {
        len = AP_SITL_SHMEM_PAYLOAD_SIZE;
    }
    auto &slot = _data->instance[_instance_id];

#ifdef __CYGWIN__
    // no cross-process lock exists on this platform - see
    // _init_segment_contents(); the copy is unguarded
    memcpy(slot.payload, data, len);
#else
    int ret = pthread_mutex_lock(&slot.payload_mutex);
#if AP_SITL_SHMEM_ROBUST_MUTEX
    if (ret == EOWNERDEAD) {
        // we hold the lock; the previous owner died without unlocking
        ret = pthread_mutex_consistent(&slot.payload_mutex);
        if (ret != 0) {
            fprintf(stderr, "SITL_SharedMem: write_payload consistent failed: %d\n", ret);
            pthread_mutex_unlock(&slot.payload_mutex);
            return;
        }
    }
#endif
    if (ret != 0) {
        fprintf(stderr, "SITL_SharedMem: write_payload lock failed: %d\n", ret);
        return;
    }

    memcpy(slot.payload, data, len);

    pthread_mutex_unlock(&slot.payload_mutex);
#endif  // __CYGWIN__
}

/*
  read a peer's payload block while holding its robust process-shared
  mutex. EOWNERDEAD means the peer died mid-write; mark it consistent
  but treat the data as unreliable and return false.
*/
bool AP_SITL_SharedMem::read_payload(uint8_t instance_id, void *data, uint32_t len) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }
    if (len > AP_SITL_SHMEM_PAYLOAD_SIZE) {
        len = AP_SITL_SHMEM_PAYLOAD_SIZE;
    }
    auto &slot = _data->instance[instance_id];

#ifdef __CYGWIN__
    // no cross-process lock exists on this platform - see
    // _init_segment_contents(); the copy is unguarded
    memcpy(data, slot.payload, len);
    return true;
#else
    int ret = pthread_mutex_lock(&slot.payload_mutex);
    bool owner_died = false;
#if AP_SITL_SHMEM_ROBUST_MUTEX
    if (ret == EOWNERDEAD) {
        owner_died = true;
        ret = pthread_mutex_consistent(&slot.payload_mutex);
        if (ret != 0) {
            fprintf(stderr, "SITL_SharedMem: read_payload consistent failed: %d\n", ret);
            pthread_mutex_unlock(&slot.payload_mutex);
            return false;
        }
    }
#endif
    if (ret != 0) {
        fprintf(stderr, "SITL_SharedMem: read_payload lock failed: %d\n", ret);
        return false;
    }

    if (!owner_died) {
        memcpy(data, slot.payload, len);
    }

    pthread_mutex_unlock(&slot.payload_mutex);

    return !owner_died;
#endif  // __CYGWIN__
}


/*
  spin-wait barrier: pause until every peer in the cluster has published
  a sim_time_us >= (sim_time_us - max_skew_us), keeping all instances
  within max_skew_us of each other. Called from Aircraft::sync_frame_time()
  after every simulation step.

  Membership is whatever is registered right now: an unclaimed slot is
  not a peer, so being alone in the cluster costs a single scan and no
  waiting at all. A wall-clock timeout still guards against a peer that
  is alive but wedged.
*/
bool AP_SITL_SharedMem::sync_with_peers(uint64_t sim_time_us,
                                         uint64_t max_skew_us,
                                         uint64_t timeout_us,
                                         uint64_t stall_grace_us)
{
    if (_data == nullptr) {
        return true;
    }

    // minimum sim_time we are willing to be ahead of peers
    const uint64_t min_peer_time = (sim_time_us > max_skew_us)
                                    ? sim_time_us - max_skew_us
                                    : 0;

    // wall-clock start, for the overall timeout
    const uint64_t start_us = _now_us();

    while (true) {
        // wall-clock "now", used both for the overall timeout and for
        // per-peer stall detection below
        const uint64_t now_us = _now_us();

        bool all_ok = true;
        uint8_t live_peers = 0;
        for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
            if (i == _instance_id) {
                continue;
            }
            const pid_t pid = _data->instance[i].pid;
            if (pid <= 0) {
                // nobody has ever claimed this slot, or its owner has
                // left. Either way it is not a peer - we never wait for
                // an instance that has not joined the cluster.
                continue;
            }
            // Is the peer still alive? Throttled: this is a syscall, and
            // the loop around it spins every ~100us.
            bool peer_gone = false;
            if (now_us - _peer_checked_wall_us[i] > AP_SITL_SHMEM_LIVENESS_CHECK_US) {
                _peer_checked_wall_us[i] = now_us;
                peer_gone = kill(pid, 0) != 0;
            }
            if (peer_gone) {
                // mark the slot dead (-1, not 0) so a later joiner can
                // tell "gone" from "never used"
                _data->instance[i].pid = -1;
                _peer_joined_announced[i] = false;
                fprintf(stderr,
                        "SITL_SharedMem: peer instance %u (pid %d) has left "
                        "cluster %u; %u instances remain\n",
                        (unsigned)i, (int)pid, (unsigned)_cluster_id,
                        (unsigned)get_instance_count());
                continue;
            }

            if (!_peer_joined_announced[i]) {
                _peer_joined_announced[i] = true;
                fprintf(stderr,
                        "SITL_SharedMem: peer instance %u joined cluster %u\n",
                        (unsigned)i, (unsigned)_cluster_id);
            }

            // track whether this peer's sim_time_us is still advancing;
            // after stall_grace_us with no movement we stop waiting on
            // it. A reboot resets sim_time_us to 0 - that backward jump
            // must not count as progress or it would restart the timer.
            const uint64_t peer_time = _load_time(_data->instance[i].sim_time_us);
            bool peer_advanced = false;
            if (peer_time > _peer_seen_time_us[i]) {
                peer_advanced = true;
                _peer_seen_time_us[i] = peer_time;
                _peer_seen_wall_us[i] = now_us;
            } else if (peer_time != _peer_seen_time_us[i]) {
                // backward jump (e.g. reboot reset) - remember the new
                // value for future comparisons, but don't treat it as
                // progress or reset the grace-period clock
                _peer_seen_time_us[i] = peer_time;
            }

            if (_peer_stalled[i]) {
                // Only rejoin once genuinely caught up (matches the
                // threshold is_behind_peers() uses to keep sprinting).
                // The cooldown below, not a wider margin, prevents flapping.
                if (peer_time >= min_peer_time) {
                    _peer_stalled[i] = false;
                    _peer_resume_announced[i] = false;
                    // brief cooldown so a scheduling hiccup right after
                    // rejoining doesn't immediately re-exclude the peer
                    _peer_cooldown_until_us[i] = now_us + 2 * stall_grace_us;
                    fprintf(stderr,
                            "SITL_SharedMem: peer instance %u has "
                            "caught-up to the shared sim time\n", (unsigned)i);
                } else {
                    if (peer_advanced && !_peer_resume_announced[i]) {
                        _peer_resume_announced[i] = true;
                        fprintf(stderr,
                                "SITL_SharedMem: peer instance %u resumed, "
                                "still catching up\n", (unsigned)i);
                    }
                    continue;
                }
            } else if (peer_time < min_peer_time &&
                       now_us - _peer_seen_wall_us[i] > stall_grace_us &&
                       now_us >= _peer_cooldown_until_us[i]) {
                // only stall a peer that is actually behind (blocking us);
                // a peer that simply updates less often than we poll but
                // is still within skew must never be marked stalled, or it
                // will instantly "rejoin" and flap forever
                _peer_stalled[i] = true;
                _peer_resume_announced[i] = false;
                fprintf(stderr,
                        "SITL_SharedMem: peer instance %u stalled "
                        "(%.2fs); no longer blocking cluster\n",
                        (unsigned)i, stall_grace_us * 1e-6);
                continue;
            }

            // this peer is live and participating in the barrier
            live_peers++;

            if (peer_time < min_peer_time) {
                all_ok = false;
                break;
            }
        }
        if (all_ok) {
            // only claim lock-step if we actually barriered against
            // somebody; "in lock-step" alone in the cluster is noise
            if (!_sync_announced && live_peers > 0) {
                _sync_announced = true;
                fprintf(stderr,
                        "SITL_SharedMem: instance %u in lock-step with %u "
                        "instances in cluster %u (max skew %llu us)\n",
                        (unsigned)_instance_id, (unsigned)(live_peers + 1),
                        (unsigned)_cluster_id,
                        (unsigned long long)max_skew_us);
            }
            return true;
        }

        // check wall-clock timeout
        const uint64_t elapsed_us = now_us - start_us;
        if (elapsed_us > timeout_us) {
            fprintf(stderr,
                    "SITL_SharedMem: sync timeout after %.1f s waiting for "
                    "peers (our sim_time_us=%llu)\n",
                    elapsed_us * 1e-6,
                    (unsigned long long)sim_time_us);
            return false;
        }

        /*
          In steady state peers are only microseconds apart, so the first
          stretch of waiting busy-spins: an OS sleep here costs at least
          one scheduler quantum (~1ms on Linux, up to 15.6ms on a Windows
          box at default timer resolution), which is what throttled
          clustered runs to near realtime however fast the machine was.
          Only a wait long enough to indicate a genuinely lagging peer
          falls back to sleeping.
        */
        if (elapsed_us > 500) {
            usleep(200);
        }
    }
}

/*
  return true if our own sim_time_us is more than max_skew_us behind the
  most-advanced live peer - used to trigger a ~2x-speedup catch-up
  sprint (see Aircraft::sync_frame_time()). The sprint is capped at
  max_catchup_us of wall-clock time, after which we give up and run at
  normal speed (still excluded from the barrier by sync_with_peers()).
*/
bool AP_SITL_SharedMem::is_behind_peers(uint64_t sim_time_us, uint64_t max_skew_us,
                                         uint64_t max_catchup_us)
{
    if (_data == nullptr) {
        return false;
    }

    const uint64_t max_peer_time = _max_peer_time();
    const bool behind = sim_time_us + max_skew_us < max_peer_time;
    const uint64_t now_us = _now_us();

    if (!behind) {
        // caught up (or never fell behind) - reset catch-up tracking so a
        // future reboot starts a fresh sprint window
        _catchup_started_wall_us = 0;
        _catchup_gave_up = false;
        return false;
    }

    if (_catchup_gave_up) {
        // already gave up sprinting this episode; stay at normal speed
        // until we catch up naturally (behind will go false above once
        // we do, which resets _catchup_gave_up for next time)
        return false;
    }

    if (_catchup_started_wall_us == 0) {
        _catchup_started_wall_us = now_us;
    } else if (now_us - _catchup_started_wall_us > max_catchup_us) {
        _catchup_gave_up = true;
        fprintf(stderr,
                "SITL_SharedMem: instance %u could not catch up to peers "
                "within %.1fs; giving up sprint, running at normal speed\n",
                (unsigned)_instance_id, max_catchup_us * 1e-6);
        return false;
    }

    return true;
}

bool AP_SITL_SharedMem::instant_catchup_if_new(uint64_t &sim_time_us, uint64_t max_uptime_us)
{
    if (_instant_catchup_done || _data == nullptr) {
        return false;
    }

    const uint64_t now_us = _now_us();
    if (now_us - _process_start_wall_us > max_uptime_us) {
        // too long since our own process started - not a fresh join
        // any more, don't instantly jump the clock
        _instant_catchup_done = true;
        return false;
    }

    const uint64_t max_peer_time = _max_peer_time();
    if (max_peer_time <= sim_time_us) {
        // not behind anyone - nothing to do, but don't consume our one
        // shot in case we fall behind moments later during startup
        return false;
    }

    _instant_catchup_done = true;
    fprintf(stderr,
            "SITL_SharedMem: instance %u joining a running cluster, jumping "
            "clock to match peers (%.2fs ahead)\n",
            (unsigned)_instance_id, (max_peer_time - sim_time_us) * 1e-6);
    sim_time_us = max_peer_time;
    return true;
}

void AP_SITL_SharedMem::_clear_slot()
{
    if (_data == nullptr) {
        return;
    }
    _store_time(_data->instance[_instance_id].sim_time_us, 0);
    // -1 (not 0) so peers can tell "left the cluster" from "never used"
    _data->instance[_instance_id].pid = -1;
}

void AP_SITL_SharedMem::_cleanup()
{
    _clear_slot();

#ifndef __CYGWIN__
    // if no other member is still alive the segment is now the record
    // of a finished cluster - a stale epoch and armed flags that the
    // next run's vehicles would join by name. The last member out
    // retires it, creator or not. (Members that die without cleanup
    // are covered by the stale-segment recovery in _attach and by the
    // harness unlinking its cluster's segment around each run.)
    bool peers_alive = false;
    if (_data != nullptr) {
        for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
            const pid_t pid = _data->instance[i].pid;
            if (pid > 0 && pid != getpid() &&
                (kill(pid, 0) == 0 || errno == EPERM)) {
                peers_alive = true;
                break;
            }
        }
    }
#endif

    _unmap();

#ifdef __CYGWIN__
    if (_win_map_handle != nullptr) {
        CloseHandle((HANDLE)_win_map_handle);
        _win_map_handle = nullptr;
    }
    // no unlink exists or is needed: the kernel deletes the mapping
    // object when its last handle closes
#else
    if (_fd >= 0) {
        close(_fd);
        _fd = -1;
    }

    // unlink when this is the creator or the last live member; if
    // other instances are still alive an unlink would be harmless to
    // them (they hold their own mappings) but would orphan the name
    // they expect late joiners to find, so leave it to the last out
    if ((_created || !peers_alive) && _name[0] != 0) {
        shm_unlink(_name);
        _created = false;
    }
#endif
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

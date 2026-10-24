# Native module design

`AP_Module` provides trusted native extensions with explicit image, heap, worker
and resource ownership. It is independent of Lua but shares scripting-level
APIs/facilities where appropriate.

See [README.md](README.md) for build/install instructions and operator settings;
this document describes the current implementation contracts.

## Trust and platform boundary

Modules execute with firmware privileges. Checksums detect corruption, not
authenticity, and Rust's type system is not an isolation boundary. Bad pointers,
arbitrary faults, unbounded host calls and locks held indefinitely can affect the
vehicle. Low priority and the optional stack guard do not make untrusted modules
safe. There is no general fault containment or per-module CPU quota.

Supported profiles are Thumb-2 Cortex-M4/FPv4-SP and Cortex-M7/FPv5-D16 hard float
on STM32, and baseline x86-64 Linux for SITL/Linux. Rust builds require Rust 1.98.1 or newer;
C++ builds require Clang/standalone LLD and do not invoke Rust tools.
Thumb uses ROPI/RWPI; x86-64 uses PIC. Fat LTO applies that model to linked Rust
bitcode, including core/alloc. Rust modules use `no_std`, panic-abort semantics
and SDK memory helpers, without host libc or system startup imports. C++ uses a
freestanding C++17 runtime. Fortran uses `ISO_C_BINDING` and generic templates for
the same complete host-call catalogue, without linking libgfortran. Its local C
bridge shares the staged field conversions and error/copyback rules with Ada,
then delegates wire encoding to the existing C++ clients. Unsigned values cross
this local boundary as same-width signed integer bit patterns; booleans use C
`bool` and Fortran `logical(c_bool)`. Strings carry explicit byte lengths and
arrays/optional values use fixed-size interoperable records, not descriptors.
GCC's ARM PIC subset still excludes mutable Fortran static data; unsupported
runtime operations and relocations fail linking or packing.

The experimental Ada backend emits GNAT LLVM IR and uses LLVM's ROPI/RWPI or
PIC backend. Generic templates emit Ada clients for the same complete host-call
catalogue and a local typed C bridge to the existing C++ clients. C-compatible
record layouts are explicit in Ada and checked in C++; firmware still receives
the existing field-wise wire encoding, not compiler object layouts. Staging
preserves mutable inputs on error, and generated outputs are initialized.
There is no binder or linked Ada runtime:
compile-time restrictions exclude tasking, allocation, finalization, secondary
stacks, implicit executable trampolines, exception handlers and elaboration code. Static globals are supported.
Runtime checks remain enabled and call non-returning module-panic helpers.
Unimplemented runtime dependencies fail linking; they are not returning stubs.

## Binding generation and ABI

The Python Lua generator parses `bindings.desc` and exports versioned JSON IR.
The native generator consumes this IR rather than duplicating the grammar. It
generates firmware adapters, private dispatch IDs, Rust, C++, Ada and Fortran clients, manifests, contract
catalogues and coverage reports. Outputs live under build directories. Waf
tracks both generators' inputs; standalone and Cargo builds generate their own
outputs.

Native source templates live in `generator/templates/`, with separate Rust,
C++ client and firmware C++ templates. Like the Lua generator, substitution uses
Python's strict `string.Template`: semantic checks and iteration stay in Python,
missing or unused substitutions fail generation, and literal dollar signs use `$$`.
Waf tracks nested template inputs; Cargo and standalone builds use the same
generator. Templates add no runtime layer: Rust still emits direct host calls
and retains its own types, lifetimes, fixed wire buffers and LTO path. Other
language backends must not impose C-interoperability wrappers on Rust.

Audited adapters are being migrated to the language-independent `call_model.py`
descriptions. These record scalar/enum/record layouts, borrowed selectors,
context-owned tokens, optional results and public method shapes. `rust_calls.py`
renders them through generic templates, sharing call helpers by wire signature
without adding runtime schemas or allocation. C++ clients use the same plans
for argument/result types, packet layouts and presence-only selector replies:
a backend's presence flag does not carry another index byte; its index comes
from the original request.
Consuming-close Rust handles are non-copyable and need no runtime open/closed
flag. Close consumes such a handle even on a host error, so it cannot be retried.
Any resource left allocated is released by firmware's module-exit cleanup.
Borrowed serial tokens are copyable and revalidated by firmware on every call;
zero tokens indicate absent factory results, not a live selector.

Bounded byte buffers describe their direction, capacity and empty-call policy.
Explicit request layouts place length words independently of public argument
order. Replies describe either a checked partial count or a presence flag;
caller buffers change only after a successful, validated response. Fixed NUL-free
strings and optional nonzero scalar inputs use the same model. Serial, I2C and
ONVIF clients are generated from these descriptions in both languages. Rust takes
borrowed slices and UTF-8 `&str`; C++ exposes fixed records/arrays, explicit lengths
and byte-oriented `StringView`. String capacity includes the trailing NUL; both
clients reject embedded NULs and zero-pad the fixed wire field. C++ does not
validate UTF-8. Sharing helpers by encoded scalar types avoids duplicating
enum/bool setting calls.
Rust adapters not yet migrated retain specialised source templates. C++ derives
the remaining adapter types from the audited contract catalogue. The shared
model supplements that catalogue; it does not replace the compatibility serializer.

`generator/coverage.json` uses schema 2 and reports each language separately:
host-call adapters, local helpers, value fields, native-language operations,
replacement-only APIs and unsupported entries. Replacement-only does not mean
the original named API exists, and replacements can have different units. The
host-call total is separate from the Lua declaration count; extra native calls
and multiple adapters per declaration mean those totals need not match.
Coverage scope remains methods, manual aliases and audited FWVersion fields,
not every field, constructor, enum, operator or global in `bindings.desc`.

Singleton methods are selected by signature/type category. Audited policies
describe ownership, lifetime, nonstandard accessors and exclusions missing from
the Lua description. Values use explicit component wire layouts, not C++ object
bytes. Complete Location and mission-item copies preserve hidden semantic fields.
New value methods conservatively copy back mutable inputs on host success,
including false results; host errors leave client values unchanged.

Pointer-backed objects need a lifetime strategy: RC channels, for example, use
revalidated indices rather than exported C++ pointers. Resource adapters track
ownership against module contexts. Unsupported categories are recorded with
reasons in `generator/coverage.json`. Manual Lua functions require deliberate
native adapters.

The C-compatible entry receives a host API table and opaque context and returns
an `int32_t` status. Rust wraps its entry using `module!`; C++ uses `AP_MODULE`. C++
classes, Rust trait objects, slices and native strings do not cross the ABI.
Core calls use fixed-width scalars, explicit lengths and defined C layouts.

`generator/host_api.json` defines core ABI version 3: eight required callbacks
for allocation, deallocation, time, sleep, stop, text, panic and binding dispatch.
The 16-byte fingerprint covers that required prefix, status codes and layout.
Appended callbacks may preserve the prefix; incompatible prefix/entry changes
require a new core ABI version.

Each binding hash is the first eight SHA-256 digest bytes of its canonical
public contract, interpreted little-endian. Inputs include public names, types,
directions, transitive wire layouts, optionality, ranges, lock policy and semantic
revisions. Private implementation names, dispatch numbers, source order, guards
and unrelated APIs are excluded. Enum type names and fixed literal expressions
are included. Collisions fail generation. Semantic changes not expressed in the
contract need a revision in `contract_revisions.json`.

Each used wrapper references a four-byte ELF import placeholder. Linker garbage
collection removes unused wrappers/imports. The packer retains live hashes,
slots and diagnostic names. Loading resolves them to firmware-private `BindingId`
values before execution; calls do not look up hashes. Unknown imports reject
loading with their name/hash. Known compiled-out APIs retain contracts and return
`Unavailable`. Unrelated binding additions do not invalidate a module.

Generated fixed-packet adapters trust buffer sizes supplied by hash-compatible
wrappers and omit repeated size checks. Handwritten resource adapters may retain
their own packet validation. Pointers are trusted, not sandbox-validated. The wire
is little-endian and input/output buffers must not overlap. Availability, ranges,
values and handle ownership are still checked. Common codecs avoid C++ padding.
Completely overwritten outputs are not pre-zeroed; absent/variable payloads are
initialized where required for canonical output.

Semaphore acquisition blocks. Locks are released before returning to module code;
firmware does not call back into module code. SDK proxies borrow a context; Rust
proxies are neither `Send` nor `Sync`. Ordinary state changes retain Lua semantics and
are not rolled back on exit. A stop request cannot interrupt an in-flight host
call.

## Container and relocation

The offline packer links ELF, resolves image-local instruction/PC-relative
references and emits `.apmod`. Firmware is not an ELF linker. Unsupported
sections, undefined symbols, TLS, constructors, unwinding and unsupported
relocations are rejected. ARM GOT/dynamic-linking tables and static Thumb
MOVW/MOVT instruction patches are unsupported. x86-64 GOT entries must resolve
to defined image symbols.

The current runtime format is `APMOD07`, little-endian with an 88-byte header:

| Field | Encoding |
| --- | --- |
| Magic | 8 bytes, `APMOD07` followed by NUL |
| Version, ELF machine, pointer width | Three u32 values |
| Segment alignments | u32: low 16 bits RW, high 16 bits RX |
| Initialized size, memory size, RX boundary, entry offset | Four u32 values |
| Fixup count, ABI version, CPU profile, stack requirement, initial heap, import count | Six u32 values |
| Core fingerprint | 16 bytes |
| Native thread priority | u32, 1 through 254, further restricted by the HAL and `MOD_THD_PRIO_MAX` |
| Container CRC32 | u32 |

Initialized image bytes follow, then sorted u32 pointer-fixup destination offsets,
sorted 16-byte import records (`u64 hash`, `u32 slot`, `u32 name offset`) and
canonical NUL-terminated API names. Fixup slots hold canonical image-relative
target offsets. Imports are sorted by slot; at most 4096 imports and 63-byte names
are accepted. Duplicate hashes/slots, invalid placeholders and overlaps with
pointer fixups are rejected.

The logical RX boundary is word-aligned on ARM and 4096-byte aligned on x86-64
for POSIX permission separation. Segment alignments are powers of two from
pointer width through 4096, derived from ELF requirements. RAM-loaded ARM RX
allocations isolate 32-byte cache lines and honor larger ELF alignment; ROMFS
payloads need only their ELF alignment. RW includes
initialized data and zero-filled BSS. Parsing checks bounds, overflow, entry,
fixup destinations/targets, imports, target, core compatibility and configured
budgets before publication. Stack declarations are trusted metadata, not a proof.

CRC32 covers the canonical file with its header CRC zeroed. Runtime loaded/running
checksums separately cover the complete canonical file including the stored CRC.
ROMFS prelinking preserves both identities by normalizing patched RX slots.

All old `APRUST` formats are rejected. `APMOD01` (NUL-padded) is a probe accepted
only by the separate test parser with `AP_MODULE_LEGACY_PROBE_ENABLED`.
Current SDK builds emit version 7; core ABI version 3 is separately versioned.
These are not released compatibility promises. Frozen `tests/compat/` fixtures
and catalogue checks detect unintended changes; do not routinely regenerate them.

CPU requirements are explicit container profiles, not ordered capability levels:

| Profile | Requirement | Supported boards |
| --- | --- | --- |
| 1 | Cortex-M4, FPv4-SP-D16, hard float | F405 and supported H7 |
| 2 | Baseline x86-64 Linux | x86-64 SITL/Linux |
| 3 | Cortex-M7, FPv5-D16 including double precision, hard float | H743/H757 |

The HAL reports the execution CPU capability, including the active core on H7.
The image parser accepts profile 1 on an M7/DP board as well as profile 3, but
rejects M7 requirements on M4, cross-architecture combinations and unknown IDs
before allocation. Older firmware rejects the new M7 profile; rebuild firmware
before using it. Existing CPU profile IDs are unchanged. CPU and
stack requirements are trusted declarations, not a sandbox or instruction verifier.

## Executable memory and ROMFS

ARM ROPI/RWPI allows separate RX/RW bases. The entry trampoline saves/restores
`r9` and installs the module RW base. SD loading copies both segments. Absolute
RX-to-RW fixups are rejected; RX-to-RX pointer slots can be resolved by the
firmware linker. Target offsets preserve byte-pointer and Thumb-function bits.

ROMFS stores `.apmod` entries raw while other files follow normal compression
policy. Firmware generation prelinks ARM RX-to-RX slots and places entries in
executable flash, preserving payload alignment after the 88-byte header. Borrowing
requires HAL confirmation that the complete range is immutable executable flash;
filename or ROMFS membership alone is insufficient. RW/BSS is always copied,
fixed up and owned. Borrowed RX has firmware lifetime and is never written/freed.

`AP_ROMFS::find_uncompressed` exposes raw storage for loading. Ordinary filesystem
reads reconstruct canonical module bytes and validate integrity. Globally
uncompressed ROMFS builds do not prelink: they preserve existing borrowed-file
semantics and the runtime copies RX.

STM32 executable allocation selects instruction-capable SRAM, not arbitrary
malloc memory or F4 CCM. Publication performs cache maintenance/barriers, without
per-image MPU isolation. POSIX uses dedicated contiguous mappings for PIC: copy
RX/RW, publish the code prefix as RX and leave data RW/non-executable. SITL ROMFS
arrays are not executable and are copied. Emulation does not establish real
cache behavior.

## Module and generation ownership

`Image` parses/relocates without owning buffers. `Module` owns code, mutable data,
heap, host context and one worker. File source bytes can be released after load;
borrowed ROMFS RX remains valid for firmware lifetime. HAL owned threads retain
stack ownership until join/reap, unlike fire-and-forget threads.

`Runtime` discovers nonrecursive `*.apmod` files in ROMFS `modules/` and the writable
directory and deduplicates basenames with ROMFS precedence/warning. A sorted,
heap-allocated list holds filenames and address-stable callback contexts; no
module-count parameter, fixed array or count-dependent manager stack is used.
Metadata allocation failure rejects the generation. All images are prepared before workers start so stacks
and heaps do not consume scarce executable RAM first. Partial startup failure
stops/reaps the new generation. The vehicle main loop never waits for worker exit.

Restart requests stop for all workers. SDK sleeps notice stop promptly; entry
functions return normally so destructors run. Bindings reject new work after
stop, while allocation/deallocation remain available within the heap budget
through cleanup. The manager joins returned workers before releasing code,
handles, data, heap or stack. Threads are never forcibly cancelled and no new
generation runs alongside live users of the old one.

A ten-second timeout retains the stopping generation and reports failure. Late
exits can be reaped; an explicit request retries loading. Panic/infallible OOM
invoke a host callback without unwinding through C++, mark failure and park until
reboot with resources retained. The SDK never uses whole-firmware panic or longjmp.

Host resources are scoped to the owning context and cleaned up only after users
join. No module function pointers are registered with firmware. DMA-sensitive
adapters use host storage or driver/filesystem bounce-buffer paths: worker stacks
need not be DMA-capable. Ordinary vehicle state is not restored implicitly.
The manager deletes the generation's list only once all modules are joined and
binding cleanup is complete; stop timeouts retain live contexts unchanged.
MAVLink and socket owner registries also allocate entries on demand and free
them on cleanup. Socket generations use a global monotonic counter so freeing
an owner cannot make stale handles valid after its address is reused. Token
exhaustion fails closed rather than wrapping. Per-module socket/queue limits
and shared device-pool budgets are independent of the number of modules.

## Memory, scheduling and health

Budgets independently cap backing heap capacity, RX plus RW/BSS image and worker stack.
The container declares an initial heap and module stack requirement. Admission
checks the initial heap against `MOD_HEAP_MAX` and declared stack plus host-call
margin against `MOD_STACK_MAX` before image allocation. Workers allocate only
those per-module requests. Armed expansion cannot exceed the heap cap.
Image limits round to 4096; actual allocation follows image size, not the limit.
HAL stack rounding and OS/guard overhead are reported separately. Host-call margin
is 4 KiB on Cortex-M and 16 KiB on x86-64, beyond the module declaration. Manager
stack requests are 4 KiB on ARM and 32 KiB on x86-64. Cortex-M text callbacks have
at most 2 KiB additional stack budget and queue formatting elsewhere.

The aligned AP_MultiHeap adapter serializes operations and tracks raw allocations
for aligned frees and reclamation. Free walks live allocations to check ownership.
Lua-style armed expansion retains a 20 KiB system reserve unless disabled for the
generation; it latches a warning/pre-arm error without stopping the worker.
Reports distinguish used/peak heap, capacity, owned RX/RW, stack/thread reservations
and metadata. Backing allocator, loader scratch, manager and persistent firmware
storage are additional, not a complete physical-memory total.

Parameter definitions and logger formats are firmware-owned until reboot, outside
module heaps. Identical parameter registration reuses metadata and restores saved
values; unsaved values can reset. Lua and native modules share dynamic-table limits. Cached
`Parameter` queries host size/alignment once, placement-constructs the shared
trivially-destructible helper in module-owned storage and frees it on drop.
C++ layout is not part of the public wire contract.

Workers use the native priority declared in their header. The loader checks it
against the HAL's application range and `MOD_THD_PRIO_MAX` before allocation.
The limit defaults to 2 on STM32 and 1 on other HALs; the manager runs at the
limit plus one, not just one above a particular worker. Limits must leave room
for the manager within the HAL's native priority range. STM32 permits limits
2..254; Linux and SITL permit 1..19. SITL uses nominal Linux priority values but
retains `SCHED_OTHER`, as does non-realtime Linux. Linux FIFO and ChibiOS honor
the native priorities. The defaults remain below vehicle/I/O work; raising the
limit can interfere with flight-critical threads. ChibiOS's normal zero quantum
means no peer round-robin: busy workers can starve equal/lower-priority modules
and Lua. Positive sleeps cooperate; zero sleeps and stop polling do not yield.

The priority field makes `APMOD07` incompatible with `APMOD06`; rebuild modules
and firmware together. The core binding ABI is unchanged. Offline resource and
priority edits preserve machine code but change the whole-file checksum.

Health participates in SYSTEM pre-arm checks and existing disable/force-arm
bypasses. Errors latch until the next generation; empty directories/normal returns
are not failures. Loaded/running requirements are 23-bit-masked XORs of canonical
whole-file CRC32 values, reset each generation. Normally returned modules remain
loaded but no longer running. Armed restart follows the same ownership rules;
Module errors never automatically disarm an already armed vehicle.

## Optional H7 stack-fault recovery

`HAL_STACK_GUARD_ENABLED` defaults off and is limited to supported H7 builds,
excluding bootloaders and IOMCU. Guarded workers have a 1 KiB-aligned
no-access area below the working area. MPU region 7 follows the guarded thread
at context switch; the null-pointer guard uses region 3.

USB debugging can be enabled in the same build. Its fault handler first repairs
guarded debugger probes, then attempts stack recovery before inspecting an
application exception frame or completing lazy FP stacking. A refused recovery
retains the normal debugger/crash path; incomplete frames never enter the monitor.
Recoverable overflows terminate the worker even while a debugger is attached;
break on `ap_stack_guard_recover` to inspect one before recovery.

Recovery requires a thread-mode PSP fault without suspended handler or interrupt
mask, with guard/allocation attribution from fault status and addresses, including
lazy FP state. Other faults follow normal crash handling. Recovery abandons damaged
frames, installs a fresh ChibiOS extended frame, notifies the owner through a
lock-free callback, releases owned mutexes with `chMtxUnlockAllS()` and exits with
`chThdExitS()` in one locked scheduler transaction. The manager joins before cleanup.

Forced mutex release cannot roll back partially completed host operations; binary
semaphores and non-mutex state are not reconstructed. A frame/write can skip the
entire guard. Arbitrary pointers and all nested/critical-section faults are not
contained. Recovery latches a module error without disarming. This narrow mechanism
does not replace stack measurement or provide general memory protection.

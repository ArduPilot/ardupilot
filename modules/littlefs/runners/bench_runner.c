/*
 * Runner for littlefs benchmarks
 *
 * Copyright (c) 2022, The littlefs authors.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include "runners/bench_runner.h"
#include "bd/lfs_emubd.h"

#include <getopt.h>
#include <sys/types.h>
#include <errno.h>
#include <setjmp.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <execinfo.h>
#include <time.h>


// some helpers

// append to an array with amortized doubling
void *mappend(void **p,
        size_t size,
        size_t *count,
        size_t *capacity) {
    uint8_t *p_ = *p;
    size_t count_ = *count;
    size_t capacity_ = *capacity;

    count_ += 1;
    if (count_ > capacity_) {
        capacity_ = (2*capacity_ < 4) ? 4 : 2*capacity_;

        p_ = realloc(p_, capacity_*size);
        if (!p_) {
            return NULL;
        }
    }

    *p = p_;
    *count = count_;
    *capacity = capacity_;
    return &p_[(count_-1)*size];
}

// a quick self-terminating text-safe varint scheme
static void leb16_print(uintmax_t x) {
    // allow 'w' to indicate negative numbers
    if ((intmax_t)x < 0) {
        printf("w");
        x = -x;
    }

    while (true) {
        char nibble = (x & 0xf) | (x > 0xf ? 0x10 : 0);
        printf("%c", (nibble < 10) ? '0'+nibble : 'a'+nibble-10);
        if (x <= 0xf) {
            break;
        }
        x >>= 4;
    }
}

static uintmax_t leb16_parse(const char *s, char **tail) {
    bool neg = false;
    uintmax_t x = 0;
    if (tail) {
        *tail = (char*)s;
    }

    if (s[0] == 'w') {
        neg = true;
        s = s+1;
    }

    size_t i = 0;
    while (true) {
        uintmax_t nibble = s[i];
        if (nibble >= '0' && nibble <= '9') {
            nibble = nibble - '0';
        } else if (nibble >= 'a' && nibble <= 'v') {
            nibble = nibble - 'a' + 10;
        } else {
            // invalid?
            return 0;
        }

        x |= (nibble & 0xf) << (4*i);
        i += 1;
        if (!(nibble & 0x10)) {
            s = s + i;
            break;
        }
    }

    if (tail) {
        *tail = (char*)s;
    }
    return neg ? -x : x;
}



// bench_runner types

typedef struct bench_geometry {
    const char *name;
    bench_define_t defines[BENCH_GEOMETRY_DEFINE_COUNT];
} bench_geometry_t;

typedef struct bench_id {
    const char *name;
    const bench_define_t *defines;
    size_t define_count;
} bench_id_t;


// bench suites are linked into a custom ld section
extern struct bench_suite __start__bench_suites;
extern struct bench_suite __stop__bench_suites;

const struct bench_suite *bench_suites = &__start__bench_suites;
#define BENCH_SUITE_COUNT \
    ((size_t)(&__stop__bench_suites - &__start__bench_suites))


// bench define management
typedef struct bench_define_map {
    const bench_define_t *defines;
    size_t count;
} bench_define_map_t;

typedef struct bench_define_names {
    const char *const *names;
    size_t count;
} bench_define_names_t;

intmax_t bench_define_lit(void *data) {
    return (intptr_t)data;
}

#define BENCH_CONST(x) {bench_define_lit, (void*)(uintptr_t)(x)}
#define BENCH_LIT(x) ((bench_define_t)BENCH_CONST(x))


#define BENCH_DEF(k, v) \
    intmax_t bench_define_##k(void *data) { \
        (void)data; \
        return v; \
    }

    BENCH_IMPLICIT_DEFINES
#undef BENCH_DEF

#define BENCH_DEFINE_MAP_OVERRIDE    0
#define BENCH_DEFINE_MAP_EXPLICIT    1
#define BENCH_DEFINE_MAP_PERMUTATION 2
#define BENCH_DEFINE_MAP_GEOMETRY    3
#define BENCH_DEFINE_MAP_IMPLICIT    4
#define BENCH_DEFINE_MAP_COUNT       5

bench_define_map_t bench_define_maps[BENCH_DEFINE_MAP_COUNT] = {
    [BENCH_DEFINE_MAP_IMPLICIT] = {
        (const bench_define_t[BENCH_IMPLICIT_DEFINE_COUNT]) {
            #define BENCH_DEF(k, v) \
                [k##_i] = {bench_define_##k, NULL},

                BENCH_IMPLICIT_DEFINES
            #undef BENCH_DEF
        },
        BENCH_IMPLICIT_DEFINE_COUNT,
    },
};

#define BENCH_DEFINE_NAMES_SUITE    0
#define BENCH_DEFINE_NAMES_IMPLICIT 1
#define BENCH_DEFINE_NAMES_COUNT    2

bench_define_names_t bench_define_names[BENCH_DEFINE_NAMES_COUNT] = {
    [BENCH_DEFINE_NAMES_IMPLICIT] = {
        (const char *const[BENCH_IMPLICIT_DEFINE_COUNT]){
            #define BENCH_DEF(k, v) \
                [k##_i] = #k,

                BENCH_IMPLICIT_DEFINES
            #undef BENCH_DEF
        },
        BENCH_IMPLICIT_DEFINE_COUNT,
    },
};

intmax_t *bench_define_cache;
size_t bench_define_cache_count;
unsigned *bench_define_cache_mask;

const char *bench_define_name(size_t define) {
    // lookup in our bench names
    for (size_t i = 0; i < BENCH_DEFINE_NAMES_COUNT; i++) {
        if (define < bench_define_names[i].count
                && bench_define_names[i].names
                && bench_define_names[i].names[define]) {
            return bench_define_names[i].names[define];
        }
    }

    return NULL;
}

bool bench_define_ispermutation(size_t define) {
    // is this define specific to the permutation?
    for (size_t i = 0; i < BENCH_DEFINE_MAP_IMPLICIT; i++) {
        if (define < bench_define_maps[i].count
                && bench_define_maps[i].defines[define].cb) {
            return true;
        }
    }

    return false;
}

intmax_t bench_define(size_t define) {
    // is the define in our cache?
    if (define < bench_define_cache_count
            && (bench_define_cache_mask[define/(8*sizeof(unsigned))]
                & (1 << (define%(8*sizeof(unsigned)))))) {
        return bench_define_cache[define];
    }

    // lookup in our bench defines
    for (size_t i = 0; i < BENCH_DEFINE_MAP_COUNT; i++) {
        if (define < bench_define_maps[i].count
                && bench_define_maps[i].defines[define].cb) {
            intmax_t v = bench_define_maps[i].defines[define].cb(
                    bench_define_maps[i].defines[define].data);

            // insert into cache!
            bench_define_cache[define] = v;
            bench_define_cache_mask[define / (8*sizeof(unsigned))]
                    |= 1 << (define%(8*sizeof(unsigned)));

            return v;
        }
    }

    return 0;

    // not found?
    const char *name = bench_define_name(define);
    fprintf(stderr, "error: undefined define %s (%zd)\n",
            name ? name : "(unknown)",
            define);
    assert(false);
    exit(-1);
}

void bench_define_flush(void) {
    // clear cache between permutations
    memset(bench_define_cache_mask, 0,
            sizeof(unsigned)*(
                (bench_define_cache_count+(8*sizeof(unsigned))-1)
                / (8*sizeof(unsigned))));
}

// geometry updates
const bench_geometry_t *bench_geometry = NULL;

void bench_define_geometry(const bench_geometry_t *geometry) {
    bench_define_maps[BENCH_DEFINE_MAP_GEOMETRY] = (bench_define_map_t){
            geometry->defines, BENCH_GEOMETRY_DEFINE_COUNT};
}

// override updates
typedef struct bench_override {
    const char *name;
    const intmax_t *defines;
    size_t permutations;
} bench_override_t;

const bench_override_t *bench_overrides = NULL;
size_t bench_override_count = 0;

bench_define_t *bench_override_defines = NULL;
size_t bench_override_define_count = 0;
size_t bench_override_define_permutations = 1;
size_t bench_override_define_capacity = 0;

// suite/perm updates
void bench_define_suite(const struct bench_suite *suite) {
    bench_define_names[BENCH_DEFINE_NAMES_SUITE] = (bench_define_names_t){
            suite->define_names, suite->define_count};

    // make sure our cache is large enough
    if (lfs_max(suite->define_count, BENCH_IMPLICIT_DEFINE_COUNT)
            > bench_define_cache_count) {
        // align to power of two to avoid any superlinear growth
        size_t ncount = 1 << lfs_npw2(
                lfs_max(suite->define_count, BENCH_IMPLICIT_DEFINE_COUNT));
        bench_define_cache = realloc(bench_define_cache, ncount*sizeof(intmax_t));
        bench_define_cache_mask = realloc(bench_define_cache_mask,
                sizeof(unsigned)*(
                    (ncount+(8*sizeof(unsigned))-1)
                    / (8*sizeof(unsigned))));
        bench_define_cache_count = ncount;
    }

    // map any overrides
    if (bench_override_count > 0) {
        // first figure out the total size of override permutations
        size_t count = 0;
        size_t permutations = 1;
        for (size_t i = 0; i < bench_override_count; i++) {
            for (size_t d = 0;
                    d < lfs_max(
                        suite->define_count,
                        BENCH_IMPLICIT_DEFINE_COUNT);
                    d++) {
                // define name match?
                const char *name = bench_define_name(d);
                if (name && strcmp(name, bench_overrides[i].name) == 0) {
                    count = lfs_max(count, d+1);
                    permutations *= bench_overrides[i].permutations;
                    break;
                }
            }
        }
        bench_override_define_count = count;
        bench_override_define_permutations = permutations;

        // make sure our override arrays are big enough
        if (count * permutations > bench_override_define_capacity) {
            // align to power of two to avoid any superlinear growth
            size_t ncapacity = 1 << lfs_npw2(count * permutations);
            bench_override_defines = realloc(
                    bench_override_defines,
                    sizeof(bench_define_t)*ncapacity);
            bench_override_define_capacity = ncapacity;
        }

        // zero unoverridden defines
        memset(bench_override_defines, 0,
                sizeof(bench_define_t) * count * permutations);

        // compute permutations
        size_t p = 1;
        for (size_t i = 0; i < bench_override_count; i++) {
            for (size_t d = 0;
                    d < lfs_max(
                        suite->define_count,
                        BENCH_IMPLICIT_DEFINE_COUNT);
                    d++) {
                // define name match?
                const char *name = bench_define_name(d);
                if (name && strcmp(name, bench_overrides[i].name) == 0) {
                    // scatter the define permutations based on already
                    // seen permutations
                    for (size_t j = 0; j < permutations; j++) {
                        bench_override_defines[j*count + d] = BENCH_LIT(
                                bench_overrides[i].defines[(j/p)
                                    % bench_overrides[i].permutations]);
                    }

                    // keep track of how many permutations we've seen so far
                    p *= bench_overrides[i].permutations;
                    break;
                }
            }
        }
    }
}

void bench_define_perm(
        const struct bench_suite *suite,
        const struct bench_case *case_,
        size_t perm) {
    if (case_->defines) {
        bench_define_maps[BENCH_DEFINE_MAP_PERMUTATION] = (bench_define_map_t){
                case_->defines + perm*suite->define_count,
                suite->define_count};
    } else {
        bench_define_maps[BENCH_DEFINE_MAP_PERMUTATION] = (bench_define_map_t){
                NULL, 0};
    }
}

void bench_define_override(size_t perm) {
    bench_define_maps[BENCH_DEFINE_MAP_OVERRIDE] = (bench_define_map_t){
            bench_override_defines + perm*bench_override_define_count,
            bench_override_define_count};
}

void bench_define_explicit(
        const bench_define_t *defines,
        size_t define_count) {
    bench_define_maps[BENCH_DEFINE_MAP_EXPLICIT] = (bench_define_map_t){
            defines, define_count};
}

void bench_define_cleanup(void) {
    // bench define management can allocate a few things
    free(bench_define_cache);
    free(bench_define_cache_mask);
    free(bench_override_defines);
}



// bench state
extern const bench_geometry_t *bench_geometries;
extern size_t bench_geometry_count;

const bench_id_t *bench_ids = (const bench_id_t[]) {
    {NULL, NULL, 0},
};
size_t bench_id_count = 1;

size_t bench_step_start = 0;
size_t bench_step_stop = -1;
size_t bench_step_step = 1;

const char *bench_disk_path = NULL;
const char *bench_trace_path = NULL;
bool bench_trace_backtrace = false;
uint32_t bench_trace_period = 0;
uint32_t bench_trace_freq = 0;
FILE *bench_trace_file = NULL;
uint32_t bench_trace_cycles = 0;
uint64_t bench_trace_time = 0;
uint64_t bench_trace_open_time = 0;
lfs_emubd_sleep_t bench_read_sleep = 0.0;
lfs_emubd_sleep_t bench_prog_sleep = 0.0;
lfs_emubd_sleep_t bench_erase_sleep = 0.0;

// this determines both the backtrace buffer and the trace printf buffer, if
// trace ends up interleaved or truncated this may need to be increased
#ifndef BENCH_TRACE_BACKTRACE_BUFFER_SIZE
#define BENCH_TRACE_BACKTRACE_BUFFER_SIZE 8192
#endif
void *bench_trace_backtrace_buffer[
    BENCH_TRACE_BACKTRACE_BUFFER_SIZE / sizeof(void*)];

// trace printing
void bench_trace(const char *fmt, ...) {
    if (bench_trace_path) {
        // sample at a specific period?
        if (bench_trace_period) {
            if (bench_trace_cycles % bench_trace_period != 0) {
                bench_trace_cycles += 1;
                return;
            }
            bench_trace_cycles += 1;
        }

        // sample at a specific frequency?
        if (bench_trace_freq) {
            struct timespec t;
            clock_gettime(CLOCK_MONOTONIC, &t);
            uint64_t now = (uint64_t)t.tv_sec*1000*1000*1000
                    + (uint64_t)t.tv_nsec;
            if (now - bench_trace_time < (1000*1000*1000) / bench_trace_freq) {
                return;
            }
            bench_trace_time = now;
        }

        if (!bench_trace_file) {
            // Tracing output is heavy and trying to open every trace
            // call is slow, so we only try to open the trace file every
            // so often. Note this doesn't affect successfully opened files
            struct timespec t;
            clock_gettime(CLOCK_MONOTONIC, &t);
            uint64_t now = (uint64_t)t.tv_sec*1000*1000*1000
                    + (uint64_t)t.tv_nsec;
            if (now - bench_trace_open_time < 100*1000*1000) {
                return;
            }
            bench_trace_open_time = now;

            // try to open the trace file
            int fd;
            if (strcmp(bench_trace_path, "-") == 0) {
                fd = dup(1);
                if (fd < 0) {
                    return;
                }
            } else {
                fd = open(
                        bench_trace_path,
                        O_WRONLY | O_CREAT | O_APPEND | O_NONBLOCK,
                        0666);
                if (fd < 0) {
                    return;
                }
                int err = fcntl(fd, F_SETFL, O_WRONLY | O_CREAT | O_APPEND);
                assert(!err);
            }

            FILE *f = fdopen(fd, "a");
            assert(f);
            int err = setvbuf(f, NULL, _IOFBF,
                    BENCH_TRACE_BACKTRACE_BUFFER_SIZE);
            assert(!err);
            bench_trace_file = f;
        }

        // print trace
        va_list va;
        va_start(va, fmt);
        int res = vfprintf(bench_trace_file, fmt, va);
        va_end(va);
        if (res < 0) {
            fclose(bench_trace_file);
            bench_trace_file = NULL;
            return;
        }

        if (bench_trace_backtrace) {
            // print backtrace
            size_t count = backtrace(
                    bench_trace_backtrace_buffer,
                    BENCH_TRACE_BACKTRACE_BUFFER_SIZE);
            // note we skip our own stack frame
            for (size_t i = 1; i < count; i++) {
                res = fprintf(bench_trace_file, "\tat %p\n",
                        bench_trace_backtrace_buffer[i]);
                if (res < 0) {
                    fclose(bench_trace_file);
                    bench_trace_file = NULL;
                    return;
                }
            }
        }

        // flush immediately
        fflush(bench_trace_file);
    }
}


// bench prng
uint32_t bench_prng(uint32_t *state) {
    // A simple xorshift32 generator, easily reproducible. Keep in mind
    // determinism is much more important than actual randomness here.
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}


// bench recording state
static struct lfs_config *bench_cfg = NULL;
static lfs_emubd_io_t bench_last_readed = 0;
static lfs_emubd_io_t bench_last_proged = 0;
static lfs_emubd_io_t bench_last_erased = 0;
lfs_emubd_io_t bench_readed = 0;
lfs_emubd_io_t bench_proged = 0;
lfs_emubd_io_t bench_erased = 0;

void bench_reset(void) {
    bench_readed = 0;
    bench_proged = 0;
    bench_erased = 0;
    bench_last_readed = 0;
    bench_last_proged = 0;
    bench_last_erased = 0;
}

void bench_start(void) {
    assert(bench_cfg);
    lfs_emubd_sio_t readed = lfs_emubd_readed(bench_cfg);
    assert(readed >= 0);
    lfs_emubd_sio_t proged = lfs_emubd_proged(bench_cfg);
    assert(proged >= 0);
    lfs_emubd_sio_t erased = lfs_emubd_erased(bench_cfg);
    assert(erased >= 0);

    bench_last_readed = readed;
    bench_last_proged = proged;
    bench_last_erased = erased;
}

void bench_stop(void) {
    assert(bench_cfg);
    lfs_emubd_sio_t readed = lfs_emubd_readed(bench_cfg);
    assert(readed >= 0);
    lfs_emubd_sio_t proged = lfs_emubd_proged(bench_cfg);
    assert(proged >= 0);
    lfs_emubd_sio_t erased = lfs_emubd_erased(bench_cfg);
    assert(erased >= 0);

    bench_readed += readed - bench_last_readed;
    bench_proged += proged - bench_last_proged;
    bench_erased += erased - bench_last_erased;
}


// encode our permutation into a reusable id
static void perm_printid(
        const struct bench_suite *suite,
        const struct bench_case *case_) {
    (void)suite;
    // case[:permutation]
    printf("%s:", case_->name);
    for (size_t d = 0;
            d < lfs_max(
                suite->define_count,
                BENCH_IMPLICIT_DEFINE_COUNT);
            d++) {
        if (bench_define_ispermutation(d)) {
            leb16_print(d);
            leb16_print(BENCH_DEFINE(d));
        }
    }
}

// a quick trie for keeping track of permutations we've seen
typedef struct bench_seen {
    struct bench_seen_branch *branches;
    size_t branch_count;
    size_t branch_capacity;
} bench_seen_t;

struct bench_seen_branch {
    intmax_t define;
    struct bench_seen branch;
};

bool bench_seen_insert(
        bench_seen_t *seen,
        const struct bench_suite *suite,
        const struct bench_case *case_) {
    (void)case_;
    bool was_seen = true;

    // use the currently set defines
    for (size_t d = 0;
            d < lfs_max(
                suite->define_count,
                BENCH_IMPLICIT_DEFINE_COUNT);
            d++) {
        // treat unpermuted defines the same as 0
        intmax_t define = bench_define_ispermutation(d) ? BENCH_DEFINE(d) : 0;

        // already seen?
        struct bench_seen_branch *branch = NULL;
        for (size_t i = 0; i < seen->branch_count; i++) {
            if (seen->branches[i].define == define) {
                branch = &seen->branches[i];
                break;
            }
        }

        // need to create a new node
        if (!branch) {
            was_seen = false;
            branch = mappend(
                    (void**)&seen->branches,
                    sizeof(struct bench_seen_branch),
                    &seen->branch_count,
                    &seen->branch_capacity);
            branch->define = define;
            branch->branch = (bench_seen_t){NULL, 0, 0};
        }

        seen = &branch->branch;
    }

    return was_seen;
}

void bench_seen_cleanup(bench_seen_t *seen) {
    for (size_t i = 0; i < seen->branch_count; i++) {
        bench_seen_cleanup(&seen->branches[i].branch);
    }
    free(seen->branches);
}

// iterate through permutations in a bench case
static void case_forperm(
        const struct bench_suite *suite,
        const struct bench_case *case_,
        const bench_define_t *defines,
        size_t define_count,
        void (*cb)(
            void *data,
            const struct bench_suite *suite,
            const struct bench_case *case_),
        void *data) {
    // explicit permutation?
    if (defines) {
        bench_define_explicit(defines, define_count);

        for (size_t v = 0; v < bench_override_define_permutations; v++) {
            // define override permutation
            bench_define_override(v);
            bench_define_flush();

            cb(data, suite, case_);
        }

        return;
    }

    bench_seen_t seen = {NULL, 0, 0};

    for (size_t k = 0; k < case_->permutations; k++) {
        // define permutation
        bench_define_perm(suite, case_, k);

        for (size_t v = 0; v < bench_override_define_permutations; v++) {
            // define override permutation
            bench_define_override(v);

            for (size_t g = 0; g < bench_geometry_count; g++) {
                // define geometry
                bench_define_geometry(&bench_geometries[g]);
                bench_define_flush();

                // have we seen this permutation before?
                bool was_seen = bench_seen_insert(&seen, suite, case_);
                if (!(k == 0 && v == 0 && g == 0) && was_seen) {
                    continue;
                }

                cb(data, suite, case_);
            }
        }
    }

    bench_seen_cleanup(&seen);
}


// how many permutations are there actually in a bench case
struct perm_count_state {
    size_t total;
    size_t filtered;
};

void perm_count(
        void *data,
        const struct bench_suite *suite,
        const struct bench_case *case_) {
    struct perm_count_state *state = data;
    (void)suite;
    (void)case_;

    state->total += 1;

    if (case_->filter && !case_->filter()) {
        return;
    }

    state->filtered += 1;
}


// operations we can do
static void summary(void) {
    printf("%-23s  %7s %7s %7s %11s\n",
            "", "flags", "suites", "cases", "perms");
    size_t suites = 0;
    size_t cases = 0;
    bench_flags_t flags = 0;
    struct perm_count_state perms = {0, 0};

    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            bench_define_suite(&bench_suites[i]);

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                cases += 1;
                case_forperm(
                        &bench_suites[i],
                        &bench_suites[i].cases[j],
                        bench_ids[t].defines,
                        bench_ids[t].define_count,
                        perm_count,
                        &perms);
            }

            suites += 1;
            flags |= bench_suites[i].flags;
        }
    }

    char perm_buf[64];
    sprintf(perm_buf, "%zu/%zu", perms.filtered, perms.total);
    char flag_buf[64];
    sprintf(flag_buf, "%s%s",
            (flags & BENCH_REENTRANT) ? "r" : "",
            (!flags) ? "-" : "");
    printf("%-23s  %7s %7zu %7zu %11s\n",
            "TOTAL",
            flag_buf,
            suites,
            cases,
            perm_buf);
}

static void list_suites(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
        size_t len = strlen(bench_suites[i].name);
        if (len > name_width) {
            name_width = len;
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %7s %7s %11s\n",
            name_width, "suite", "flags", "cases", "perms");
    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            bench_define_suite(&bench_suites[i]);

            size_t cases = 0;
            struct perm_count_state perms = {0, 0};

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                cases += 1;
                case_forperm(
                        &bench_suites[i],
                        &bench_suites[i].cases[j],
                        bench_ids[t].defines,
                        bench_ids[t].define_count,
                        perm_count,
                        &perms);
            }

            // no benches found?
            if (!cases) {
                continue;
            }

            char perm_buf[64];
            sprintf(perm_buf, "%zu/%zu", perms.filtered, perms.total);
            char flag_buf[64];
            sprintf(flag_buf, "%s%s",
                    (bench_suites[i].flags & BENCH_REENTRANT) ? "r" : "",
                    (!bench_suites[i].flags) ? "-" : "");
            printf("%-*s  %7s %7zu %11s\n",
                    name_width,
                    bench_suites[i].name,
                    flag_buf,
                    cases,
                    perm_buf);
        }
    }
}

static void list_cases(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
        for (size_t j = 0; j < bench_suites[i].case_count; j++) {
            size_t len = strlen(bench_suites[i].cases[j].name);
            if (len > name_width) {
                name_width = len;
            }
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %7s %11s\n", name_width, "case", "flags", "perms");
    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            bench_define_suite(&bench_suites[i]);

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                struct perm_count_state perms = {0, 0};
                case_forperm(
                        &bench_suites[i],
                        &bench_suites[i].cases[j],
                        bench_ids[t].defines,
                        bench_ids[t].define_count,
                        perm_count,
                        &perms);

                char perm_buf[64];
                sprintf(perm_buf, "%zu/%zu", perms.filtered, perms.total);
                char flag_buf[64];
                sprintf(flag_buf, "%s%s",
                        (bench_suites[i].cases[j].flags & BENCH_REENTRANT)
                            ? "r" : "",
                        (!bench_suites[i].cases[j].flags)
                            ? "-" : "");
                printf("%-*s  %7s %11s\n",
                        name_width,
                        bench_suites[i].cases[j].name,
                        flag_buf,
                        perm_buf);
            }
        }
    }
}

static void list_suite_paths(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
        size_t len = strlen(bench_suites[i].name);
        if (len > name_width) {
            name_width = len;
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %s\n", name_width, "suite", "path");
    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            size_t cases = 0;

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;

                    cases += 1;
                }
            }

            // no benches found?
            if (!cases) {
                continue;
            }

            printf("%-*s  %s\n",
                    name_width,
                    bench_suites[i].name,
                    bench_suites[i].path);
        }
    }
}

static void list_case_paths(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
        for (size_t j = 0; j < bench_suites[i].case_count; j++) {
            size_t len = strlen(bench_suites[i].cases[j].name);
            if (len > name_width) {
                name_width = len;
            }
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %s\n", name_width, "case", "path");
    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                printf("%-*s  %s\n",
                        name_width,
                        bench_suites[i].cases[j].name,
                        bench_suites[i].cases[j].path);
            }
        }
    }
}

struct list_defines_define {
    const char *name;
    intmax_t *values;
    size_t value_count;
    size_t value_capacity;
};

struct list_defines_defines {
    struct list_defines_define *defines;
    size_t define_count;
    size_t define_capacity;
};

static void list_defines_add(
        struct list_defines_defines *defines,
        size_t d) {
    const char *name = bench_define_name(d);
    intmax_t value = BENCH_DEFINE(d);

    // define already in defines?
    for (size_t i = 0; i < defines->define_count; i++) {
        if (strcmp(defines->defines[i].name, name) == 0) {
            // value already in values?
            for (size_t j = 0; j < defines->defines[i].value_count; j++) {
                if (defines->defines[i].values[j] == value) {
                    return;
                }
            }

            *(intmax_t*)mappend(
                (void**)&defines->defines[i].values,
                sizeof(intmax_t),
                &defines->defines[i].value_count,
                &defines->defines[i].value_capacity) = value;

            return;
        }
    }

    // new define?
    struct list_defines_define *define = mappend(
            (void**)&defines->defines,
            sizeof(struct list_defines_define),
            &defines->define_count,
            &defines->define_capacity);
    define->name = name;
    define->values = malloc(sizeof(intmax_t));
    define->values[0] = value;
    define->value_count = 1;
    define->value_capacity = 1;
}

void perm_list_defines(
        void *data,
        const struct bench_suite *suite,
        const struct bench_case *case_) {
    struct list_defines_defines *defines = data;
    (void)suite;
    (void)case_;

    // collect defines
    for (size_t d = 0;
            d < lfs_max(suite->define_count,
                BENCH_IMPLICIT_DEFINE_COUNT);
            d++) {
        if (d < BENCH_IMPLICIT_DEFINE_COUNT
                || bench_define_ispermutation(d)) {
            list_defines_add(defines, d);
        }
    }
}

void perm_list_permutation_defines(
        void *data,
        const struct bench_suite *suite,
        const struct bench_case *case_) {
    struct list_defines_defines *defines = data;
    (void)suite;
    (void)case_;

    // collect permutation_defines
    for (size_t d = 0;
            d < lfs_max(suite->define_count,
                BENCH_IMPLICIT_DEFINE_COUNT);
            d++) {
        if (bench_define_ispermutation(d)) {
            list_defines_add(defines, d);
        }
    }
}

extern const bench_geometry_t builtin_geometries[];

static void list_defines(void) {
    struct list_defines_defines defines = {NULL, 0, 0};

    // add defines
    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            bench_define_suite(&bench_suites[i]);

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                case_forperm(
                        &bench_suites[i],
                        &bench_suites[i].cases[j],
                        bench_ids[t].defines,
                        bench_ids[t].define_count,
                        perm_list_defines,
                        &defines);
            }
        }
    }

    for (size_t i = 0; i < defines.define_count; i++) {
        printf("%s=", defines.defines[i].name);
        for (size_t j = 0; j < defines.defines[i].value_count; j++) {
            printf("%jd", defines.defines[i].values[j]);
            if (j != defines.defines[i].value_count-1) {
                printf(",");
            }
        }
        printf("\n");
    }

    for (size_t i = 0; i < defines.define_count; i++) {
        free(defines.defines[i].values);
    }
    free(defines.defines);
}

static void list_permutation_defines(void) {
    struct list_defines_defines defines = {NULL, 0, 0};

    // add permutation defines
    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            bench_define_suite(&bench_suites[i]);

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                case_forperm(
                        &bench_suites[i],
                        &bench_suites[i].cases[j],
                        bench_ids[t].defines,
                        bench_ids[t].define_count,
                        perm_list_permutation_defines,
                        &defines);
            }
        }
    }

    for (size_t i = 0; i < defines.define_count; i++) {
        printf("%s=", defines.defines[i].name);
        for (size_t j = 0; j < defines.defines[i].value_count; j++) {
            printf("%jd", defines.defines[i].values[j]);
            if (j != defines.defines[i].value_count-1) {
                printf(",");
            }
        }
        printf("\n");
    }

    for (size_t i = 0; i < defines.define_count; i++) {
        free(defines.defines[i].values);
    }
    free(defines.defines);
}

static void list_implicit_defines(void) {
    struct list_defines_defines defines = {NULL, 0, 0};

    // yes we do need to define a suite, this does a bit of bookeeping
    // such as setting up the define cache
    bench_define_suite(&(const struct bench_suite){0});

    // make sure to include builtin geometries here
    extern const bench_geometry_t builtin_geometries[];
    for (size_t g = 0; builtin_geometries[g].name; g++) {
        bench_define_geometry(&builtin_geometries[g]);
        bench_define_flush();

        // add implicit defines
        for (size_t d = 0; d < BENCH_IMPLICIT_DEFINE_COUNT; d++) {
            list_defines_add(&defines, d);
        }
    }

    for (size_t i = 0; i < defines.define_count; i++) {
        printf("%s=", defines.defines[i].name);
        for (size_t j = 0; j < defines.defines[i].value_count; j++) {
            printf("%jd", defines.defines[i].values[j]);
            if (j != defines.defines[i].value_count-1) {
                printf(",");
            }
        }
        printf("\n");
    }

    for (size_t i = 0; i < defines.define_count; i++) {
        free(defines.defines[i].values);
    }
    free(defines.defines);
}



// geometries to bench

const bench_geometry_t builtin_geometries[] = {
    {"default", {{0}, BENCH_CONST(16),   BENCH_CONST(512),   {0}}},
    {"eeprom",  {{0}, BENCH_CONST(1),    BENCH_CONST(512),   {0}}},
    {"emmc",    {{0}, {0},               BENCH_CONST(512),   {0}}},
    {"nor",     {{0}, BENCH_CONST(1),    BENCH_CONST(4096),  {0}}},
    {"nand",    {{0}, BENCH_CONST(4096), BENCH_CONST(32768), {0}}},
    {NULL, {{0}, {0}, {0}, {0}}},
};

const bench_geometry_t *bench_geometries = builtin_geometries;
size_t bench_geometry_count = 5;

static void list_geometries(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t g = 0; builtin_geometries[g].name; g++) {
        size_t len = strlen(builtin_geometries[g].name);
        if (len > name_width) {
            name_width = len;
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    // yes we do need to define a suite, this does a bit of bookeeping
    // such as setting up the define cache
    bench_define_suite(&(const struct bench_suite){0});

    printf("%-*s  %7s %7s %7s %7s %11s\n",
            name_width, "geometry", "read", "prog", "erase", "count", "size");
    for (size_t g = 0; builtin_geometries[g].name; g++) {
        bench_define_geometry(&builtin_geometries[g]);
        bench_define_flush();
        printf("%-*s  %7ju %7ju %7ju %7ju %11ju\n",
                name_width,
                builtin_geometries[g].name,
                READ_SIZE,
                PROG_SIZE,
                ERASE_SIZE,
                ERASE_COUNT,
                ERASE_SIZE*ERASE_COUNT);
    }
}



// global bench step count
size_t bench_step = 0;

void perm_run(
        void *data,
        const struct bench_suite *suite,
        const struct bench_case *case_) {
    (void)data;

    // skip this step?
    if (!(bench_step >= bench_step_start
            && bench_step < bench_step_stop
            && (bench_step-bench_step_start) % bench_step_step == 0)) {
        bench_step += 1;
        return;
    }
    bench_step += 1;

    // filter?
    if (case_->filter && !case_->filter()) {
        printf("skipped ");
        perm_printid(suite, case_);
        printf("\n");
        return;
    }

    // create block device and configuration
    lfs_emubd_t bd;

    struct lfs_config cfg = {
        .context            = &bd,
        .read               = lfs_emubd_read,
        .prog               = lfs_emubd_prog,
        .erase              = lfs_emubd_erase,
        .sync               = lfs_emubd_sync,
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .block_size         = BLOCK_SIZE,
        .block_count        = BLOCK_COUNT,
        .block_cycles       = BLOCK_CYCLES,
        .cache_size         = CACHE_SIZE,
        .lookahead_size     = LOOKAHEAD_SIZE,
        .compact_thresh     = COMPACT_THRESH,
        .metadata_max       = METADATA_MAX,
        .inline_max         = INLINE_MAX,
    };

    struct lfs_emubd_config bdcfg = {
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .erase_size         = ERASE_SIZE,
        .erase_count        = ERASE_COUNT,
        .erase_value        = ERASE_VALUE,
        .erase_cycles       = ERASE_CYCLES,
        .badblock_behavior  = BADBLOCK_BEHAVIOR,
        .disk_path          = bench_disk_path,
        .read_sleep         = bench_read_sleep,
        .prog_sleep         = bench_prog_sleep,
        .erase_sleep        = bench_erase_sleep,
    };

    int err = lfs_emubd_create(&cfg, &bdcfg);
    if (err) {
        fprintf(stderr, "error: could not create block device: %d\n", err);
        exit(-1);
    }

    // run the bench
    bench_cfg = &cfg;
    bench_reset();
    printf("running ");
    perm_printid(suite, case_);
    printf("\n");

    case_->run(&cfg);

    printf("finished ");
    perm_printid(suite, case_);
    printf(" %"PRIu64" %"PRIu64" %"PRIu64,
        bench_readed,
        bench_proged,
        bench_erased);
    printf("\n");

    // cleanup
    err = lfs_emubd_destroy(&cfg);
    if (err) {
        fprintf(stderr, "error: could not destroy block device: %d\n", err);
        exit(-1);
    }
}

static void run(void) {
    // ignore disconnected pipes
    signal(SIGPIPE, SIG_IGN);

    for (size_t t = 0; t < bench_id_count; t++) {
        for (size_t i = 0; i < BENCH_SUITE_COUNT; i++) {
            bench_define_suite(&bench_suites[i]);

            for (size_t j = 0; j < bench_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (bench_ids[t].name && !(
                        strcmp(bench_ids[t].name,
                            bench_suites[i].name) == 0
                        || strcmp(bench_ids[t].name,
                            bench_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                case_forperm(
                        &bench_suites[i],
                        &bench_suites[i].cases[j],
                        bench_ids[t].defines,
                        bench_ids[t].define_count,
                        perm_run,
                        NULL);
            }
        }
    }
}



// option handling
enum opt_flags {
    OPT_HELP                     = 'h',
    OPT_SUMMARY                  = 'Y',
    OPT_LIST_SUITES              = 'l',
    OPT_LIST_CASES               = 'L',
    OPT_LIST_SUITE_PATHS         = 1,
    OPT_LIST_CASE_PATHS          = 2,
    OPT_LIST_DEFINES             = 3,
    OPT_LIST_PERMUTATION_DEFINES = 4,
    OPT_LIST_IMPLICIT_DEFINES    = 5,
    OPT_LIST_GEOMETRIES          = 6,
    OPT_DEFINE                   = 'D',
    OPT_GEOMETRY                 = 'G',
    OPT_STEP                     = 's',
    OPT_DISK                     = 'd',
    OPT_TRACE                    = 't',
    OPT_TRACE_BACKTRACE          = 7,
    OPT_TRACE_PERIOD             = 8,
    OPT_TRACE_FREQ               = 9,
    OPT_READ_SLEEP               = 10,
    OPT_PROG_SLEEP               = 11,
    OPT_ERASE_SLEEP              = 12,
};

const char *short_opts = "hYlLD:G:s:d:t:";

const struct option long_opts[] = {
    {"help",             no_argument,       NULL, OPT_HELP},
    {"summary",          no_argument,       NULL, OPT_SUMMARY},
    {"list-suites",      no_argument,       NULL, OPT_LIST_SUITES},
    {"list-cases",       no_argument,       NULL, OPT_LIST_CASES},
    {"list-suite-paths", no_argument,       NULL, OPT_LIST_SUITE_PATHS},
    {"list-case-paths",  no_argument,       NULL, OPT_LIST_CASE_PATHS},
    {"list-defines",     no_argument,       NULL, OPT_LIST_DEFINES},
    {"list-permutation-defines",
                         no_argument,       NULL, OPT_LIST_PERMUTATION_DEFINES},
    {"list-implicit-defines",
                         no_argument,       NULL, OPT_LIST_IMPLICIT_DEFINES},
    {"list-geometries",  no_argument,       NULL, OPT_LIST_GEOMETRIES},
    {"define",           required_argument, NULL, OPT_DEFINE},
    {"geometry",         required_argument, NULL, OPT_GEOMETRY},
    {"step",             required_argument, NULL, OPT_STEP},
    {"disk",             required_argument, NULL, OPT_DISK},
    {"trace",            required_argument, NULL, OPT_TRACE},
    {"trace-backtrace",  no_argument,       NULL, OPT_TRACE_BACKTRACE},
    {"trace-period",     required_argument, NULL, OPT_TRACE_PERIOD},
    {"trace-freq",       required_argument, NULL, OPT_TRACE_FREQ},
    {"read-sleep",       required_argument, NULL, OPT_READ_SLEEP},
    {"prog-sleep",       required_argument, NULL, OPT_PROG_SLEEP},
    {"erase-sleep",      required_argument, NULL, OPT_ERASE_SLEEP},
    {NULL, 0, NULL, 0},
};

const char *const help_text[] = {
    "Show this help message.",
    "Show quick summary.",
    "List bench suites.",
    "List bench cases.",
    "List the path for each bench suite.",
    "List the path and line number for each bench case.",
    "List all defines in this bench-runner.",
    "List explicit defines in this bench-runner.",
    "List implicit defines in this bench-runner.",
    "List the available disk geometries.",
    "Override a bench define.",
    "Comma-separated list of disk geometries to bench.",
    "Comma-separated range of bench permutations to run (start,stop,step).",
    "Direct block device operations to this file.",
    "Direct trace output to this file.",
    "Include a backtrace with every trace statement.",
    "Sample trace output at this period in cycles.",
    "Sample trace output at this frequency in hz.",
    "Artificial read delay in seconds.",
    "Artificial prog delay in seconds.",
    "Artificial erase delay in seconds.",
};

int main(int argc, char **argv) {
    void (*op)(void) = run;

    size_t bench_override_capacity = 0;
    size_t bench_geometry_capacity = 0;
    size_t bench_id_capacity = 0;

    // parse options
    while (true) {
        int c = getopt_long(argc, argv, short_opts, long_opts, NULL);
        switch (c) {
            // generate help message
            case OPT_HELP: {
                printf("usage: %s [options] [bench_id]\n", argv[0]);
                printf("\n");

                printf("options:\n");
                size_t i = 0;
                while (long_opts[i].name) {
                    size_t indent;
                    if (long_opts[i].has_arg == no_argument) {
                        if (long_opts[i].val >= '0' && long_opts[i].val < 'z') {
                            indent = printf("  -%c, --%s ",
                                    long_opts[i].val,
                                    long_opts[i].name);
                        } else {
                            indent = printf("  --%s ",
                                    long_opts[i].name);
                        }
                    } else {
                        if (long_opts[i].val >= '0' && long_opts[i].val < 'z') {
                            indent = printf("  -%c %s, --%s %s ",
                                    long_opts[i].val,
                                    long_opts[i].name,
                                    long_opts[i].name,
                                    long_opts[i].name);
                        } else {
                            indent = printf("  --%s %s ",
                                    long_opts[i].name,
                                    long_opts[i].name);
                        }
                    }

                    // a quick, hacky, byte-level method for text wrapping
                    size_t len = strlen(help_text[i]);
                    size_t j = 0;
                    if (indent < 24) {
                        printf("%*s %.80s\n",
                                (int)(24-1-indent),
                                "",
                                &help_text[i][j]);
                        j += 80;
                    } else {
                        printf("\n");
                    }

                    while (j < len) {
                        printf("%24s%.80s\n", "", &help_text[i][j]);
                        j += 80;
                    }

                    i += 1;
                }

                printf("\n");
                exit(0);
            }
            // summary/list flags
            case OPT_SUMMARY:
                op = summary;
                break;
            case OPT_LIST_SUITES:
                op = list_suites;
                break;
            case OPT_LIST_CASES:
                op = list_cases;
                break;
            case OPT_LIST_SUITE_PATHS:
                op = list_suite_paths;
                break;
            case OPT_LIST_CASE_PATHS:
                op = list_case_paths;
                break;
            case OPT_LIST_DEFINES:
                op = list_defines;
                break;
            case OPT_LIST_PERMUTATION_DEFINES:
                op = list_permutation_defines;
                break;
            case OPT_LIST_IMPLICIT_DEFINES:
                op = list_implicit_defines;
                break;
            case OPT_LIST_GEOMETRIES:
                op = list_geometries;
                break;
            // configuration
            case OPT_DEFINE: {
                // allocate space
                bench_override_t *override = mappend(
                        (void**)&bench_overrides,
                        sizeof(bench_override_t),
                        &bench_override_count,
                        &bench_override_capacity);

                // parse into string key/intmax_t value, cannibalizing the
                // arg in the process
                char *sep = strchr(optarg, '=');
                char *parsed = NULL;
                if (!sep) {
                    goto invalid_define;
                }
                *sep = '\0';
                override->name = optarg;
                optarg = sep+1;

                // parse comma-separated permutations
                {
                    override->defines = NULL;
                    override->permutations = 0;
                    size_t override_capacity = 0;
                    while (true) {
                        optarg += strspn(optarg, " ");

                        if (strncmp(optarg, "range", strlen("range")) == 0) {
                            // range of values
                            optarg += strlen("range");
                            optarg += strspn(optarg, " ");
                            if (*optarg != '(') {
                                goto invalid_define;
                            }
                            optarg += 1;

                            intmax_t start = strtoumax(optarg, &parsed, 0);
                            intmax_t stop = -1;
                            intmax_t step = 1;
                            // allow empty string for start=0
                            if (parsed == optarg) {
                                start = 0;
                            }
                            optarg = parsed + strspn(parsed, " ");

                            if (*optarg != ',' && *optarg != ')') {
                                goto invalid_define;
                            }

                            if (*optarg == ',') {
                                optarg += 1;
                                stop = strtoumax(optarg, &parsed, 0);
                                // allow empty string for stop=end
                                if (parsed == optarg) {
                                    stop = -1;
                                }
                                optarg = parsed + strspn(parsed, " ");

                                if (*optarg != ',' && *optarg != ')') {
                                    goto invalid_define;
                                }

                                if (*optarg == ',') {
                                    optarg += 1;
                                    step = strtoumax(optarg, &parsed, 0);
                                    // allow empty string for stop=1
                                    if (parsed == optarg) {
                                        step = 1;
                                    }
                                    optarg = parsed + strspn(parsed, " ");

                                    if (*optarg != ')') {
                                        goto invalid_define;
                                    }
                                }
                            } else {
                                // single value = stop only
                                stop = start;
                                start = 0;
                            }

                            if (*optarg != ')') {
                                goto invalid_define;
                            }
                            optarg += 1;

                            // calculate the range of values
                            assert(step != 0);
                            for (intmax_t i = start;
                                    (step < 0)
                                        ? i > stop
                                        : (uintmax_t)i < (uintmax_t)stop;
                                    i += step) {
                                *(intmax_t*)mappend(
                                        (void**)&override->defines,
                                        sizeof(intmax_t),
                                        &override->permutations,
                                        &override_capacity) = i;
                            }
                        } else if (*optarg != '\0') {
                            // single value
                            intmax_t define = strtoimax(optarg, &parsed, 0);
                            if (parsed == optarg) {
                                goto invalid_define;
                            }
                            optarg = parsed + strspn(parsed, " ");
                            *(intmax_t*)mappend(
                                    (void**)&override->defines,
                                    sizeof(intmax_t),
                                    &override->permutations,
                                    &override_capacity) = define;
                        } else {
                            break;
                        }

                        if (*optarg == ',') {
                            optarg += 1;
                        }
                    }
                }
                assert(override->permutations > 0);
                break;

invalid_define:
                fprintf(stderr, "error: invalid define: %s\n", optarg);
                exit(-1);
            }
            case OPT_GEOMETRY: {
                // reset our geometry scenarios
                if (bench_geometry_capacity > 0) {
                    free((bench_geometry_t*)bench_geometries);
                }
                bench_geometries = NULL;
                bench_geometry_count = 0;
                bench_geometry_capacity = 0;

                // parse the comma separated list of disk geometries
                while (*optarg) {
                    // allocate space
                    bench_geometry_t *geometry = mappend(
                            (void**)&bench_geometries,
                            sizeof(bench_geometry_t),
                            &bench_geometry_count,
                            &bench_geometry_capacity);

                    // parse the disk geometry
                    optarg += strspn(optarg, " ");

                    // named disk geometry
                    size_t len = strcspn(optarg, " ,");
                    for (size_t i = 0; builtin_geometries[i].name; i++) {
                        if (len == strlen(builtin_geometries[i].name)
                                && memcmp(optarg,
                                    builtin_geometries[i].name,
                                    len) == 0) {
                            *geometry = builtin_geometries[i];
                            optarg += len;
                            goto geometry_next;
                        }
                    }

                    // comma-separated read/prog/erase/count
                    if (*optarg == '{') {
                        lfs_size_t sizes[4];
                        size_t count = 0;

                        char *s = optarg + 1;
                        while (count < 4) {
                            char *parsed = NULL;
                            sizes[count] = strtoumax(s, &parsed, 0);
                            count += 1;

                            s = parsed + strspn(parsed, " ");
                            if (*s == ',') {
                                s += 1;
                                continue;
                            } else if (*s == '}') {
                                s += 1;
                                break;
                            } else {
                                goto geometry_unknown;
                            }
                        }

                        // allow implicit r=p and p=e for common geometries
                        memset(geometry, 0, sizeof(bench_geometry_t));
                        if (count >= 3) {
                            geometry->defines[READ_SIZE_i]
                                    = BENCH_LIT(sizes[0]);
                            geometry->defines[PROG_SIZE_i]
                                    = BENCH_LIT(sizes[1]);
                            geometry->defines[ERASE_SIZE_i]
                                    = BENCH_LIT(sizes[2]);
                        } else if (count >= 2) {
                            geometry->defines[PROG_SIZE_i]
                                    = BENCH_LIT(sizes[0]);
                            geometry->defines[ERASE_SIZE_i]
                                    = BENCH_LIT(sizes[1]);
                        } else {
                            geometry->defines[ERASE_SIZE_i]
                                    = BENCH_LIT(sizes[0]);
                        }
                        if (count >= 4) {
                            geometry->defines[ERASE_COUNT_i]
                                    = BENCH_LIT(sizes[3]);
                        }
                        optarg = s;
                        goto geometry_next;
                    }

                    // leb16-encoded read/prog/erase/count
                    if (*optarg == ':') {
                        lfs_size_t sizes[4];
                        size_t count = 0;

                        char *s = optarg + 1;
                        while (true) {
                            char *parsed = NULL;
                            uintmax_t x = leb16_parse(s, &parsed);
                            if (parsed == s || count >= 4) {
                                break;
                            }

                            sizes[count] = x;
                            count += 1;
                            s = parsed;
                        }

                        // allow implicit r=p and p=e for common geometries
                        memset(geometry, 0, sizeof(bench_geometry_t));
                        if (count >= 3) {
                            geometry->defines[READ_SIZE_i]
                                    = BENCH_LIT(sizes[0]);
                            geometry->defines[PROG_SIZE_i]
                                    = BENCH_LIT(sizes[1]);
                            geometry->defines[ERASE_SIZE_i]
                                    = BENCH_LIT(sizes[2]);
                        } else if (count >= 2) {
                            geometry->defines[PROG_SIZE_i]
                                    = BENCH_LIT(sizes[0]);
                            geometry->defines[ERASE_SIZE_i]
                                    = BENCH_LIT(sizes[1]);
                        } else {
                            geometry->defines[ERASE_SIZE_i]
                                    = BENCH_LIT(sizes[0]);
                        }
                        if (count >= 4) {
                            geometry->defines[ERASE_COUNT_i]
                                    = BENCH_LIT(sizes[3]);
                        }
                        optarg = s;
                        goto geometry_next;
                    }

geometry_unknown:
                    // unknown scenario?
                    fprintf(stderr, "error: unknown disk geometry: %s\n",
                            optarg);
                    exit(-1);

geometry_next:
                    optarg += strspn(optarg, " ");
                    if (*optarg == ',') {
                        optarg += 1;
                    } else if (*optarg == '\0') {
                        break;
                    } else {
                        goto geometry_unknown;
                    }
                }
                break;
            }
            case OPT_STEP: {
                char *parsed = NULL;
                bench_step_start = strtoumax(optarg, &parsed, 0);
                bench_step_stop = -1;
                bench_step_step = 1;
                // allow empty string for start=0
                if (parsed == optarg) {
                    bench_step_start = 0;
                }
                optarg = parsed + strspn(parsed, " ");

                if (*optarg != ',' && *optarg != '\0') {
                    goto step_unknown;
                }

                if (*optarg == ',') {
                    optarg += 1;
                    bench_step_stop = strtoumax(optarg, &parsed, 0);
                    // allow empty string for stop=end
                    if (parsed == optarg) {
                        bench_step_stop = -1;
                    }
                    optarg = parsed + strspn(parsed, " ");

                    if (*optarg != ',' && *optarg != '\0') {
                        goto step_unknown;
                    }

                    if (*optarg == ',') {
                        optarg += 1;
                        bench_step_step = strtoumax(optarg, &parsed, 0);
                        // allow empty string for stop=1
                        if (parsed == optarg) {
                            bench_step_step = 1;
                        }
                        optarg = parsed + strspn(parsed, " ");

                        if (*optarg != '\0') {
                            goto step_unknown;
                        }
                    }
                } else {
                    // single value = stop only
                    bench_step_stop = bench_step_start;
                    bench_step_start = 0;
                }

                break;
step_unknown:
                fprintf(stderr, "error: invalid step: %s\n", optarg);
                exit(-1);
            }
            case OPT_DISK:
                bench_disk_path = optarg;
                break;
            case OPT_TRACE:
                bench_trace_path = optarg;
                break;
            case OPT_TRACE_BACKTRACE:
                bench_trace_backtrace = true;
                break;
            case OPT_TRACE_PERIOD: {
                char *parsed = NULL;
                bench_trace_period = strtoumax(optarg, &parsed, 0);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid trace-period: %s\n", optarg);
                    exit(-1);
                }
                break;
            }
            case OPT_TRACE_FREQ: {
                char *parsed = NULL;
                bench_trace_freq = strtoumax(optarg, &parsed, 0);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid trace-freq: %s\n", optarg);
                    exit(-1);
                }
                break;
            }
            case OPT_READ_SLEEP: {
                char *parsed = NULL;
                double read_sleep = strtod(optarg, &parsed);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid read-sleep: %s\n", optarg);
                    exit(-1);
                }
                bench_read_sleep = read_sleep*1.0e9;
                break;
            }
            case OPT_PROG_SLEEP: {
                char *parsed = NULL;
                double prog_sleep = strtod(optarg, &parsed);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid prog-sleep: %s\n", optarg);
                    exit(-1);
                }
                bench_prog_sleep = prog_sleep*1.0e9;
                break;
            }
            case OPT_ERASE_SLEEP: {
                char *parsed = NULL;
                double erase_sleep = strtod(optarg, &parsed);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid erase-sleep: %s\n", optarg);
                    exit(-1);
                }
                bench_erase_sleep = erase_sleep*1.0e9;
                break;
            }
            // done parsing
            case -1:
                goto getopt_done;
            // unknown arg, getopt prints a message for us
            default:
                exit(-1);
        }
    }
getopt_done: ;

    if (argc > optind) {
        // reset our bench identifier list
        bench_ids = NULL;
        bench_id_count = 0;
        bench_id_capacity = 0;
    }

    // parse bench identifier, if any, cannibalizing the arg in the process
    for (; argc > optind; optind++) {
        bench_define_t *defines = NULL;
        size_t define_count = 0;

        // parse name, can be suite or case
        char *name = argv[optind];
        char *defines_ = strchr(name, ':');
        if (defines_) {
            *defines_ = '\0';
            defines_ += 1;
        }

        // remove optional path and .toml suffix
        char *slash = strrchr(name, '/');
        if (slash) {
            name = slash+1;
        }

        size_t name_len = strlen(name);
        if (name_len > 5 && strcmp(&name[name_len-5], ".toml") == 0) {
            name[name_len-5] = '\0';
        }

        if (defines_) {
            // parse defines
            while (true) {
                char *parsed;
                size_t d = leb16_parse(defines_, &parsed);
                intmax_t v = leb16_parse(parsed, &parsed);
                if (parsed == defines_) {
                    break;
                }
                defines_ = parsed;

                if (d >= define_count) {
                    // align to power of two to avoid any superlinear growth
                    size_t ncount = 1 << lfs_npw2(d+1);
                    defines = realloc(defines,
                            ncount*sizeof(bench_define_t));
                    memset(defines+define_count, 0,
                            (ncount-define_count)*sizeof(bench_define_t));
                    define_count = ncount;
                }
                defines[d] = BENCH_LIT(v);
            }
        }

        // append to identifier list
        *(bench_id_t*)mappend(
                (void**)&bench_ids,
                sizeof(bench_id_t),
                &bench_id_count,
                &bench_id_capacity) = (bench_id_t){
            .name = name,
            .defines = defines,
            .define_count = define_count,
        };
    }

    // do the thing
    op();

    // cleanup (need to be done for valgrind benching)
    bench_define_cleanup();
    if (bench_overrides) {
        for (size_t i = 0; i < bench_override_count; i++) {
            free((void*)bench_overrides[i].defines);
        }
        free((void*)bench_overrides);
    }
    if (bench_geometry_capacity) {
        free((void*)bench_geometries);
    }
    if (bench_id_capacity) {
        for (size_t i = 0; i < bench_id_count; i++) {
            free((void*)bench_ids[i].defines);
        }
        free((void*)bench_ids);
    }
}

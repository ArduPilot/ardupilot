/*
 * Runner for littlefs tests
 *
 * Copyright (c) 2022, The littlefs authors.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#endif

#include "runners/test_runner.h"
#include "bd/lfs_emubd.h"

#include <getopt.h>
#include <sys/types.h>
#include <errno.h>
#include <setjmp.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <execinfo.h>


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



// test_runner types

typedef struct test_geometry {
    const char *name;
    test_define_t defines[TEST_GEOMETRY_DEFINE_COUNT];
} test_geometry_t;

typedef struct test_powerloss {
    const char *name;
    void (*run)(
            const lfs_emubd_powercycles_t *cycles,
            size_t cycle_count,
            const struct test_suite *suite,
            const struct test_case *case_);
    const lfs_emubd_powercycles_t *cycles;
    size_t cycle_count;
} test_powerloss_t;

typedef struct test_id {
    const char *name;
    const test_define_t *defines;
    size_t define_count;
    const lfs_emubd_powercycles_t *cycles;
    size_t cycle_count;
} test_id_t;


// test suites are linked into a custom ld section
extern struct test_suite __start__test_suites;
extern struct test_suite __stop__test_suites;

const struct test_suite *test_suites = &__start__test_suites;
#define TEST_SUITE_COUNT \
    ((size_t)(&__stop__test_suites - &__start__test_suites))


// test define management
typedef struct test_define_map {
    const test_define_t *defines;
    size_t count;
} test_define_map_t;

typedef struct test_define_names {
    const char *const *names;
    size_t count;
} test_define_names_t;

intmax_t test_define_lit(void *data) {
    return (intptr_t)data;
}

#define TEST_CONST(x) {test_define_lit, (void*)(uintptr_t)(x)}
#define TEST_LIT(x) ((test_define_t)TEST_CONST(x))


#define TEST_DEF(k, v) \
    intmax_t test_define_##k(void *data) { \
        (void)data; \
        return v; \
    }

    TEST_IMPLICIT_DEFINES
#undef TEST_DEF

#define TEST_DEFINE_MAP_OVERRIDE    0
#define TEST_DEFINE_MAP_EXPLICIT    1
#define TEST_DEFINE_MAP_PERMUTATION 2
#define TEST_DEFINE_MAP_GEOMETRY    3
#define TEST_DEFINE_MAP_IMPLICIT    4
#define TEST_DEFINE_MAP_COUNT       5

test_define_map_t test_define_maps[TEST_DEFINE_MAP_COUNT] = {
    [TEST_DEFINE_MAP_IMPLICIT] = {
        (const test_define_t[TEST_IMPLICIT_DEFINE_COUNT]) {
            #define TEST_DEF(k, v) \
                [k##_i] = {test_define_##k, NULL},

                TEST_IMPLICIT_DEFINES
            #undef TEST_DEF
        },
        TEST_IMPLICIT_DEFINE_COUNT,
    },
};

#define TEST_DEFINE_NAMES_SUITE    0
#define TEST_DEFINE_NAMES_IMPLICIT 1
#define TEST_DEFINE_NAMES_COUNT    2

test_define_names_t test_define_names[TEST_DEFINE_NAMES_COUNT] = {
    [TEST_DEFINE_NAMES_IMPLICIT] = {
        (const char *const[TEST_IMPLICIT_DEFINE_COUNT]){
            #define TEST_DEF(k, v) \
                [k##_i] = #k,

                TEST_IMPLICIT_DEFINES
            #undef TEST_DEF
        },
        TEST_IMPLICIT_DEFINE_COUNT,
    },
};

intmax_t *test_define_cache;
size_t test_define_cache_count;
unsigned *test_define_cache_mask;

const char *test_define_name(size_t define) {
    // lookup in our test names
    for (size_t i = 0; i < TEST_DEFINE_NAMES_COUNT; i++) {
        if (define < test_define_names[i].count
                && test_define_names[i].names
                && test_define_names[i].names[define]) {
            return test_define_names[i].names[define];
        }
    }

    return NULL;
}

bool test_define_ispermutation(size_t define) {
    // is this define specific to the permutation?
    for (size_t i = 0; i < TEST_DEFINE_MAP_IMPLICIT; i++) {
        if (define < test_define_maps[i].count
                && test_define_maps[i].defines[define].cb) {
            return true;
        }
    }

    return false;
}

intmax_t test_define(size_t define) {
    // is the define in our cache?
    if (define < test_define_cache_count
            && (test_define_cache_mask[define/(8*sizeof(unsigned))]
                & (1 << (define%(8*sizeof(unsigned)))))) {
        return test_define_cache[define];
    }

    // lookup in our test defines
    for (size_t i = 0; i < TEST_DEFINE_MAP_COUNT; i++) {
        if (define < test_define_maps[i].count
                && test_define_maps[i].defines[define].cb) {
            intmax_t v = test_define_maps[i].defines[define].cb(
                    test_define_maps[i].defines[define].data);

            // insert into cache!
            test_define_cache[define] = v;
            test_define_cache_mask[define / (8*sizeof(unsigned))]
                    |= 1 << (define%(8*sizeof(unsigned)));

            return v;
        }
    }

    return 0;

    // not found?
    const char *name = test_define_name(define);
    fprintf(stderr, "error: undefined define %s (%zd)\n",
            name ? name : "(unknown)",
            define);
    assert(false);
    exit(-1);
}

void test_define_flush(void) {
    // clear cache between permutations
    memset(test_define_cache_mask, 0,
            sizeof(unsigned)*(
                (test_define_cache_count+(8*sizeof(unsigned))-1)
                / (8*sizeof(unsigned))));
}

// geometry updates
const test_geometry_t *test_geometry = NULL;

void test_define_geometry(const test_geometry_t *geometry) {
    test_define_maps[TEST_DEFINE_MAP_GEOMETRY] = (test_define_map_t){
            geometry->defines, TEST_GEOMETRY_DEFINE_COUNT};
}

// override updates
typedef struct test_override {
    const char *name;
    const intmax_t *defines;
    size_t permutations;
} test_override_t;

const test_override_t *test_overrides = NULL;
size_t test_override_count = 0;

test_define_t *test_override_defines = NULL;
size_t test_override_define_count = 0;
size_t test_override_define_permutations = 1;
size_t test_override_define_capacity = 0;

// suite/perm updates
void test_define_suite(const struct test_suite *suite) {
    test_define_names[TEST_DEFINE_NAMES_SUITE] = (test_define_names_t){
            suite->define_names, suite->define_count};

    // make sure our cache is large enough
    if (lfs_max(suite->define_count, TEST_IMPLICIT_DEFINE_COUNT)
            > test_define_cache_count) {
        // align to power of two to avoid any superlinear growth
        size_t ncount = 1 << lfs_npw2(
                lfs_max(suite->define_count, TEST_IMPLICIT_DEFINE_COUNT));
        test_define_cache = realloc(test_define_cache, ncount*sizeof(intmax_t));
        test_define_cache_mask = realloc(test_define_cache_mask,
                sizeof(unsigned)*(
                    (ncount+(8*sizeof(unsigned))-1)
                    / (8*sizeof(unsigned))));
        test_define_cache_count = ncount;
    }

    // map any overrides
    if (test_override_count > 0) {
        // first figure out the total size of override permutations
        size_t count = 0;
        size_t permutations = 1;
        for (size_t i = 0; i < test_override_count; i++) {
            for (size_t d = 0;
                    d < lfs_max(
                        suite->define_count,
                        TEST_IMPLICIT_DEFINE_COUNT);
                    d++) {
                // define name match?
                const char *name = test_define_name(d);
                if (name && strcmp(name, test_overrides[i].name) == 0) {
                    count = lfs_max(count, d+1);
                    permutations *= test_overrides[i].permutations;
                    break;
                }
            }
        }
        test_override_define_count = count;
        test_override_define_permutations = permutations;

        // make sure our override arrays are big enough
        if (count * permutations > test_override_define_capacity) {
            // align to power of two to avoid any superlinear growth
            size_t ncapacity = 1 << lfs_npw2(count * permutations);
            test_override_defines = realloc(
                    test_override_defines,
                    sizeof(test_define_t)*ncapacity);
            test_override_define_capacity = ncapacity;
        }

        // zero unoverridden defines
        memset(test_override_defines, 0,
                sizeof(test_define_t) * count * permutations);

        // compute permutations
        size_t p = 1;
        for (size_t i = 0; i < test_override_count; i++) {
            for (size_t d = 0;
                    d < lfs_max(
                        suite->define_count,
                        TEST_IMPLICIT_DEFINE_COUNT);
                    d++) {
                // define name match?
                const char *name = test_define_name(d);
                if (name && strcmp(name, test_overrides[i].name) == 0) {
                    // scatter the define permutations based on already
                    // seen permutations
                    for (size_t j = 0; j < permutations; j++) {
                        test_override_defines[j*count + d] = TEST_LIT(
                                test_overrides[i].defines[(j/p)
                                    % test_overrides[i].permutations]);
                    }

                    // keep track of how many permutations we've seen so far
                    p *= test_overrides[i].permutations;
                    break;
                }
            }
        }
    }
}

void test_define_perm(
        const struct test_suite *suite,
        const struct test_case *case_,
        size_t perm) {
    if (case_->defines) {
        test_define_maps[TEST_DEFINE_MAP_PERMUTATION] = (test_define_map_t){
                case_->defines + perm*suite->define_count,
                suite->define_count};
    } else {
        test_define_maps[TEST_DEFINE_MAP_PERMUTATION] = (test_define_map_t){
                NULL, 0};
    }
}

void test_define_override(size_t perm) {
    test_define_maps[TEST_DEFINE_MAP_OVERRIDE] = (test_define_map_t){
            test_override_defines + perm*test_override_define_count,
            test_override_define_count};
}

void test_define_explicit(
        const test_define_t *defines,
        size_t define_count) {
    test_define_maps[TEST_DEFINE_MAP_EXPLICIT] = (test_define_map_t){
            defines, define_count};
}

void test_define_cleanup(void) {
    // test define management can allocate a few things
    free(test_define_cache);
    free(test_define_cache_mask);
    free(test_override_defines);
}



// test state
extern const test_geometry_t *test_geometries;
extern size_t test_geometry_count;

extern const test_powerloss_t *test_powerlosses;
extern size_t test_powerloss_count;

const test_id_t *test_ids = (const test_id_t[]) {
    {NULL, NULL, 0, NULL, 0},
};
size_t test_id_count = 1;

size_t test_step_start = 0;
size_t test_step_stop = -1;
size_t test_step_step = 1;

const char *test_disk_path = NULL;
const char *test_trace_path = NULL;
bool test_trace_backtrace = false;
uint32_t test_trace_period = 0;
uint32_t test_trace_freq = 0;
FILE *test_trace_file = NULL;
uint32_t test_trace_cycles = 0;
uint64_t test_trace_time = 0;
uint64_t test_trace_open_time = 0;
lfs_emubd_sleep_t test_read_sleep = 0.0;
lfs_emubd_sleep_t test_prog_sleep = 0.0;
lfs_emubd_sleep_t test_erase_sleep = 0.0;

// this determines both the backtrace buffer and the trace printf buffer, if
// trace ends up interleaved or truncated this may need to be increased
#ifndef TEST_TRACE_BACKTRACE_BUFFER_SIZE
#define TEST_TRACE_BACKTRACE_BUFFER_SIZE 8192
#endif
void *test_trace_backtrace_buffer[
    TEST_TRACE_BACKTRACE_BUFFER_SIZE / sizeof(void*)];

// trace printing
void test_trace(const char *fmt, ...) {
    if (test_trace_path) {
        // sample at a specific period?
        if (test_trace_period) {
            if (test_trace_cycles % test_trace_period != 0) {
                test_trace_cycles += 1;
                return;
            }
            test_trace_cycles += 1;
        }

        // sample at a specific frequency?
        if (test_trace_freq) {
            struct timespec t;
            clock_gettime(CLOCK_MONOTONIC, &t);
            uint64_t now = (uint64_t)t.tv_sec*1000*1000*1000
                    + (uint64_t)t.tv_nsec;
            if (now - test_trace_time < (1000*1000*1000) / test_trace_freq) {
                return;
            }
            test_trace_time = now;
        }

        if (!test_trace_file) {
            // Tracing output is heavy and trying to open every trace
            // call is slow, so we only try to open the trace file every
            // so often. Note this doesn't affect successfully opened files
            struct timespec t;
            clock_gettime(CLOCK_MONOTONIC, &t);
            uint64_t now = (uint64_t)t.tv_sec*1000*1000*1000
                    + (uint64_t)t.tv_nsec;
            if (now - test_trace_open_time < 100*1000*1000) {
                return;
            }
            test_trace_open_time = now;

            // try to open the trace file
            int fd;
            if (strcmp(test_trace_path, "-") == 0) {
                fd = dup(1);
                if (fd < 0) {
                    return;
                }
            } else {
                fd = open(
                        test_trace_path,
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
                    TEST_TRACE_BACKTRACE_BUFFER_SIZE);
            assert(!err);
            test_trace_file = f;
        }

        // print trace
        va_list va;
        va_start(va, fmt);
        int res = vfprintf(test_trace_file, fmt, va);
        va_end(va);
        if (res < 0) {
            fclose(test_trace_file);
            test_trace_file = NULL;
            return;
        }

        if (test_trace_backtrace) {
            // print backtrace
            size_t count = backtrace(
                    test_trace_backtrace_buffer,
                    TEST_TRACE_BACKTRACE_BUFFER_SIZE);
            // note we skip our own stack frame
            for (size_t i = 1; i < count; i++) {
                res = fprintf(test_trace_file, "\tat %p\n",
                        test_trace_backtrace_buffer[i]);
                if (res < 0) {
                    fclose(test_trace_file);
                    test_trace_file = NULL;
                    return;
                }
            }
        }

        // flush immediately
        fflush(test_trace_file);
    }
}


// test prng
uint32_t test_prng(uint32_t *state) {
    // A simple xorshift32 generator, easily reproducible. Keep in mind
    // determinism is much more important than actual randomness here.
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}


// encode our permutation into a reusable id
static void perm_printid(
        const struct test_suite *suite,
        const struct test_case *case_,
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count) {
    (void)suite;
    // case[:permutation[:powercycles]]
    printf("%s:", case_->name);
    for (size_t d = 0;
            d < lfs_max(
                suite->define_count,
                TEST_IMPLICIT_DEFINE_COUNT);
            d++) {
        if (test_define_ispermutation(d)) {
            leb16_print(d);
            leb16_print(TEST_DEFINE(d));
        }
    }

    // only print power-cycles if any occured
    if (cycles) {
        printf(":");
        for (size_t i = 0; i < cycle_count; i++) {
            leb16_print(cycles[i]);
        }
    }
}


// a quick trie for keeping track of permutations we've seen
typedef struct test_seen {
    struct test_seen_branch *branches;
    size_t branch_count;
    size_t branch_capacity;
} test_seen_t;

struct test_seen_branch {
    intmax_t define;
    struct test_seen branch;
};

bool test_seen_insert(
        test_seen_t *seen,
        const struct test_suite *suite,
        const struct test_case *case_) {
    (void)case_;
    bool was_seen = true;

    // use the currently set defines
    for (size_t d = 0;
            d < lfs_max(
                suite->define_count,
                TEST_IMPLICIT_DEFINE_COUNT);
            d++) {
        // treat unpermuted defines the same as 0
        intmax_t define = test_define_ispermutation(d) ? TEST_DEFINE(d) : 0;

        // already seen?
        struct test_seen_branch *branch = NULL;
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
                    sizeof(struct test_seen_branch),
                    &seen->branch_count,
                    &seen->branch_capacity);
            branch->define = define;
            branch->branch = (test_seen_t){NULL, 0, 0};
        }

        seen = &branch->branch;
    }

    return was_seen;
}

void test_seen_cleanup(test_seen_t *seen) {
    for (size_t i = 0; i < seen->branch_count; i++) {
        test_seen_cleanup(&seen->branches[i].branch);
    }
    free(seen->branches);
}

static void run_powerloss_none(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_);
static void run_powerloss_cycles(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_);

// iterate through permutations in a test case
static void case_forperm(
        const struct test_suite *suite,
        const struct test_case *case_,
        const test_define_t *defines,
        size_t define_count,
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        void (*cb)(
            void *data,
            const struct test_suite *suite,
            const struct test_case *case_,
            const test_powerloss_t *powerloss),
        void *data) {
    // explicit permutation?
    if (defines) {
        test_define_explicit(defines, define_count);

        for (size_t v = 0; v < test_override_define_permutations; v++) {
            // define override permutation
            test_define_override(v);
            test_define_flush();

            // explicit powerloss cycles?
            if (cycles) {
                cb(data, suite, case_, &(test_powerloss_t){
                        .run=run_powerloss_cycles,
                        .cycles=cycles,
                        .cycle_count=cycle_count});
            } else {
                for (size_t p = 0; p < test_powerloss_count; p++) {
                    // skip non-reentrant tests when powerloss testing
                    if (test_powerlosses[p].run != run_powerloss_none
                            && !(case_->flags & TEST_REENTRANT)) {
                        continue;
                    }

                    cb(data, suite, case_, &test_powerlosses[p]);
                }
            }
        }

        return;
    }

    test_seen_t seen = {NULL, 0, 0};

    for (size_t k = 0; k < case_->permutations; k++) {
        // define permutation
        test_define_perm(suite, case_, k);

        for (size_t v = 0; v < test_override_define_permutations; v++) {
            // define override permutation
            test_define_override(v);

            for (size_t g = 0; g < test_geometry_count; g++) {
                // define geometry
                test_define_geometry(&test_geometries[g]);
                test_define_flush();

                // have we seen this permutation before?
                bool was_seen = test_seen_insert(&seen, suite, case_);
                if (!(k == 0 && v == 0 && g == 0) && was_seen) {
                    continue;
                }

                if (cycles) {
                    cb(data, suite, case_, &(test_powerloss_t){
                            .run=run_powerloss_cycles,
                            .cycles=cycles,
                            .cycle_count=cycle_count});
                } else {
                    for (size_t p = 0; p < test_powerloss_count; p++) {
                        // skip non-reentrant tests when powerloss testing
                        if (test_powerlosses[p].run != run_powerloss_none
                                && !(case_->flags & TEST_REENTRANT)) {
                            continue;
                        }

                        cb(data, suite, case_, &test_powerlosses[p]);
                    }
                }
            }
        }
    }

    test_seen_cleanup(&seen);
}


// how many permutations are there actually in a test case
struct perm_count_state {
    size_t total;
    size_t filtered;
};

void perm_count(
        void *data,
        const struct test_suite *suite,
        const struct test_case *case_,
        const test_powerloss_t *powerloss) {
    struct perm_count_state *state = data;
    (void)suite;
    (void)case_;
    (void)powerloss;

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
    test_flags_t flags = 0;
    struct perm_count_state perms = {0, 0};

    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            test_define_suite(&test_suites[i]);

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                cases += 1;
                case_forperm(
                        &test_suites[i],
                        &test_suites[i].cases[j],
                        test_ids[t].defines,
                        test_ids[t].define_count,
                        test_ids[t].cycles,
                        test_ids[t].cycle_count,
                        perm_count,
                        &perms);
            }

            suites += 1;
            flags |= test_suites[i].flags;
        }
    }

    char perm_buf[64];
    sprintf(perm_buf, "%zu/%zu", perms.filtered, perms.total);
    char flag_buf[64];
    sprintf(flag_buf, "%s%s",
            (flags & TEST_REENTRANT) ? "r" : "",
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
    for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
        size_t len = strlen(test_suites[i].name);
        if (len > name_width) {
            name_width = len;
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %7s %7s %11s\n",
            name_width, "suite", "flags", "cases", "perms");
    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            test_define_suite(&test_suites[i]);

            size_t cases = 0;
            struct perm_count_state perms = {0, 0};

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                cases += 1;
                case_forperm(
                        &test_suites[i],
                        &test_suites[i].cases[j],
                        test_ids[t].defines,
                        test_ids[t].define_count,
                        test_ids[t].cycles,
                        test_ids[t].cycle_count,
                        perm_count,
                        &perms);
            }

            // no tests found?
            if (!cases) {
                continue;
            }

            char perm_buf[64];
            sprintf(perm_buf, "%zu/%zu", perms.filtered, perms.total);
            char flag_buf[64];
            sprintf(flag_buf, "%s%s",
                    (test_suites[i].flags & TEST_REENTRANT) ? "r" : "",
                    (!test_suites[i].flags) ? "-" : "");
            printf("%-*s  %7s %7zu %11s\n",
                    name_width,
                    test_suites[i].name,
                    flag_buf,
                    cases,
                    perm_buf);
        }
    }
}

static void list_cases(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
        for (size_t j = 0; j < test_suites[i].case_count; j++) {
            size_t len = strlen(test_suites[i].cases[j].name);
            if (len > name_width) {
                name_width = len;
            }
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %7s %11s\n", name_width, "case", "flags", "perms");
    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            test_define_suite(&test_suites[i]);

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                struct perm_count_state perms = {0, 0};
                case_forperm(
                        &test_suites[i],
                        &test_suites[i].cases[j],
                        test_ids[t].defines,
                        test_ids[t].define_count,
                        test_ids[t].cycles,
                        test_ids[t].cycle_count,
                        perm_count,
                        &perms);

                char perm_buf[64];
                sprintf(perm_buf, "%zu/%zu", perms.filtered, perms.total);
                char flag_buf[64];
                sprintf(flag_buf, "%s%s",
                        (test_suites[i].cases[j].flags & TEST_REENTRANT)
                            ? "r" : "",
                        (!test_suites[i].cases[j].flags)
                            ? "-" : "");
                printf("%-*s  %7s %11s\n",
                        name_width,
                        test_suites[i].cases[j].name,
                        flag_buf,
                        perm_buf);
            }
        }
    }
}

static void list_suite_paths(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
        size_t len = strlen(test_suites[i].name);
        if (len > name_width) {
            name_width = len;
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %s\n", name_width, "suite", "path");
    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            size_t cases = 0;

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                cases += 1;
            }

            // no tests found?
            if (!cases) {
                continue;
            }

            printf("%-*s  %s\n",
                    name_width,
                    test_suites[i].name,
                    test_suites[i].path);
        }
    }
}

static void list_case_paths(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
        for (size_t j = 0; j < test_suites[i].case_count; j++) {
            size_t len = strlen(test_suites[i].cases[j].name);
            if (len > name_width) {
                name_width = len;
            }
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s  %s\n", name_width, "case", "path");
    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                printf("%-*s  %s\n",
                        name_width,
                        test_suites[i].cases[j].name,
                        test_suites[i].cases[j].path);
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
    const char *name = test_define_name(d);
    intmax_t value = TEST_DEFINE(d);

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
        const struct test_suite *suite,
        const struct test_case *case_,
        const test_powerloss_t *powerloss) {
    struct list_defines_defines *defines = data;
    (void)suite;
    (void)case_;
    (void)powerloss;

    // collect defines
    for (size_t d = 0;
            d < lfs_max(suite->define_count,
                TEST_IMPLICIT_DEFINE_COUNT);
            d++) {
        if (d < TEST_IMPLICIT_DEFINE_COUNT
                || test_define_ispermutation(d)) {
            list_defines_add(defines, d);
        }
    }
}

void perm_list_permutation_defines(
        void *data,
        const struct test_suite *suite,
        const struct test_case *case_,
        const test_powerloss_t *powerloss) {
    struct list_defines_defines *defines = data;
    (void)suite;
    (void)case_;
    (void)powerloss;

    // collect permutation_defines
    for (size_t d = 0;
            d < lfs_max(suite->define_count,
                TEST_IMPLICIT_DEFINE_COUNT);
            d++) {
        if (test_define_ispermutation(d)) {
            list_defines_add(defines, d);
        }
    }
}

extern const test_geometry_t builtin_geometries[];

static void list_defines(void) {
    struct list_defines_defines defines = {NULL, 0, 0};

    // add defines
    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            test_define_suite(&test_suites[i]);

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                case_forperm(
                        &test_suites[i],
                        &test_suites[i].cases[j],
                        test_ids[t].defines,
                        test_ids[t].define_count,
                        test_ids[t].cycles,
                        test_ids[t].cycle_count,
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
    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            test_define_suite(&test_suites[i]);

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                case_forperm(
                        &test_suites[i],
                        &test_suites[i].cases[j],
                        test_ids[t].defines,
                        test_ids[t].define_count,
                        test_ids[t].cycles,
                        test_ids[t].cycle_count,
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
    test_define_suite(&(const struct test_suite){0});

    // make sure to include builtin geometries here
    extern const test_geometry_t builtin_geometries[];
    for (size_t g = 0; builtin_geometries[g].name; g++) {
        test_define_geometry(&builtin_geometries[g]);
        test_define_flush();

        // add implicit defines
        for (size_t d = 0; d < TEST_IMPLICIT_DEFINE_COUNT; d++) {
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



// geometries to test

const test_geometry_t builtin_geometries[] = {
    {"default", {{0}, TEST_CONST(16),   TEST_CONST(512),   {0}}},
    {"eeprom",  {{0}, TEST_CONST(1),    TEST_CONST(512),   {0}}},
    {"emmc",    {{0}, {0},              TEST_CONST(512),   {0}}},
    {"nor",     {{0}, TEST_CONST(1),    TEST_CONST(4096),  {0}}},
    {"nand",    {{0}, TEST_CONST(4096), TEST_CONST(32768), {0}}},
    {NULL, {{0}, {0}, {0}, {0}}},
};

const test_geometry_t *test_geometries = builtin_geometries;
size_t test_geometry_count = 5;

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
    test_define_suite(&(const struct test_suite){0});

    printf("%-*s  %7s %7s %7s %7s %11s\n",
            name_width, "geometry", "read", "prog", "erase", "count", "size");
    for (size_t g = 0; builtin_geometries[g].name; g++) {
        test_define_geometry(&builtin_geometries[g]);
        test_define_flush();
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


// scenarios to run tests under power-loss

static void run_powerloss_none(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_) {
    (void)cycles;
    (void)cycle_count;
    (void)suite;

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
    #ifdef LFS_MULTIVERSION
        .disk_version       = DISK_VERSION,
    #endif
    };

    struct lfs_emubd_config bdcfg = {
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .erase_size         = ERASE_SIZE,
        .erase_count        = ERASE_COUNT,
        .erase_value        = ERASE_VALUE,
        .erase_cycles       = ERASE_CYCLES,
        .badblock_behavior  = BADBLOCK_BEHAVIOR,
        .disk_path          = test_disk_path,
        .read_sleep         = test_read_sleep,
        .prog_sleep         = test_prog_sleep,
        .erase_sleep        = test_erase_sleep,
    };

    int err = lfs_emubd_create(&cfg, &bdcfg);
    if (err) {
        fprintf(stderr, "error: could not create block device: %d\n", err);
        exit(-1);
    }

    // run the test
    printf("running ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    case_->run(&cfg);

    printf("finished ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    // cleanup
    err = lfs_emubd_destroy(&cfg);
    if (err) {
        fprintf(stderr, "error: could not destroy block device: %d\n", err);
        exit(-1);
    }
}

static void powerloss_longjmp(void *c) {
    jmp_buf *powerloss_jmp = c;
    longjmp(*powerloss_jmp, 1);
}

static void run_powerloss_linear(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_) {
    (void)cycles;
    (void)cycle_count;
    (void)suite;

    // create block device and configuration
    lfs_emubd_t bd;
    jmp_buf powerloss_jmp;
    volatile lfs_emubd_powercycles_t i = 1;

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
    #ifdef LFS_MULTIVERSION
        .disk_version       = DISK_VERSION,
    #endif
    };

    struct lfs_emubd_config bdcfg = {
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .erase_size         = ERASE_SIZE,
        .erase_count        = ERASE_COUNT,
        .erase_value        = ERASE_VALUE,
        .erase_cycles       = ERASE_CYCLES,
        .badblock_behavior  = BADBLOCK_BEHAVIOR,
        .disk_path          = test_disk_path,
        .read_sleep         = test_read_sleep,
        .prog_sleep         = test_prog_sleep,
        .erase_sleep        = test_erase_sleep,
        .power_cycles       = i,
        .powerloss_behavior = POWERLOSS_BEHAVIOR,
        .powerloss_cb       = powerloss_longjmp,
        .powerloss_data     = &powerloss_jmp,
    };

    int err = lfs_emubd_create(&cfg, &bdcfg);
    if (err) {
        fprintf(stderr, "error: could not create block device: %d\n", err);
        exit(-1);
    }

    // run the test, increasing power-cycles as power-loss events occur
    printf("running ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    while (true) {
        if (!setjmp(powerloss_jmp)) {
            // run the test
            case_->run(&cfg);
            break;
        }

        // power-loss!
        printf("powerloss ");
        perm_printid(suite, case_, NULL, 0);
        printf(":");
        for (lfs_emubd_powercycles_t j = 1; j <= i; j++) {
            leb16_print(j);
        }
        printf("\n");

        i += 1;
        lfs_emubd_setpowercycles(&cfg, i);
    }

    printf("finished ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    // cleanup
    err = lfs_emubd_destroy(&cfg);
    if (err) {
        fprintf(stderr, "error: could not destroy block device: %d\n", err);
        exit(-1);
    }
}

static void run_powerloss_log(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_) {
    (void)cycles;
    (void)cycle_count;
    (void)suite;

    // create block device and configuration
    lfs_emubd_t bd;
    jmp_buf powerloss_jmp;
    volatile lfs_emubd_powercycles_t i = 1;

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
    #ifdef LFS_MULTIVERSION
        .disk_version       = DISK_VERSION,
    #endif
    };

    struct lfs_emubd_config bdcfg = {
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .erase_size         = ERASE_SIZE,
        .erase_count        = ERASE_COUNT,
        .erase_value        = ERASE_VALUE,
        .erase_cycles       = ERASE_CYCLES,
        .badblock_behavior  = BADBLOCK_BEHAVIOR,
        .disk_path          = test_disk_path,
        .read_sleep         = test_read_sleep,
        .prog_sleep         = test_prog_sleep,
        .erase_sleep        = test_erase_sleep,
        .power_cycles       = i,
        .powerloss_behavior = POWERLOSS_BEHAVIOR,
        .powerloss_cb       = powerloss_longjmp,
        .powerloss_data     = &powerloss_jmp,
    };

    int err = lfs_emubd_create(&cfg, &bdcfg);
    if (err) {
        fprintf(stderr, "error: could not create block device: %d\n", err);
        exit(-1);
    }

    // run the test, increasing power-cycles as power-loss events occur
    printf("running ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    while (true) {
        if (!setjmp(powerloss_jmp)) {
            // run the test
            case_->run(&cfg);
            break;
        }

        // power-loss!
        printf("powerloss ");
        perm_printid(suite, case_, NULL, 0);
        printf(":");
        for (lfs_emubd_powercycles_t j = 1; j <= i; j *= 2) {
            leb16_print(j);
        }
        printf("\n");

        i *= 2;
        lfs_emubd_setpowercycles(&cfg, i);
    }

    printf("finished ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    // cleanup
    err = lfs_emubd_destroy(&cfg);
    if (err) {
        fprintf(stderr, "error: could not destroy block device: %d\n", err);
        exit(-1);
    }
}

static void run_powerloss_cycles(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_) {
    (void)suite;

    // create block device and configuration
    lfs_emubd_t bd;
    jmp_buf powerloss_jmp;
    volatile size_t i = 0;

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
    #ifdef LFS_MULTIVERSION
        .disk_version       = DISK_VERSION,
    #endif
    };

    struct lfs_emubd_config bdcfg = {
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .erase_size         = ERASE_SIZE,
        .erase_count        = ERASE_COUNT,
        .erase_value        = ERASE_VALUE,
        .erase_cycles       = ERASE_CYCLES,
        .badblock_behavior  = BADBLOCK_BEHAVIOR,
        .disk_path          = test_disk_path,
        .read_sleep         = test_read_sleep,
        .prog_sleep         = test_prog_sleep,
        .erase_sleep        = test_erase_sleep,
        .power_cycles       = (i < cycle_count) ? cycles[i] : 0,
        .powerloss_behavior = POWERLOSS_BEHAVIOR,
        .powerloss_cb       = powerloss_longjmp,
        .powerloss_data     = &powerloss_jmp,
    };

    int err = lfs_emubd_create(&cfg, &bdcfg);
    if (err) {
        fprintf(stderr, "error: could not create block device: %d\n", err);
        exit(-1);
    }

    // run the test, increasing power-cycles as power-loss events occur
    printf("running ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    while (true) {
        if (!setjmp(powerloss_jmp)) {
            // run the test
            case_->run(&cfg);
            break;
        }

        // power-loss!
        assert(i <= cycle_count);
        printf("powerloss ");
        perm_printid(suite, case_, cycles, i+1);
        printf("\n");

        i += 1;
        lfs_emubd_setpowercycles(&cfg,
                (i < cycle_count) ? cycles[i] : 0);
    }

    printf("finished ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    // cleanup
    err = lfs_emubd_destroy(&cfg);
    if (err) {
        fprintf(stderr, "error: could not destroy block device: %d\n", err);
        exit(-1);
    }
}

struct powerloss_exhaustive_state {
    struct lfs_config *cfg;

    lfs_emubd_t *branches;
    size_t branch_count;
    size_t branch_capacity;
};

struct powerloss_exhaustive_cycles {
    lfs_emubd_powercycles_t *cycles;
    size_t cycle_count;
    size_t cycle_capacity;
};

static void powerloss_exhaustive_branch(void *c) {
    struct powerloss_exhaustive_state *state = c;
    // append to branches
    lfs_emubd_t *branch = mappend(
            (void**)&state->branches,
            sizeof(lfs_emubd_t),
            &state->branch_count,
            &state->branch_capacity);
    if (!branch) {
        fprintf(stderr, "error: exhaustive: out of memory\n");
        exit(-1);
    }

    // create copy-on-write copy
    int err = lfs_emubd_copy(state->cfg, branch);
    if (err) {
        fprintf(stderr, "error: exhaustive: could not create bd copy\n");
        exit(-1);
    }

    // also trigger on next power cycle
    lfs_emubd_setpowercycles(state->cfg, 1);
}

static void run_powerloss_exhaustive_layer(
        struct powerloss_exhaustive_cycles *cycles,
        const struct test_suite *suite,
        const struct test_case *case_,
        struct lfs_config *cfg,
        struct lfs_emubd_config *bdcfg,
        size_t depth) {
    (void)suite;

    struct powerloss_exhaustive_state state = {
        .cfg = cfg,
        .branches = NULL,
        .branch_count = 0,
        .branch_capacity = 0,
    };

    // run through the test without additional powerlosses, collecting possible
    // branches as we do so
    lfs_emubd_setpowercycles(state.cfg, depth > 0 ? 1 : 0);
    bdcfg->powerloss_data = &state;

    // run the tests
    case_->run(cfg);

    // aggressively clean up memory here to try to keep our memory usage low
    int err = lfs_emubd_destroy(cfg);
    if (err) {
        fprintf(stderr, "error: could not destroy block device: %d\n", err);
        exit(-1);
    }

    // recurse into each branch
    for (size_t i = 0; i < state.branch_count; i++) {
        // first push and print the branch
        lfs_emubd_powercycles_t *cycle = mappend(
                (void**)&cycles->cycles,
                sizeof(lfs_emubd_powercycles_t),
                &cycles->cycle_count,
                &cycles->cycle_capacity);
        if (!cycle) {
            fprintf(stderr, "error: exhaustive: out of memory\n");
            exit(-1);
        }
        *cycle = i+1;

        printf("powerloss ");
        perm_printid(suite, case_, cycles->cycles, cycles->cycle_count);
        printf("\n");

        // now recurse
        cfg->context = &state.branches[i];
        run_powerloss_exhaustive_layer(cycles,
                suite, case_,
                cfg, bdcfg, depth-1);

        // pop the cycle
        cycles->cycle_count -= 1;
    }

    // clean up memory
    free(state.branches);
}

static void run_powerloss_exhaustive(
        const lfs_emubd_powercycles_t *cycles,
        size_t cycle_count,
        const struct test_suite *suite,
        const struct test_case *case_) {
    (void)cycles;
    (void)suite;

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
    #ifdef LFS_MULTIVERSION
        .disk_version       = DISK_VERSION,
    #endif
    };

    struct lfs_emubd_config bdcfg = {
        .read_size          = READ_SIZE,
        .prog_size          = PROG_SIZE,
        .erase_size         = ERASE_SIZE,
        .erase_count        = ERASE_COUNT,
        .erase_value        = ERASE_VALUE,
        .erase_cycles       = ERASE_CYCLES,
        .badblock_behavior  = BADBLOCK_BEHAVIOR,
        .disk_path          = test_disk_path,
        .read_sleep         = test_read_sleep,
        .prog_sleep         = test_prog_sleep,
        .erase_sleep        = test_erase_sleep,
        .powerloss_behavior = POWERLOSS_BEHAVIOR,
        .powerloss_cb       = powerloss_exhaustive_branch,
        .powerloss_data     = NULL,
    };

    int err = lfs_emubd_create(&cfg, &bdcfg);
    if (err) {
        fprintf(stderr, "error: could not create block device: %d\n", err);
        exit(-1);
    }

    // run the test, increasing power-cycles as power-loss events occur
    printf("running ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");

    // recursively exhaust each layer of powerlosses
    run_powerloss_exhaustive_layer(
            &(struct powerloss_exhaustive_cycles){NULL, 0, 0},
            suite, case_,
            &cfg, &bdcfg, cycle_count);

    printf("finished ");
    perm_printid(suite, case_, NULL, 0);
    printf("\n");
}


const test_powerloss_t builtin_powerlosses[] = {
    {"none",       run_powerloss_none,       NULL, 0},
    {"log",        run_powerloss_log,        NULL, 0},
    {"linear",     run_powerloss_linear,     NULL, 0},
    {"exhaustive", run_powerloss_exhaustive, NULL, SIZE_MAX},
    {NULL, NULL, NULL, 0},
};

const char *const builtin_powerlosses_help[] = {
    "Run with no power-losses.",
    "Run with exponentially-decreasing power-losses.",
    "Run with linearly-decreasing power-losses.",
    "Run a all permutations of power-losses, this may take a while.",
    "Run a all permutations of n power-losses.",
    "Run a custom comma-separated set of power-losses.",
    "Run a custom leb16-encoded set of power-losses.",
};

// default to -Pnone,linear, which provides a good heuristic while still
// running quickly
const test_powerloss_t *test_powerlosses = (const test_powerloss_t[]){
    {"none",   run_powerloss_none,   NULL, 0},
    {"linear", run_powerloss_linear, NULL, 0},
};
size_t test_powerloss_count = 2;

static void list_powerlosses(void) {
    // at least size so that names fit
    unsigned name_width = 23;
    for (size_t i = 0; builtin_powerlosses[i].name; i++) {
        size_t len = strlen(builtin_powerlosses[i].name);
        if (len > name_width) {
            name_width = len;
        }
    }
    name_width = 4*((name_width+1+4-1)/4)-1;

    printf("%-*s %s\n", name_width, "scenario", "description");
    size_t i = 0;
    for (; builtin_powerlosses[i].name; i++) {
        printf("%-*s %s\n",
                name_width,
                builtin_powerlosses[i].name,
                builtin_powerlosses_help[i]);
    }

    // a couple more options with special parsing
    printf("%-*s %s\n", name_width, "1,2,3",   builtin_powerlosses_help[i+0]);
    printf("%-*s %s\n", name_width, "{1,2,3}", builtin_powerlosses_help[i+1]);
    printf("%-*s %s\n", name_width, ":1248g1", builtin_powerlosses_help[i+2]);
}


// global test step count
size_t test_step = 0;

void perm_run(
        void *data,
        const struct test_suite *suite,
        const struct test_case *case_,
        const test_powerloss_t *powerloss) {
    (void)data;

    // skip this step?
    if (!(test_step >= test_step_start
            && test_step < test_step_stop
            && (test_step-test_step_start) % test_step_step == 0)) {
        test_step += 1;
        return;
    }
    test_step += 1;

    // filter?
    if (case_->filter && !case_->filter()) {
        printf("skipped ");
        perm_printid(suite, case_, NULL, 0);
        printf("\n");
        return;
    }

    powerloss->run(
            powerloss->cycles, powerloss->cycle_count,
            suite, case_);
}

static void run(void) {
    // ignore disconnected pipes
    signal(SIGPIPE, SIG_IGN);

    for (size_t t = 0; t < test_id_count; t++) {
        for (size_t i = 0; i < TEST_SUITE_COUNT; i++) {
            test_define_suite(&test_suites[i]);

            for (size_t j = 0; j < test_suites[i].case_count; j++) {
                // does neither suite nor case name match?
                if (test_ids[t].name && !(
                        strcmp(test_ids[t].name,
                            test_suites[i].name) == 0
                        || strcmp(test_ids[t].name,
                            test_suites[i].cases[j].name) == 0)) {
                    continue;
                }

                case_forperm(
                        &test_suites[i],
                        &test_suites[i].cases[j],
                        test_ids[t].defines,
                        test_ids[t].define_count,
                        test_ids[t].cycles,
                        test_ids[t].cycle_count,
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
    OPT_LIST_POWERLOSSES         = 7,
    OPT_DEFINE                   = 'D',
    OPT_GEOMETRY                 = 'G',
    OPT_POWERLOSS                = 'P',
    OPT_STEP                     = 's',
    OPT_DISK                     = 'd',
    OPT_TRACE                    = 't',
    OPT_TRACE_BACKTRACE          = 8,
    OPT_TRACE_PERIOD             = 9,
    OPT_TRACE_FREQ               = 10,
    OPT_READ_SLEEP               = 11,
    OPT_PROG_SLEEP               = 12,
    OPT_ERASE_SLEEP              = 13,
};

const char *short_opts = "hYlLD:G:P:s:d:t:";

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
    {"list-powerlosses", no_argument,       NULL, OPT_LIST_POWERLOSSES},
    {"define",           required_argument, NULL, OPT_DEFINE},
    {"geometry",         required_argument, NULL, OPT_GEOMETRY},
    {"powerloss",        required_argument, NULL, OPT_POWERLOSS},
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
    "List test suites.",
    "List test cases.",
    "List the path for each test suite.",
    "List the path and line number for each test case.",
    "List all defines in this test-runner.",
    "List explicit defines in this test-runner.",
    "List implicit defines in this test-runner.",
    "List the available disk geometries.",
    "List the available power-loss scenarios.",
    "Override a test define.",
    "Comma-separated list of disk geometries to test.",
    "Comma-separated list of power-loss scenarios to test.",
    "Comma-separated range of test permutations to run (start,stop,step).",
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

    size_t test_override_capacity = 0;
    size_t test_geometry_capacity = 0;
    size_t test_powerloss_capacity = 0;
    size_t test_id_capacity = 0;

    // parse options
    while (true) {
        int c = getopt_long(argc, argv, short_opts, long_opts, NULL);
        switch (c) {
            // generate help message
            case OPT_HELP: {
                printf("usage: %s [options] [test_id]\n", argv[0]);
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
            case OPT_LIST_POWERLOSSES:
                op = list_powerlosses;
                break;
            // configuration
            case OPT_DEFINE: {
                // allocate space
                test_override_t *override = mappend(
                        (void**)&test_overrides,
                        sizeof(test_override_t),
                        &test_override_count,
                        &test_override_capacity);

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
                if (test_geometry_capacity > 0) {
                    free((test_geometry_t*)test_geometries);
                }
                test_geometries = NULL;
                test_geometry_count = 0;
                test_geometry_capacity = 0;

                // parse the comma separated list of disk geometries
                while (*optarg) {
                    // allocate space
                    test_geometry_t *geometry = mappend(
                            (void**)&test_geometries,
                            sizeof(test_geometry_t),
                            &test_geometry_count,
                            &test_geometry_capacity);

                    // parse the disk geometry
                    optarg += strspn(optarg, " ");

                    // named disk geometry
                    size_t len = strcspn(optarg, " ,");
                    for (size_t i = 0; builtin_geometries[i].name; i++) {
                        if (len == strlen(builtin_geometries[i].name)
                                && memcmp(optarg,
                                    builtin_geometries[i].name,
                                    len) == 0)  {
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
                        memset(geometry, 0, sizeof(test_geometry_t));
                        if (count >= 3) {
                            geometry->defines[READ_SIZE_i]
                                    = TEST_LIT(sizes[0]);
                            geometry->defines[PROG_SIZE_i]
                                    = TEST_LIT(sizes[1]);
                            geometry->defines[ERASE_SIZE_i]
                                    = TEST_LIT(sizes[2]);
                        } else if (count >= 2) {
                            geometry->defines[PROG_SIZE_i]
                                    = TEST_LIT(sizes[0]);
                            geometry->defines[ERASE_SIZE_i]
                                    = TEST_LIT(sizes[1]);
                        } else {
                            geometry->defines[ERASE_SIZE_i]
                                    = TEST_LIT(sizes[0]);
                        }
                        if (count >= 4) {
                            geometry->defines[ERASE_COUNT_i]
                                    = TEST_LIT(sizes[3]);
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
                        memset(geometry, 0, sizeof(test_geometry_t));
                        if (count >= 3) {
                            geometry->defines[READ_SIZE_i]
                                    = TEST_LIT(sizes[0]);
                            geometry->defines[PROG_SIZE_i]
                                    = TEST_LIT(sizes[1]);
                            geometry->defines[ERASE_SIZE_i]
                                    = TEST_LIT(sizes[2]);
                        } else if (count >= 2) {
                            geometry->defines[PROG_SIZE_i]
                                    = TEST_LIT(sizes[0]);
                            geometry->defines[ERASE_SIZE_i]
                                    = TEST_LIT(sizes[1]);
                        } else {
                            geometry->defines[ERASE_SIZE_i]
                                    = TEST_LIT(sizes[0]);
                        }
                        if (count >= 4) {
                            geometry->defines[ERASE_COUNT_i]
                                    = TEST_LIT(sizes[3]);
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
            case OPT_POWERLOSS: {
                // reset our powerloss scenarios
                if (test_powerloss_capacity > 0) {
                    free((test_powerloss_t*)test_powerlosses);
                }
                test_powerlosses = NULL;
                test_powerloss_count = 0;
                test_powerloss_capacity = 0;

                // parse the comma separated list of power-loss scenarios
                while (*optarg) {
                    // allocate space
                    test_powerloss_t *powerloss = mappend(
                            (void**)&test_powerlosses,
                            sizeof(test_powerloss_t),
                            &test_powerloss_count,
                            &test_powerloss_capacity);

                    // parse the power-loss scenario
                    optarg += strspn(optarg, " ");

                    // named power-loss scenario
                    size_t len = strcspn(optarg, " ,");
                    for (size_t i = 0; builtin_powerlosses[i].name; i++) {
                        if (len == strlen(builtin_powerlosses[i].name)
                                && memcmp(optarg,
                                    builtin_powerlosses[i].name,
                                    len) == 0) {
                            *powerloss = builtin_powerlosses[i];
                            optarg += len;
                            goto powerloss_next;
                        }
                    }

                    // comma-separated permutation
                    if (*optarg == '{') {
                        lfs_emubd_powercycles_t *cycles = NULL;
                        size_t cycle_count = 0;
                        size_t cycle_capacity = 0;

                        char *s = optarg + 1;
                        while (true) {
                            char *parsed = NULL;
                            *(lfs_emubd_powercycles_t*)mappend(
                                    (void**)&cycles,
                                    sizeof(lfs_emubd_powercycles_t),
                                    &cycle_count,
                                    &cycle_capacity)
                                    = strtoumax(s, &parsed, 0);

                            s = parsed + strspn(parsed, " ");
                            if (*s == ',') {
                                s += 1;
                                continue;
                            } else if (*s == '}') {
                                s += 1;
                                break;
                            } else {
                                goto powerloss_unknown;
                            }
                        }

                        *powerloss = (test_powerloss_t){
                            .run = run_powerloss_cycles,
                            .cycles = cycles,
                            .cycle_count = cycle_count,
                        };
                        optarg = s;
                        goto powerloss_next;
                    }

                    // leb16-encoded permutation
                    if (*optarg == ':') {
                        lfs_emubd_powercycles_t *cycles = NULL;
                        size_t cycle_count = 0;
                        size_t cycle_capacity = 0;

                        char *s = optarg + 1;
                        while (true) {
                            char *parsed = NULL;
                            uintmax_t x = leb16_parse(s, &parsed);
                            if (parsed == s) {
                                break;
                            }

                            *(lfs_emubd_powercycles_t*)mappend(
                                    (void**)&cycles,
                                    sizeof(lfs_emubd_powercycles_t),
                                    &cycle_count,
                                    &cycle_capacity) = x;
                            s = parsed;
                        }

                        *powerloss = (test_powerloss_t){
                            .run = run_powerloss_cycles,
                            .cycles = cycles,
                            .cycle_count = cycle_count,
                        };
                        optarg = s;
                        goto powerloss_next;
                    }

                    // exhaustive permutations
                    {
                        char *parsed = NULL;
                        size_t count = strtoumax(optarg, &parsed, 0);
                        if (parsed == optarg) {
                            goto powerloss_unknown;
                        }
                        *powerloss = (test_powerloss_t){
                            .run = run_powerloss_exhaustive,
                            .cycles = NULL,
                            .cycle_count = count,
                        };
                        optarg = (char*)parsed;
                        goto powerloss_next;
                    }

powerloss_unknown:
                    // unknown scenario?
                    fprintf(stderr, "error: unknown power-loss scenario: %s\n",
                            optarg);
                    exit(-1);

powerloss_next:
                    optarg += strspn(optarg, " ");
                    if (*optarg == ',') {
                        optarg += 1;
                    } else if (*optarg == '\0') {
                        break;
                    } else {
                        goto powerloss_unknown;
                    }
                }
                break;
            }
            case OPT_STEP: {
                char *parsed = NULL;
                test_step_start = strtoumax(optarg, &parsed, 0);
                test_step_stop = -1;
                test_step_step = 1;
                // allow empty string for start=0
                if (parsed == optarg) {
                    test_step_start = 0;
                }
                optarg = parsed + strspn(parsed, " ");

                if (*optarg != ',' && *optarg != '\0') {
                    goto step_unknown;
                }

                if (*optarg == ',') {
                    optarg += 1;
                    test_step_stop = strtoumax(optarg, &parsed, 0);
                    // allow empty string for stop=end
                    if (parsed == optarg) {
                        test_step_stop = -1;
                    }
                    optarg = parsed + strspn(parsed, " ");

                    if (*optarg != ',' && *optarg != '\0') {
                        goto step_unknown;
                    }

                    if (*optarg == ',') {
                        optarg += 1;
                        test_step_step = strtoumax(optarg, &parsed, 0);
                        // allow empty string for stop=1
                        if (parsed == optarg) {
                            test_step_step = 1;
                        }
                        optarg = parsed + strspn(parsed, " ");

                        if (*optarg != '\0') {
                            goto step_unknown;
                        }
                    }
                } else {
                    // single value = stop only
                    test_step_stop = test_step_start;
                    test_step_start = 0;
                }

                break;
step_unknown:
                fprintf(stderr, "error: invalid step: %s\n", optarg);
                exit(-1);
            }
            case OPT_DISK:
                test_disk_path = optarg;
                break;
            case OPT_TRACE:
                test_trace_path = optarg;
                break;
            case OPT_TRACE_BACKTRACE:
                test_trace_backtrace = true;
                break;
            case OPT_TRACE_PERIOD: {
                char *parsed = NULL;
                test_trace_period = strtoumax(optarg, &parsed, 0);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid trace-period: %s\n", optarg);
                    exit(-1);
                }
                break;
            }
            case OPT_TRACE_FREQ: {
                char *parsed = NULL;
                test_trace_freq = strtoumax(optarg, &parsed, 0);
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
                test_read_sleep = read_sleep*1.0e9;
                break;
            }
            case OPT_PROG_SLEEP: {
                char *parsed = NULL;
                double prog_sleep = strtod(optarg, &parsed);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid prog-sleep: %s\n", optarg);
                    exit(-1);
                }
                test_prog_sleep = prog_sleep*1.0e9;
                break;
            }
            case OPT_ERASE_SLEEP: {
                char *parsed = NULL;
                double erase_sleep = strtod(optarg, &parsed);
                if (parsed == optarg) {
                    fprintf(stderr, "error: invalid erase-sleep: %s\n", optarg);
                    exit(-1);
                }
                test_erase_sleep = erase_sleep*1.0e9;
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
        // reset our test identifier list
        test_ids = NULL;
        test_id_count = 0;
        test_id_capacity = 0;
    }

    // parse test identifier, if any, cannibalizing the arg in the process
    for (; argc > optind; optind++) {
        test_define_t *defines = NULL;
        size_t define_count = 0;
        lfs_emubd_powercycles_t *cycles = NULL;
        size_t cycle_count = 0;

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
            char *cycles_ = strchr(defines_, ':');
            if (cycles_) {
                *cycles_ = '\0';
                cycles_ += 1;
            }

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
                            ncount*sizeof(test_define_t));
                    memset(defines+define_count, 0,
                            (ncount-define_count)*sizeof(test_define_t));
                    define_count = ncount;
                }
                defines[d] = TEST_LIT(v);
            }

            if (cycles_) {
                // parse power cycles
                size_t cycle_capacity = 0;
                while (*cycles_ != '\0') {
                    char *parsed = NULL;
                    *(lfs_emubd_powercycles_t*)mappend(
                            (void**)&cycles,
                            sizeof(lfs_emubd_powercycles_t),
                            &cycle_count,
                            &cycle_capacity)
                            = leb16_parse(cycles_, &parsed);
                    if (parsed == cycles_) {
                        fprintf(stderr, "error: "
                                "could not parse test cycles: %s\n",
                                cycles_);
                        exit(-1);
                    }
                    cycles_ = parsed;
                }
            }
        }

        // append to identifier list
        *(test_id_t*)mappend(
                (void**)&test_ids,
                sizeof(test_id_t),
                &test_id_count,
                &test_id_capacity) = (test_id_t){
            .name = name,
            .defines = defines,
            .define_count = define_count,
            .cycles = cycles,
            .cycle_count = cycle_count,
        };
    }

    // do the thing
    op();

    // cleanup (need to be done for valgrind testing)
    test_define_cleanup();
    if (test_overrides) {
        for (size_t i = 0; i < test_override_count; i++) {
            free((void*)test_overrides[i].defines);
        }
        free((void*)test_overrides);
    }
    if (test_geometry_capacity) {
        free((void*)test_geometries);
    }
    if (test_powerloss_capacity) {
        for (size_t i = 0; i < test_powerloss_count; i++) {
            free((void*)test_powerlosses[i].cycles);
        }
        free((void*)test_powerlosses);
    }
    if (test_id_capacity) {
        for (size_t i = 0; i < test_id_count; i++) {
            free((void*)test_ids[i].defines);
            free((void*)test_ids[i].cycles);
        }
        free((void*)test_ids);
    }
}

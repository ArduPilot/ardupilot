/*
 * Runner for littlefs benchmarks
 *
 * Copyright (c) 2022, The littlefs authors.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef BENCH_RUNNER_H
#define BENCH_RUNNER_H


// override LFS_TRACE
void bench_trace(const char *fmt, ...);

#define LFS_TRACE_(fmt, ...) \
    bench_trace("%s:%d:trace: " fmt "%s\n", \
        __FILE__, \
        __LINE__, \
        __VA_ARGS__)
#define LFS_TRACE(...) LFS_TRACE_(__VA_ARGS__, "")
#define LFS_EMUBD_TRACE(...) LFS_TRACE_(__VA_ARGS__, "")

// provide BENCH_START/BENCH_STOP macros
void bench_start(void);
void bench_stop(void);

#define BENCH_START() bench_start()
#define BENCH_STOP() bench_stop()


// note these are indirectly included in any generated files
#include "bd/lfs_emubd.h"
#include <stdio.h>

// give source a chance to define feature macros
#undef _FEATURES_H
#undef _STDIO_H


// generated bench configurations
struct lfs_config;

enum bench_flags {
    BENCH_REENTRANT = 0x1,
};
typedef uint8_t bench_flags_t;

typedef struct bench_define {
    intmax_t (*cb)(void *data);
    void *data;
} bench_define_t;

struct bench_case {
    const char *name;
    const char *path;
    bench_flags_t flags;
    size_t permutations;

    const bench_define_t *defines;

    bool (*filter)(void);
    void (*run)(struct lfs_config *cfg);
};

struct bench_suite {
    const char *name;
    const char *path;
    bench_flags_t flags;

    const char *const *define_names;
    size_t define_count;

    const struct bench_case *cases;
    size_t case_count;
};


// deterministic prng for pseudo-randomness in benches
uint32_t bench_prng(uint32_t *state);

#define BENCH_PRNG(state) bench_prng(state)


// access generated bench defines
intmax_t bench_define(size_t define);

#define BENCH_DEFINE(i) bench_define(i)

// a few preconfigured defines that control how benches run
 
#define READ_SIZE_i          0
#define PROG_SIZE_i          1
#define ERASE_SIZE_i         2
#define ERASE_COUNT_i        3
#define BLOCK_SIZE_i         4
#define BLOCK_COUNT_i        5
#define CACHE_SIZE_i         6
#define LOOKAHEAD_SIZE_i     7
#define COMPACT_THRESH_i     8
#define METADATA_MAX_i       9
#define INLINE_MAX_i         10
#define BLOCK_CYCLES_i       11
#define ERASE_VALUE_i        12
#define ERASE_CYCLES_i       13
#define BADBLOCK_BEHAVIOR_i  14
#define POWERLOSS_BEHAVIOR_i 15

#define READ_SIZE           bench_define(READ_SIZE_i)
#define PROG_SIZE           bench_define(PROG_SIZE_i)
#define ERASE_SIZE          bench_define(ERASE_SIZE_i)
#define ERASE_COUNT         bench_define(ERASE_COUNT_i)
#define BLOCK_SIZE          bench_define(BLOCK_SIZE_i)
#define BLOCK_COUNT         bench_define(BLOCK_COUNT_i)
#define CACHE_SIZE          bench_define(CACHE_SIZE_i)
#define LOOKAHEAD_SIZE      bench_define(LOOKAHEAD_SIZE_i)
#define COMPACT_THRESH      bench_define(COMPACT_THRESH_i)
#define METADATA_MAX        bench_define(METADATA_MAX_i)
#define INLINE_MAX          bench_define(INLINE_MAX_i)
#define BLOCK_CYCLES        bench_define(BLOCK_CYCLES_i)
#define ERASE_VALUE         bench_define(ERASE_VALUE_i)
#define ERASE_CYCLES        bench_define(ERASE_CYCLES_i)
#define BADBLOCK_BEHAVIOR   bench_define(BADBLOCK_BEHAVIOR_i)
#define POWERLOSS_BEHAVIOR  bench_define(POWERLOSS_BEHAVIOR_i)

#define BENCH_IMPLICIT_DEFINES \
    BENCH_DEF(READ_SIZE,          PROG_SIZE) \
    BENCH_DEF(PROG_SIZE,          ERASE_SIZE) \
    BENCH_DEF(ERASE_SIZE,         0) \
    BENCH_DEF(ERASE_COUNT,        (1024*1024)/BLOCK_SIZE) \
    BENCH_DEF(BLOCK_SIZE,         ERASE_SIZE) \
    BENCH_DEF(BLOCK_COUNT,        ERASE_COUNT/lfs_max(BLOCK_SIZE/ERASE_SIZE,1))\
    BENCH_DEF(CACHE_SIZE,         lfs_max(64,lfs_max(READ_SIZE,PROG_SIZE))) \
    BENCH_DEF(LOOKAHEAD_SIZE,     16) \
    BENCH_DEF(COMPACT_THRESH,     0) \
    BENCH_DEF(METADATA_MAX,       0) \
    BENCH_DEF(INLINE_MAX,         0) \
    BENCH_DEF(BLOCK_CYCLES,       -1) \
    BENCH_DEF(ERASE_VALUE,        0xff) \
    BENCH_DEF(ERASE_CYCLES,       0) \
    BENCH_DEF(BADBLOCK_BEHAVIOR,  LFS_EMUBD_BADBLOCK_PROGERROR) \
    BENCH_DEF(POWERLOSS_BEHAVIOR, LFS_EMUBD_POWERLOSS_NOOP)

#define BENCH_GEOMETRY_DEFINE_COUNT 4
#define BENCH_IMPLICIT_DEFINE_COUNT 16


#endif

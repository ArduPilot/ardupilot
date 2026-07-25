/*
 * Runner for littlefs tests
 *
 * Copyright (c) 2022, The littlefs authors.
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H


// override LFS_TRACE
void test_trace(const char *fmt, ...);

#define LFS_TRACE_(fmt, ...) \
    test_trace("%s:%d:trace: " fmt "%s\n", \
        __FILE__, \
        __LINE__, \
        __VA_ARGS__)
#define LFS_TRACE(...) LFS_TRACE_(__VA_ARGS__, "")
#define LFS_EMUBD_TRACE(...) LFS_TRACE_(__VA_ARGS__, "")


// note these are indirectly included in any generated files
#include "bd/lfs_emubd.h"
#include <stdio.h>

// give source a chance to define feature macros
#undef _FEATURES_H
#undef _STDIO_H


// generated test configurations
struct lfs_config;

enum test_flags {
    TEST_REENTRANT = 0x1,
};
typedef uint8_t test_flags_t;

typedef struct test_define {
    intmax_t (*cb)(void *data);
    void *data;
} test_define_t;

struct test_case {
    const char *name;
    const char *path;
    test_flags_t flags;
    size_t permutations;

    const test_define_t *defines;

    bool (*filter)(void);
    void (*run)(struct lfs_config *cfg);
};

struct test_suite {
    const char *name;
    const char *path;
    test_flags_t flags;

    const char *const *define_names;
    size_t define_count;

    const struct test_case *cases;
    size_t case_count;
};


// deterministic prng for pseudo-randomness in testes
uint32_t test_prng(uint32_t *state);

#define TEST_PRNG(state) test_prng(state)


// access generated test defines
intmax_t test_define(size_t define);

#define TEST_DEFINE(i) test_define(i)

// a few preconfigured defines that control how tests run
 
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
#define DISK_VERSION_i       16

#define READ_SIZE           TEST_DEFINE(READ_SIZE_i)
#define PROG_SIZE           TEST_DEFINE(PROG_SIZE_i)
#define ERASE_SIZE          TEST_DEFINE(ERASE_SIZE_i)
#define ERASE_COUNT         TEST_DEFINE(ERASE_COUNT_i)
#define BLOCK_SIZE          TEST_DEFINE(BLOCK_SIZE_i)
#define BLOCK_COUNT         TEST_DEFINE(BLOCK_COUNT_i)
#define CACHE_SIZE          TEST_DEFINE(CACHE_SIZE_i)
#define LOOKAHEAD_SIZE      TEST_DEFINE(LOOKAHEAD_SIZE_i)
#define COMPACT_THRESH      TEST_DEFINE(COMPACT_THRESH_i)
#define METADATA_MAX        TEST_DEFINE(METADATA_MAX_i)
#define INLINE_MAX          TEST_DEFINE(INLINE_MAX_i)
#define BLOCK_CYCLES        TEST_DEFINE(BLOCK_CYCLES_i)
#define ERASE_VALUE         TEST_DEFINE(ERASE_VALUE_i)
#define ERASE_CYCLES        TEST_DEFINE(ERASE_CYCLES_i)
#define BADBLOCK_BEHAVIOR   TEST_DEFINE(BADBLOCK_BEHAVIOR_i)
#define POWERLOSS_BEHAVIOR  TEST_DEFINE(POWERLOSS_BEHAVIOR_i)
#define DISK_VERSION        TEST_DEFINE(DISK_VERSION_i)

#define TEST_IMPLICIT_DEFINES \
    TEST_DEF(READ_SIZE,          PROG_SIZE) \
    TEST_DEF(PROG_SIZE,          ERASE_SIZE) \
    TEST_DEF(ERASE_SIZE,         0) \
    TEST_DEF(ERASE_COUNT,        (1024*1024)/ERASE_SIZE) \
    TEST_DEF(BLOCK_SIZE,         ERASE_SIZE) \
    TEST_DEF(BLOCK_COUNT,        ERASE_COUNT/lfs_max(BLOCK_SIZE/ERASE_SIZE,1)) \
    TEST_DEF(CACHE_SIZE,         lfs_max(64,lfs_max(READ_SIZE,PROG_SIZE))) \
    TEST_DEF(LOOKAHEAD_SIZE,     16) \
    TEST_DEF(COMPACT_THRESH,     0) \
    TEST_DEF(METADATA_MAX,       0) \
    TEST_DEF(INLINE_MAX,         0) \
    TEST_DEF(BLOCK_CYCLES,       -1) \
    TEST_DEF(ERASE_VALUE,        0xff) \
    TEST_DEF(ERASE_CYCLES,       0) \
    TEST_DEF(BADBLOCK_BEHAVIOR,  LFS_EMUBD_BADBLOCK_PROGERROR) \
    TEST_DEF(POWERLOSS_BEHAVIOR, LFS_EMUBD_POWERLOSS_NOOP) \
    TEST_DEF(DISK_VERSION,       0)

#define TEST_GEOMETRY_DEFINE_COUNT 4
#define TEST_IMPLICIT_DEFINE_COUNT 17


#endif

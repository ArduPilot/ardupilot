/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"
#include "oslib_test_root.h"

/**
 * @file    oslib_test_sequence_006.c
 * @brief   Test Sequence 006 code.
 *
 * @page oslib_test_sequence_006 [6] Objects Caches
 *
 * File: @ref oslib_test_sequence_006.c
 *
 * <h2>Description</h2>
 * This sequence tests the ChibiOS library functionalities related to
 * Objects Caches.
 *
 * <h2>Conditions</h2>
 * This sequence is only executed if the following preprocessor condition
 * evaluates to true:
 * - CH_CFG_USE_OBJ_CACHES == TRUE
 * .
 *
 * <h2>Test Cases</h2>
 * - @subpage oslib_test_006_001
 * .
 */

#if (CH_CFG_USE_OBJ_CACHES == TRUE) || defined(__DOXYGEN__)

/****************************************************************************
 * Shared code.
 ****************************************************************************/

#include <string.h>

#define SIZE_OBJECTS        16
#define NUM_OBJECTS         4
#define NUM_HASH_ENTRIES    (NUM_OBJECTS * 2)

/* Cached object type used for test.*/
typedef struct {
  oc_object_t       header;
  uint8_t           data[SIZE_OBJECTS];
} cached_object_t;

static oc_hash_header_t hash_headers[NUM_HASH_ENTRIES];
static cached_object_t objects[NUM_OBJECTS];
static objects_cache_t cache1;

static bool obj_read(objects_cache_t *ocp,
                     oc_object_t *objp,
                     bool async) {

  test_emit_token('a' + objp->obj_key);

  objp->obj_flags &= ~OC_FLAG_NOTSYNC;

  if (async) {
    chCacheReleaseObject(ocp, objp);
  }

  return false;
}

static bool obj_write(objects_cache_t *ocp,
                      oc_object_t *objp,
                      bool async) {
  (void)ocp;
  (void)async;

  test_emit_token('A' + objp->obj_key);

  return false;
}

/****************************************************************************
 * Test cases.
 ****************************************************************************/

/**
 * @page oslib_test_006_001 [6.1] Cache initialization
 *
 * <h2>Description</h2>
 * A cache object is initialized, some initial conditions are checked.
 *
 * <h2>Test Steps</h2>
 * - [6.1.1] Cache initialization.
 * - [6.1.2] Getting and releasing objects without initialization.
 * - [6.1.3] Getting and releasing objects with synchronous
 *   initialization.
 * - [6.1.4] Getting and releasing objects with asynchronous
 *   initialization.
 * - [6.1.5] Checking cached objects.
 * - [6.1.6] Checking non-cached objects.
 * .
 */

static void oslib_test_006_001_execute(void) {

  /* [6.1.1] Cache initialization.*/
  test_set_step(1);
  {
    chCacheObjectInit(&cache1,
                      NUM_HASH_ENTRIES,
                      hash_headers,
                      NUM_OBJECTS,
                      sizeof (cached_object_t),
                      objects,
                      obj_read,
                      obj_write);
  }
  test_end_step(1);

  /* [6.1.2] Getting and releasing objects without initialization.*/
  test_set_step(2);
  {
    uint32_t i;

    for (i = 0; i < (NUM_OBJECTS * 2); i++) {
      oc_object_t * objp = chCacheGetObject(&cache1, 0U, i);

      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) != 0U, "should not be in sync");

      chCacheReleaseObject(&cache1, objp);
    }

    test_assert_sequence("", "unexpected tokens");
  }
  test_end_step(2);

  /* [6.1.3] Getting and releasing objects with synchronous
     initialization.*/
  test_set_step(3);
  {
    uint32_t i;
    bool error;

    for (i = 0; i < (NUM_OBJECTS * 2); i++) {
      oc_object_t *objp = chCacheGetObject(&cache1, 0U, i);

      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) != 0U, "in sync");

      error = chCacheReadObject(&cache1, objp, false);

      test_assert(error == false, "returned error");
      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) == 0U, "not in sync");

      chCacheReleaseObject(&cache1, objp);
    }

    test_assert_sequence("abcdefgh", "unexpected tokens");
  }
  test_end_step(3);

  /* [6.1.4] Getting and releasing objects with asynchronous
     initialization.*/
  test_set_step(4);
  {
    uint32_t i;
    bool error;

    for (i = 0; i < (NUM_OBJECTS * 2); i++) {
      oc_object_t *objp = chCacheGetObject(&cache1, 0U, i);

      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) != 0U, "in sync");

      error = chCacheReadObject(&cache1, objp, true);

      test_assert(error == false, "returned error");

      objp = chCacheGetObject(&cache1, 0U, i);

      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) == 0U, "not in sync");

      chCacheReleaseObject(&cache1, objp);
    }

    test_assert_sequence("abcdefgh", "unexpected tokens");
  }
  test_end_step(4);

  /* [6.1.5] Checking cached objects.*/
  test_set_step(5);
  {
    uint32_t i;

    for (i = NUM_OBJECTS; i < (NUM_OBJECTS * 2); i++) {
      oc_object_t *objp = chCacheGetObject(&cache1, 0U, i);

      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) == 0U, "not in sync");

      chCacheReleaseObject(&cache1, objp);
    }

    test_assert_sequence("", "unexpected tokens");
  }
  test_end_step(5);

  /* [6.1.6] Checking non-cached objects.*/
  test_set_step(6);
  {
    uint32_t i;

    for (i = 0; i < NUM_OBJECTS; i++) {
      oc_object_t *objp = chCacheGetObject(&cache1, 0U, i);

      test_assert((objp->obj_flags & OC_FLAG_INHASH) != 0U, "not in hash");
      test_assert((objp->obj_flags & OC_FLAG_NOTSYNC) != 0U, "in sync");

      chCacheReleaseObject(&cache1, objp);
    }

    test_assert_sequence("", "unexpected tokens");
  }
  test_end_step(6);
}

static const testcase_t oslib_test_006_001 = {
  "Cache initialization",
  NULL,
  NULL,
  oslib_test_006_001_execute
};

/****************************************************************************
 * Exported data.
 ****************************************************************************/

/**
 * @brief   Array of test cases.
 */
const testcase_t * const oslib_test_sequence_006_array[] = {
  &oslib_test_006_001,
  NULL
};

/**
 * @brief   Objects Caches.
 */
const testsequence_t oslib_test_sequence_006 = {
  "Objects Caches",
  oslib_test_sequence_006_array
};

#endif /* CH_CFG_USE_OBJ_CACHES == TRUE */

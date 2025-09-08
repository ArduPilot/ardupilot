#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <stdint.h>

#define ASCON_80PQ_VARIANT 0
#define ASCON_AEAD_VARIANT 1
#define ASCON_HASH_VARIANT 2
#define ASCON_XOF_VARIANT 3
#define ASCON_CXOF_VARIANT 4
#define ASCON_MAC_VARIANT 5
#define ASCON_PRF_VARIANT 6
#define ASCON_PRFS_VARIANT 7

#define ASCON_TAG_SIZE 16
#define ASCON_HASH_SIZE 32

#define ASCON_128_RATE 8
#define ASCON_128A_RATE 16
#define ASCON_HASH_RATE 8
#define ASCON_PRF_IN_RATE 32
#define ASCON_PRFA_IN_RATE 40
#define ASCON_PRF_OUT_RATE 16

#define ASCON_PA_ROUNDS 12
#define ASCON_128_PB_ROUNDS 6
#define ASCON_128A_PB_ROUNDS 8
#define ASCON_HASH_PB_ROUNDS 12
#define ASCON_HASHA_PB_ROUNDS 8
#define ASCON_PRF_PB_ROUNDS 12
#define ASCON_PRFA_PB_ROUNDS 8

#define ASCON_128_IV                         \
  (((uint64_t)(ASCON_AEAD_VARIANT) << 0) |   \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |     \
   ((uint64_t)(ASCON_128_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_TAG_SIZE * 8) << 24) |  \
   ((uint64_t)(ASCON_128_RATE) << 40))

#define ASCON_128A_IV                         \
  (((uint64_t)(ASCON_AEAD_VARIANT) << 0) |    \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |      \
   ((uint64_t)(ASCON_128A_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_TAG_SIZE * 8) << 24) |   \
   ((uint64_t)(ASCON_128A_RATE) << 40))

#define ASCON_80PQ_IV                                                          \
  (((uint64_t)(ASCON_80PQ_VARIANT) << 0) | ((uint64_t)(ASCON_128_RATE) << 8) | \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |                                       \
   ((uint64_t)(ASCON_128_PB_ROUNDS) << 20) |                                   \
   ((uint64_t)(ASCON_TAG_SIZE * 8) << 24))

#define ASCON_HASH_IV                         \
  (((uint64_t)(ASCON_HASH_VARIANT) << 0) |    \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |      \
   ((uint64_t)(ASCON_HASH_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_HASH_SIZE * 8) << 24) |  \
   ((uint64_t)(ASCON_HASH_RATE) << 40))

#define ASCON_HASHA_IV                         \
  (((uint64_t)(ASCON_HASH_VARIANT) << 0) |     \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |       \
   ((uint64_t)(ASCON_HASHA_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_HASH_SIZE * 8) << 24) |   \
   ((uint64_t)(ASCON_HASH_RATE) << 40))

#define ASCON_XOF_IV                          \
  (((uint64_t)(ASCON_XOF_VARIANT) << 0) |     \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |      \
   ((uint64_t)(ASCON_HASH_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_HASH_RATE) << 40))

#define ASCON_XOFA_IV                          \
  (((uint64_t)(ASCON_XOF_VARIANT) << 0) |      \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |       \
   ((uint64_t)(ASCON_HASHA_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_HASH_RATE) << 40))

#define ASCON_CXOF_IV                         \
  (((uint64_t)(ASCON_CXOF_VARIANT) << 0) |    \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |      \
   ((uint64_t)(ASCON_HASH_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_HASH_RATE) << 40))

#define ASCON_CXOFA_IV                         \
  (((uint64_t)(ASCON_CXOF_VARIANT) << 0) |     \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |       \
   ((uint64_t)(ASCON_HASHA_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_HASH_RATE) << 40))

#define ASCON_MAC_IV                         \
  (((uint64_t)(ASCON_MAC_VARIANT) << 0) |    \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |     \
   ((uint64_t)(ASCON_PRF_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_TAG_SIZE * 8) << 24) |  \
   ((uint64_t)(ASCON_PRF_IN_RATE) << 40) |   \
   ((uint64_t)(ASCON_PRF_OUT_RATE) << 48))

#define ASCON_MACA_IV                         \
  (((uint64_t)(ASCON_MAC_VARIANT) << 0) |     \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |      \
   ((uint64_t)(ASCON_PRFA_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_TAG_SIZE * 8) << 24) |   \
   ((uint64_t)(ASCON_PRFA_IN_RATE) << 40) |   \
   ((uint64_t)(ASCON_PRF_OUT_RATE) << 48))

#define ASCON_PRF_IV                         \
  (((uint64_t)(ASCON_PRF_VARIANT) << 0) |    \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |     \
   ((uint64_t)(ASCON_PRF_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_PRF_IN_RATE) << 40) |   \
   ((uint64_t)(ASCON_PRF_OUT_RATE) << 48))

#define ASCON_PRFA_IV                         \
  (((uint64_t)(ASCON_PRF_VARIANT) << 0) |     \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |      \
   ((uint64_t)(ASCON_PRFA_PB_ROUNDS) << 20) | \
   ((uint64_t)(ASCON_PRFA_IN_RATE) << 40) |   \
   ((uint64_t)(ASCON_PRF_OUT_RATE) << 48))

#define ASCON_PRFS_IV                      \
  (((uint64_t)(ASCON_PRFS_VARIANT) << 0) | \
   ((uint64_t)(ASCON_PA_ROUNDS) << 16) |   \
   ((uint64_t)(ASCON_TAG_SIZE * 8) << 24))

#endif /* CONSTANTS_H_ */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// AP_FATFS_CRYPTO_ENABLED is the integration switch. It is normally set with
// "define AP_FATFS_CRYPTO_ENABLED 1" in a board's hwdef.dat. When set it turns
// on transparent full-disk AES-256-XTS at the FatFS diskio layer, so every SD
// card sector (including FAT metadata) is encrypted on the media.
#ifndef AP_FATFS_CRYPTO_ENABLED
#define AP_FATFS_CRYPTO_ENABLED 0
#endif

// AP_DISKCRYPTO_ENABLED compiles the AES-256-XTS engine itself. It follows the
// integration switch, and is additionally always enabled on SITL so the engine
// can be exercised by unit tests and host tooling.
#ifndef AP_DISKCRYPTO_ENABLED
#define AP_DISKCRYPTO_ENABLED (AP_FATFS_CRYPTO_ENABLED || (CONFIG_HAL_BOARD == HAL_BOARD_SITL))
#endif

// Length of the AES-256-XTS key in bytes: two independent 256-bit keys, the
// first used for the data cipher and the second for the tweak cipher.
#define AP_DISKCRYPTO_KEY_LEN 64

// Optional hardware-accelerated block-cipher backend (e.g. STM32 CRYP). None is
// provided yet; the software backend is always the fallback. The backend factory
// in AP_DiskCrypto_Backend.cpp is the single place to wire one in.
#ifndef AP_DISKCRYPTO_HW_ENABLED
#define AP_DISKCRYPTO_HW_ENABLED 0
#endif

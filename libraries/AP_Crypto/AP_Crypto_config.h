#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_CRYPTO_ENABLED
#define AP_CRYPTO_ENABLED 1
#endif

// Fernet-compatible encryption (uses ChaCha20-Poly1305 instead of AES-128-CBC)
// Note: Not 100% compatible with Python's Fernet, but provides similar functionality
#ifndef AP_CRYPTO_FERNET_ENABLED
#define AP_CRYPTO_FERNET_ENABLED AP_CRYPTO_ENABLED
#endif


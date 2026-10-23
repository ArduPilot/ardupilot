/*
  implement a Float16 type
 */

#pragma once
#include <stdint.h>

struct float16_s {
    float get(void) const;
    void set(float value);

    uint16_t v16;
};

typedef struct float16_s Float16_t;

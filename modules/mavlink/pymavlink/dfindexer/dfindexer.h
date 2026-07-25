#ifndef DFINDEXER_H
#define DFINDEXER_H

#include <stdint.h>
#include <stddef.h>
#include <Python.h>

typedef struct OffsetArray {
    uint64_t *data;
    size_t len;
    size_t cap;
} OffsetArray;

OffsetArray* scan_offsets(const uint8_t *data, size_t len,
                          uint8_t fmt_type, uint8_t fmt_length,
                          uint8_t type_offset, uint8_t length_offset,
                          uint8_t head1, uint8_t head2,
                          PyObject *progress_callback);

void free_offsets(OffsetArray *offsets);

#endif

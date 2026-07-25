#include "dfindexer.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>

#define NUM_TYPES 256
#define INITIAL_CAP 1024

// Add a #if, since windows doesn't have __attribute__((noreturn))
#if defined(_WIN32) || defined(_WIN64)
#define NORETURN
#else
#define NORETURN __attribute__((noreturn))
#endif
static void panic(const char *msg, ...) NORETURN;
static void panic(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    vfprintf(stderr, msg, args);
    va_end(args);
    fprintf(stderr, "\n");
    exit(EXIT_FAILURE);
}

static void ensure_capacity(OffsetArray *arr) {
    if (arr->len >= arr->cap) {
        size_t new_cap = arr->cap == 0 ? INITIAL_CAP : arr->cap * 2;
        uint64_t *new_data = realloc(arr->data, new_cap * sizeof(uint64_t));
        if (!new_data) panic("Memory allocation failed");
        arr->data = new_data;
        arr->cap = new_cap;
    }
}

static bool call_progress_callback(PyObject *callback, int percent) {
    PyObject *arg = PyLong_FromLong(percent);
    PyObject *res = PyObject_CallFunctionObjArgs(callback, arg, NULL);
    Py_DECREF(arg);
    if (!res) {
        if (PyErr_Occurred()) {
            PyErr_Print();
        }
        return false;
    }
    Py_DECREF(res);
    return true;
}

static int last_percent = -1;

OffsetArray* scan_offsets(const uint8_t *data, size_t len,
                          uint8_t fmt_type, uint8_t fmt_length,
                          uint8_t type_offset, uint8_t length_offset,
                          uint8_t head1, uint8_t head2,
                          PyObject *progress_callback) {
    uint8_t lengths[NUM_TYPES] = {0};
    OffsetArray *results = calloc(NUM_TYPES, sizeof(OffsetArray));
    if (!results) panic("Memory allocation failed");

    // Make an a-priori assumption about the FMT message length, since
    // sometimes, somehow, the FMT for FMT ends up NOT being the first message
    // in the file.
    lengths[fmt_type] = fmt_length;

    size_t i = 0;
    size_t loop_count = 0;
    while (i + 3 < len) {
        // Allow SIGINT to interrupt the loop
        if ((++loop_count & 0xFFFF) == 0) {
            if (PyErr_CheckSignals()) {
                free_offsets(results);
                fprintf(stderr, "scan_offsets interrupted!\n");
                return NULL;
            }
        }
        // Even less frequently, check if we should call the progress callback
        if (progress_callback && (loop_count & 0x3FFFFF) == 0) {
            int percent = (int)((i * 100) / len);
            if (percent != last_percent) {
                last_percent = percent;
                if (!call_progress_callback(progress_callback, percent)) {
                    free_offsets(results);
                    fprintf(stderr, "Error calling progress callback\n");
                    return NULL;
                }
            }
        }
        if (data[i] != head1 || data[i + 1] != head2) {
            if (len - i >= 528 || len < 528) {
                fprintf(stderr, "bad header 0x%02x%02x at %zu\n", data[i], data[i + 1], i);
            }
            i++;
            continue;
        }

        uint8_t mtype = data[i + 2];

        // If this is an FMT message, populate the lengths array
        if (mtype == fmt_type) {
            uint8_t type_in_fmt = data[i + type_offset];
            uint8_t len_in_fmt = data[i + length_offset];
            if (len_in_fmt < 3) {
                panic("Invalid length in FMT message at %zu", i);
            }
            lengths[type_in_fmt] = len_in_fmt;
        }

        uint8_t mlen = lengths[mtype];
        if (mlen == 0) {
            panic("Invalid length in FMT message at %zu", i);
        }

        OffsetArray *arr = &results[mtype];
        ensure_capacity(arr);
        arr->data[arr->len++] = i;
        i += mlen;
    }
    // Fire the progress callback one last time
    if (progress_callback) {
        call_progress_callback(progress_callback, 100);
    }

    return results;
}

void free_offsets(OffsetArray *offsets)
{
    if (!offsets) return;
    for (size_t i = 0; i < NUM_TYPES; i++) {
        free(offsets[i].data);
    }
    free(offsets);
}

from cpython.ref cimport PyObject, Py_INCREF, Py_DECREF

cdef extern from "dfindexer.h":
    cdef struct OffsetArray:
        unsigned long long* data
        size_t len
        size_t cap

    OffsetArray* scan_offsets(const unsigned char* data, size_t len,
                              unsigned char fmt_type, unsigned char fmt_length,
                              unsigned char type_offset, unsigned char length_offset,
                              unsigned char head1, unsigned char head2,
                              PyObject* progress_callback)

    void free_offsets(OffsetArray* offsets)

def build_offsets(const unsigned char[:] data,
                  unsigned char fmt_type, unsigned char fmt_length,
                  unsigned char type_offset, unsigned char length_offset,
                  unsigned char head1, unsigned char head2,
                  progress_callback=None):
    cdef OffsetArray* results
    cdef size_t i, j
    cdef list py_offsets = []

    cdef PyObject* cb = NULL
    if progress_callback is not None:
        Py_INCREF(progress_callback)  # ensure it stays alive during the C call
        cb = <PyObject*>progress_callback

    results = scan_offsets(&data[0], data.shape[0],
                           fmt_type, fmt_length,
                           type_offset, length_offset,
                           head1, head2,
                           cb)

    if progress_callback is not None:
        Py_DECREF(progress_callback)  # clean up the reference

    if results == NULL:
        raise MemoryError("scan_offsets returned NULL")

    for i in range(256):
        entry = results[i]
        py_offsets.append([entry.data[j] for j in range(entry.len)])

    free_offsets(results)
    return py_offsets

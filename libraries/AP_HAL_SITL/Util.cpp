#include "Util.h"
#include <sys/time.h>

#ifdef WITH_SITL_TONEALARM
HALSITL::ToneAlarm_SF HALSITL::Util::_toneAlarm;
#endif

uint64_t HALSITL::Util::get_hw_rtc() const
{
#ifndef CLOCK_REALTIME
    struct timeval ts;
    gettimeofday(&ts, nullptr);
    return ((long long)((ts.tv_sec * 1000000) + ts.tv_usec));
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    const uint64_t seconds = ts.tv_sec;
    const uint64_t nanoseconds = ts.tv_nsec;
    return (seconds * 1000000ULL + nanoseconds/1000ULL);
#endif
}

/*
  get a (hopefully unique) machine ID
 */
bool HALSITL::Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    char *cbuf = (char *)buf;

    // try first to use machine-id file. Most systems will have this
    const char *paths[] = { "/etc/machine-id", "/var/lib/dbus/machine-id" };
    for (uint8_t i=0; i<ARRAY_SIZE(paths); i++) {
        int fd = open(paths[i], O_RDONLY);
        if (fd == -1) {
            continue;
        }
        ssize_t ret = read(fd, buf, len);
        close(fd);
        if (ret <= 0) {
            continue;
        }
        if (ret == len) {
            cbuf[len-1] = '\0';
        } else {
            cbuf[ret] = '\0';
        }
        len = ret;
        char *p = strchr(cbuf, '\n');
        if (p) {
            *p = 0;
        }
        len = strnlen(cbuf, len);
        return true;
    }

    // fallback to hostname
    if (gethostname(cbuf, len) != 0) {
        // use a default name so this always succeeds. Without it we can't
        // implement some features (such as UAVCAN)
        strncpy(cbuf, "sitl-unknown", len);
    }
    len = strnlen(cbuf, len);
    return true;
}

/*
  as get_system_id_unformatted will already be ascii, we use the same
  ID here
 */
bool HALSITL::Util::get_system_id(char buf[40])
{
    uint8_t len = 40;
    return get_system_id_unformatted((uint8_t *)buf, len);
}

#ifdef ENABLE_HEAP
void *HALSITL::Util::allocate_heap_memory(size_t size)
{
    struct heap *new_heap = (struct heap*)malloc(sizeof(struct heap));
    if (new_heap != nullptr) {
        new_heap->scripting_max_heap_size = size;
        new_heap->current_heap_usage = 0;
    }
    return (void *)new_heap;
}

void *HALSITL::Util::heap_realloc(void *heap_ptr, void *ptr, size_t new_size)
{
    if (heap_ptr == nullptr) {
        return nullptr;
    }

    struct heap *heapp = (struct heap*)heap_ptr;

    // extract appropriate headers
    size_t old_size = 0;
    heap_allocation_header *old_header = nullptr;
    if (ptr != nullptr) {
        old_header = ((heap_allocation_header *)ptr) - 1;
        old_size = old_header->allocation_size;
    }

    if ((heapp->current_heap_usage + new_size - old_size) > heapp->scripting_max_heap_size) {
        // fail the allocation as we don't have the memory. Note that we don't simulate fragmentation
        return nullptr;
    }

    heapp->current_heap_usage -= old_size;
    if (new_size == 0) {
       free(old_header);
       return nullptr;
    }

    heap_allocation_header *new_header = (heap_allocation_header *)malloc(new_size + sizeof(heap_allocation_header));
    if (new_header == nullptr) {
        // total failure to allocate, this is very surprising in SITL
        return nullptr;
    }
    heapp->current_heap_usage += new_size;
    new_header->allocation_size = new_size;
    void *new_mem = new_header + 1;

    if (ptr == nullptr) {
        return new_mem;
    }
    memcpy(new_mem, ptr, old_size > new_size ? new_size : old_size);
    free(old_header);
    return new_mem;
}

#endif // ENABLE_HEAP

enum AP_HAL::Util::safety_state HALSITL::Util::safety_switch_state(void)
{
    const SITL::SITL *sitl = AP::sitl();
    if (sitl == nullptr) {
        return AP_HAL::Util::SAFETY_NONE;
    }
    return sitl->safety_switch_state();
}

#include "AP_HAL.h"
#include "Util.h"
#include "utility/print_vprintf.h"
#if defined(__APPLE__) && defined(__MACH__)
#include <sys/time.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include "ch.h"
#include "hal.h"
#else
#include <time.h>
#endif
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Math/AP_Math.h>

/* Helper class implements AP_HAL::Print so we can use utility/vprintf */
class BufferPrinter : public AP_HAL::BetterStream {
public:
    BufferPrinter(char* str, size_t size)  :
        _offs(0), _str(str), _size(size)  {}

    size_t write(uint8_t c) override {
        if (_offs < _size) {
            _str[_offs] = c;
        }
        _offs++;
        return 1;
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        size_t n = 0;
        while (size--) {
            n += write(*buffer++);
        }
        return n;
    }

    size_t _offs;
    char* const  _str;
    const size_t _size;

    uint32_t available() override { return 0; }
    bool read(uint8_t &b) override { return false; }
    uint32_t txspace() override { return 0; }
    bool discard_input() override { return false; }
};

int AP_HAL::Util::snprintf(char* str, size_t size, const char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    int res = vsnprintf(str, size, format, ap);
    va_end(ap);
    return res;
}

int AP_HAL::Util::vsnprintf(char* str, size_t size, const char *format, va_list ap)
{
    // note that size==0 must be handled as functions like vasprintf() rely on the return
    // value being the number of bytes that would be printed if there was enough space.
    BufferPrinter buf(str, size?size-1:0);
    print_vprintf(&buf, format, ap);
    // null terminate
    size_t ret = buf._offs;
    if (ret < size) {
        // if the string did fit then nul terminate
        str[ret] = '\0';
    } else if (size > 0) {
        // if it didn't fit then terminate using passed in size
        str[size-1] = 0;
    }
    return int(ret);
}

void AP_HAL::Util::set_soft_armed(const bool b)
{
    if (b != soft_armed) {
        soft_armed = b;
        last_armed_change_ms = AP_HAL::millis();
        if (!was_watchdog_reset()) {
            persistent_data.armed = b;
        }
    }
}

#if ENABLE_HEAP
/*
  default functions for heap management for lua scripting for systems
  that don't have the ability to create separable heaps
 */

struct heap_allocation_header {
    struct heap *hp;
    uint32_t allocation_size; // size of allocated block, not including this header
};

#define HEAP_MAGIC 0x5681ef9f

struct heap {
    uint32_t magic;
    uint32_t max_heap_size;
    uint32_t current_heap_usage;
};

void *AP_HAL::Util::heap_create(uint32_t size)
{
    struct heap *new_heap = (struct heap*)malloc(sizeof(struct heap));
    if (new_heap != nullptr) {
        new_heap->max_heap_size = size;
    }
    new_heap->magic = HEAP_MAGIC;
    return (void *)new_heap;
}

void AP_HAL::Util::heap_destroy(void *heap_ptr)
{
    struct heap *heapp = (struct heap*)heap_ptr;
    if (heapp->magic != HEAP_MAGIC) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (heapp->current_heap_usage != 0) {
        // lua should guarantee that there is no memory still
        // allocated when we destroy the heap. Throw an error in SITL
        // if this proves not to be true
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    }
#endif

    // free the heap structure
    free(heapp);
}

/*
  allocate memory on a specific heap
 */
void *AP_HAL::Util::heap_allocate(void *heap_ptr, uint32_t size)
{
    if (heap_ptr == nullptr || size == 0) {
        return nullptr;
    }
    struct heap *heapp = (struct heap*)heap_ptr;
    if (heapp->magic != HEAP_MAGIC) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return nullptr;
    }

    if (heapp->current_heap_usage + size > heapp->max_heap_size) {
        // fail the allocation as we don't have the memory. Note that
        // we don't simulate the fragmentation that we would get with
        // HAL_ChibiOS
        return nullptr;
    }

    auto *header = (heap_allocation_header *)malloc(size + sizeof(heap_allocation_header));
    if (header == nullptr) {
        // can't allocate the new memory
        return nullptr;
    }
    header->hp = heapp;
    header->allocation_size = size;

    heapp->current_heap_usage += size;

    return header + 1;
}

/*
  free memory from a previous heap_allocate call
 */
void AP_HAL::Util::heap_free(void *ptr)
{
    if (ptr == nullptr) {
        return;
    }
    // extract header
    auto *header = ((struct heap_allocation_header *)ptr)-1;
    auto *heapp = header->hp;
    if (heapp->magic != HEAP_MAGIC) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    const uint32_t size = header->allocation_size;
    heapp->current_heap_usage -= size;

    // free the memory
    free(header);
}

#endif // ENABLE_HEAP

/*
  common EKF Buffer class. This handles the storage buffers for EKF data to
  bring it onto the fusion time horizon
*/

#include "EKF_Buffer.h"
#include <stdlib.h>
#include <string.h>
#include <AP_InternalError/AP_InternalError.h>

// constructor
ekf_ring_buffer::ekf_ring_buffer(uint8_t _elsize) :
    elsize(_elsize),
    buffer(nullptr)
{}

bool ekf_ring_buffer::init(uint8_t _size)
{
    // allocate before freeing so a failure leaves the old buffer usable
    void *newbuf = calloc(_size, elsize);
    if (newbuf == nullptr) {
        return false;
    }
    free(buffer);
    buffer = newbuf;
    size = _size;
    reset();
    return true;
}

/*
  get buffer offset for an index
 */
void *ekf_ring_buffer::get_offset(uint8_t idx) const
{
    return (void*)(((uint8_t*)buffer)+idx*uint32_t(elsize));
}

/*
  get a reference to the timestamp for an index
 */
uint32_t ekf_ring_buffer::time_ms(uint8_t idx) const
{
    EKF_obs_element_t *el = (EKF_obs_element_t *)get_offset(idx);
    return el->time_ms;
}

/*
  Search through a ring buffer and return the newest data that is
  older than the time specified by sample_time_ms
  Returns false if no data can be found that is less than 100msec old
*/
bool ekf_ring_buffer::recall(void *element, const uint32_t sample_time_ms)
{
    bool ret = false;
    uint8_t best_index = 0;  // only valid when ret becomes true
    while (count > 0) {
        const uint32_t toldest = time_ms(oldest);
        const int32_t dt = sample_time_ms - toldest;
        const bool matches = dt >= 0 && dt < 100;
        if (matches) {
            best_index = oldest;
            ret = true;
        }
        if (dt < 0) {
            // the oldest element is younger than we want, stop
            // searching and don't consume this element
            break;
        }
        // discard the sample
        count--;
        oldest = (oldest+1) % size;
    }

    if (ret) {
        memcpy(element, get_offset(best_index), elsize);
    }

    return ret;
}

/*
 * Writes data and timestamp to a Ring buffer and advances indices that
 * define the location of the newest and oldest data
 */
void ekf_ring_buffer::push(const void *element)
{
    if (buffer == nullptr) {
        return;
    }

    // Advance head to next available index
    const uint8_t head = (oldest+count) % size;

    // New data is written at the head
    memcpy(get_offset(head), element, elsize);

    if (count < size) {
        count++;
    } else {
        oldest = (oldest+1) % size;
    }
}


// zeroes all data in the ring buffer
void ekf_ring_buffer::reset()
{
    count = 0;
    oldest = 0;
}

////////////////////////////////////////////////////
/*
  IMU buffer operations implemented separately due to different
  semantics
*/

// constructor
ekf_imu_buffer::ekf_imu_buffer(uint8_t _elsize) :
    elsize(_elsize)
{}

/*
  get buffer offset for an index
 */
void *ekf_imu_buffer::get_offset(uint8_t idx) const
{
    return (void*)(((uint8_t*)buffer)+idx*uint32_t(elsize));
}

// initialise buffer, returns false when allocation has failed
bool ekf_imu_buffer::init(uint32_t size)
{
    // allow for init twice; allocate before freeing so a failure
    // leaves the old buffer usable
    void *newbuf = calloc(size, elsize);
    if (newbuf == nullptr) {
        return false;
    }
    free(buffer);
    buffer = newbuf;
    _size = size;
    _youngest = 0;
    _oldest = 0;
    _filled = false;
    return true;
}

/*
  Writes data to a Ring buffer and advances indices that
  define the location of the newest and oldest data
*/
void ekf_imu_buffer::push_youngest_element(const void *element)
{
    if (!buffer) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return;
    }
    // push youngest to the buffer
    _youngest++;
    if (_youngest == _size) {
        _youngest = 0;
        _filled = true;
    }
    memcpy(get_offset(_youngest), element, elsize);
    // set oldest data index
    _oldest = (_youngest+1) % _size;
}

// retrieve the oldest data from the ring buffer tail
void ekf_imu_buffer::get_oldest_element(void *element)
{
    if (buffer == nullptr) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        memset(element, 0, elsize);
    } else {
        memcpy(element, get_offset(_oldest), elsize);
    }
}

// writes the same data to all elements in the ring buffer
void ekf_imu_buffer::reset_history(const void *element)
{
    for (uint8_t index=0; index<_size; index++) {
        memcpy(get_offset(index), element, elsize);
    }
}

// zeroes all data in the ring buffer
void ekf_imu_buffer::reset()
{
    _youngest = 0;
    _oldest = 0;
    memset(buffer, 0, _size*uint32_t(elsize));
}

// retrieves data from the ring buffer at a specified index
void *ekf_imu_buffer::get(uint8_t index) const
{
    return get_offset(index);
}

/*
  save and restore of complete buffer state, used by the EKF3 arming
  snapshot. The layout is indices followed by the raw element data;
  both sides must be initialised to the same size and element type.
 */
uint16_t ekf_ring_buffer::serialise_len(void) const
{
    return 3 + size*uint16_t(elsize);
}

bool ekf_ring_buffer::serialise(uint8_t *buf, uint16_t len) const
{
    if (len != serialise_len()) {
        return false;
    }
    buf[0] = size;
    buf[1] = oldest;
    buf[2] = count;
    if (size > 0) {
        memcpy(&buf[3], buffer, size*uint32_t(elsize));
    }
    return true;
}

// the stored size wins: buffer sizes derive from measured sensor rates,
// which can differ between the writer and loader of a snapshot
bool ekf_ring_buffer::deserialise(const uint8_t *buf, uint16_t len)
{
    if (len < 3) {
        return false;
    }
    const uint8_t stored_size = buf[0];
    if (len != 3 + stored_size*uint16_t(elsize)) {
        return false;
    }
    if (stored_size == 0) {
        // the writer never used this buffer; keep our capacity and
        // just clear any stale contents
        if (buffer != nullptr) {
            reset();
        }
        return true;
    }
    if (stored_size != size && !init(stored_size)) {
        return false;
    }
    if (buf[1] >= stored_size || buf[2] > stored_size) {
        return false;
    }
    oldest = buf[1];
    count = buf[2];
    memcpy(buffer, &buf[3], size*uint32_t(elsize));
    return true;
}

uint16_t ekf_imu_buffer::serialise_len(void) const
{
    return 4 + _size*uint16_t(elsize);
}

bool ekf_imu_buffer::serialise(uint8_t *buf, uint16_t len) const
{
    if (len != serialise_len()) {
        return false;
    }
    buf[0] = _size;
    buf[1] = _oldest;
    buf[2] = _youngest;
    buf[3] = _filled ? 1 : 0;
    if (_size > 0) {
        memcpy(&buf[4], buffer, _size*uint32_t(elsize));
    }
    return true;
}

bool ekf_imu_buffer::deserialise(const uint8_t *buf, uint16_t len)
{
    if (len < 4) {
        return false;
    }
    const uint8_t stored_size = buf[0];
    if (stored_size == 0 || len != 4 + stored_size*uint16_t(elsize)) {
        return false;
    }
    if (stored_size != _size && !init(stored_size)) {
        return false;
    }
    if (buf[1] >= stored_size || buf[2] >= stored_size) {
        return false;
    }
    _oldest = buf[1];
    _youngest = buf[2];
    _filled = buf[3] != 0;
    memcpy(buffer, &buf[4], _size*uint32_t(elsize));
    return true;
}

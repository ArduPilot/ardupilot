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
    elsize(_elsize)
{}

bool ekf_ring_buffer::init(uint8_t size)
{
    if (buffer) {
        free(buffer);
    }
    buffer = calloc(size, elsize);
    if (buffer == nullptr) {
        return false;
    }
    _size = size;
    _head = 0;
    _tail = 0;
    _new_data = false;
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
uint32_t &ekf_ring_buffer::time_ms(uint8_t idx)
{
    EKF_obs_element_t *el = (EKF_obs_element_t *)get_offset(idx);
    return el->time_ms;
}

/*
  Search through a ring buffer and return the newest data that is
  older than the time specified by sample_time_ms Zeros old data
  so it cannot not be used again Returns false if no data can be
  found that is less than 100msec old
*/
bool ekf_ring_buffer::recall(void *element,uint32_t sample_time)
{
    if (!_new_data) {
        return false;
    }
    bool success = false;
    uint8_t tail = _tail, bestIndex;

    if (_head == tail) {
        if (time_ms(tail) != 0 && time_ms(tail) <= sample_time) {
            // if head is equal to tail just check if the data is unused and within time horizon window
            if (((sample_time - time_ms(tail)) < 100)) {
                bestIndex = tail;
                success = true;
                _new_data = false;
            }
        }
    } else {
        while(_head != tail) {
            // find a measurement older than the fusion time horizon that we haven't checked before
            if (time_ms(tail) != 0 && time_ms(tail) <= sample_time) {
                // Find the most recent non-stale measurement that meets the time horizon criteria
                if (((sample_time - time_ms(tail)) < 100)) {
                    bestIndex = tail;
                    success = true;
                }
            } else if (time_ms(tail) > sample_time){
                break;
            }
            tail = (tail+1) % _size;
        }
    }

    if (!success) {
        return false;
    }

    memcpy(element, get_offset(bestIndex), elsize);
    _tail = (bestIndex+1) % _size;
    // make time zero to stop using it again,
    // resolves corner case of reusing the element when head == tail
    time_ms(bestIndex) = 0;
    return true;
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
    _head = (_head+1) % _size;
    // New data is written at the head
    memcpy(get_offset(_head), element, elsize);
    _new_data = true;
}


// zeroes all data in the ring buffer
void ekf_ring_buffer::reset()
{
    _head = 0;
    _tail = 0;
    _new_data = false;
    memset((void *)buffer,0,_size*uint32_t(elsize));
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
    if (buffer != nullptr) {
        // allow for init twice
        free(buffer);
    }
    buffer = calloc(size, elsize);
    if (buffer == nullptr) {
        return false;
    }
    _size = size;
    _youngest = 0;
    _oldest = 0;
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
    _youngest = (_youngest+1) % _size;
    memcpy(get_offset(_youngest), element, elsize);
    // set oldest data index
    _oldest = (_youngest+1) % _size;
    if (_oldest == 0) {
        _filled = true;
    }
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

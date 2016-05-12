/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// EKF Buffer models

// this buffer model is to be used for observation buffers,
// the data is pushed into buffer like any standard ring buffer
// return is based on the sample time provided
template <typename element_type>
class obs_ring_buffer_t
{
public:
    struct element_t{
        element_type element;
    } *buffer;

    // initialise buffer, returns false when allocation has failed
    bool init(uint32_t size)
    {
        buffer = new element_t[size];
        if(buffer == NULL)
        {
            return false;
        }
        memset(buffer,0,size*sizeof(element_t));
        _size = size;
        _head = 0;
        _tail = 0;
        _new_data = false;
        return true;
    }

    /*
     * Searches through a ring buffer and return the newest data that is older than the
     * time specified by sample_time_ms
     * Zeros old data so it cannot not be used again
     * Returns false if no data can be found that is less than 100msec old
    */

    bool recall(element_type &element,uint32_t sample_time)
    {
        if(!_new_data) {
            return false;
        }
        bool success = false;
        uint8_t tail = _tail, bestIndex;

        if(_head == tail) {
            if (buffer[tail].element.time_ms != 0 && buffer[tail].element.time_ms <= sample_time) {
                // if head is equal to tail just check if the data is unused and within time horizon window
                if (((sample_time - buffer[tail].element.time_ms) < 100)) {
                    bestIndex = tail;
                    success = true;
                    _new_data = false;
                }
            }
        } else {
            while(_head != tail) {
                // find a measurement older than the fusion time horizon that we haven't checked before
                if (buffer[tail].element.time_ms != 0 && buffer[tail].element.time_ms <= sample_time) {
                    // Find the most recent non-stale measurement that meets the time horizon criteria
                    if (((sample_time - buffer[tail].element.time_ms) < 100)) {
                        bestIndex = tail;
                        success = true;
                    }
                } else if(buffer[tail].element.time_ms > sample_time){
                    break;
                }
                tail = (tail+1)%_size;
            }
        }

        if (success) {
            element = buffer[bestIndex].element;
            _tail = (bestIndex+1)%_size;
            //make time zero to stop using it again, 
            //resolves corner case of reusing the element when head == tail
            buffer[bestIndex].element.time_ms = 0;
            return true;
        } else {
            return false;
        }
    }

    /*
     * Writes data and timestamp to a Ring buffer and advances indices that
     * define the location of the newest and oldest data
    */
    inline void push(element_type element)
    {
        // Advance head to next available index
        _head = (_head+1)%_size;
        // New data is written at the head
        buffer[_head].element = element;
        _new_data = true;
    }
    // writes the same data to all elements in the ring buffer
    inline void reset_history(element_type element, uint32_t sample_time) {
        for (uint8_t index=0; index<_size; index++) {
            buffer[index].element = element;
        }
    }

    // zeroes all data in the ring buffer
    inline void reset() {
        _head = 0;
        _tail = 0;
        _new_data = false;
        memset(buffer,0,_size*sizeof(element_t));
    }

private:
    uint8_t _size,_head,_tail,_new_data;
};


// Following buffer model is for IMU data,
// it achieves a distance of sample size
// between youngest and oldest
template <typename element_type>
class imu_ring_buffer_t
{
public:
    struct element_t{
        element_type element;
    } *buffer;

    // initialise buffer, returns false when allocation has failed
    bool init(uint32_t size)
    {
        buffer = new element_t[size];
        if(buffer == NULL)
        {
            return false;
        }
        memset(buffer,0,size*sizeof(element_t));
        _size = size;
        _youngest = 0;
        _oldest = 0;
        return true;
    }
    /*
     * Writes data to a Ring buffer and advances indices that
     * define the location of the newest and oldest data
    */
    inline void push_youngest_element(element_type element)
    {
        // push youngest to the buffer
        _youngest = (_youngest+1)%_size;
        buffer[_youngest].element = element;
        // set oldest data index
        _oldest = (_youngest+1)%_size;
    }

    // retrieve the oldest data from the ring buffer tail
    inline element_type pop_oldest_element() {
        element_type ret = buffer[_oldest].element;
        return ret;
    }

    // writes the same data to all elements in the ring buffer
    inline void reset_history(element_type element) {
        for (uint8_t index=0; index<_size; index++) {
            buffer[index].element = element;
        }
    }

    // zeroes all data in the ring buffer
    inline void reset() {
        _youngest = 0;
        _oldest = 0;
        memset(buffer,0,_size*sizeof(element_t));
    }

    // retrieves data from the ring buffer at a specified index
    inline element_type& operator[](uint32_t index) {
        return buffer[index].element;
    }

    // returns the index for the ring buffer oldest data
    inline uint8_t get_oldest_index(){
        return _oldest;
    }

    // returns the index for the ring buffer youngest data
    inline uint8_t get_youngest_index(){
        return _youngest;
    }
private:
    uint8_t _size,_oldest,_youngest;
};
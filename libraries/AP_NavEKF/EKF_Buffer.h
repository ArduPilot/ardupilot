/*
  common EKF buffer classes. These handles the storage buffers for EKF
  data to bring it onto the fusion time horizon
*/

#include <stdint.h>
#include <type_traits>

typedef struct {
    // measurement timestamp (msec)
    uint32_t    time_ms;
} EKF_obs_element_t;

// this class is to be used for observation buffers, the data is
// pushed into buffer like any standard ring buffer return is based on
// the sample time provided
class ekf_ring_buffer
{
public:
    ekf_ring_buffer(uint8_t elsize);

    // initialise buffer, returns false when allocation has failed
    bool init(uint8_t size);

    /*
     * Searches through a ring buffer and return the newest data that is older than the
     * time specified by sample_time_ms
     * Zeros old data so it cannot not be used again
     * Returns false if no data can be found that is less than 100msec old
    */
    bool recall(void *element, const uint32_t sample_time_ms);

    /*
     * Writes data and timestamp to a Ring buffer and advances indices that
     * define the location of the newest and oldest data
    */
    void push(const void *element);

    // zeroes all data in the ring buffer
    void reset();

private:
    const uint8_t elsize;
    void *buffer;

    // size of allocated buffer in elsize units
    uint8_t size;

    // index of the oldest element in the buffer
    uint8_t oldest;

    // total number of elements in the buffer
    uint8_t count;

    uint32_t time_ms(uint8_t idx) const;
    void *get_offset(uint8_t idx) const;
};

/*
  template class for more convenient type handling
 */
template <typename element_type>
class EKF_obs_buffer_t : ekf_ring_buffer
{
    static_assert(
        std::is_base_of<EKF_obs_element_t, element_type>::value,
        "must be a descendant of EKF_obs_element_t"
    );
public:
    EKF_obs_buffer_t() :
        ekf_ring_buffer(sizeof(element_type))
        {}

    bool init(uint8_t _size) {
        return ekf_ring_buffer::init(_size);
    }

    bool recall(element_type &element,uint32_t sample_time) {
        return ekf_ring_buffer::recall(&element, sample_time);
    }

    void push(const element_type &element) {
        return ekf_ring_buffer::push(&element);
    }

    void reset() {
        return ekf_ring_buffer::reset();
    }
};


/*
  ring buffer for IMU data,
*/
class ekf_imu_buffer
{
public:
    ekf_imu_buffer(uint8_t elsize);
    
    // initialise buffer, returns false when allocation has failed
    bool init(uint32_t size);

    /*
      Writes data to a Ring buffer and advances indices that
      define the location of the newest and oldest data
    */
    void push_youngest_element(const void *element);

    // return true if the buffer has been filled at least once
    bool is_filled(void) const {
        return _filled;
    }
    
    // retrieve the oldest data from the ring buffer tail
    void get_oldest_element(void *element);

    // writes the same data to all elements in the ring buffer
    void reset_history(const void *element);

    // zeroes all data in the ring buffer
    void reset();

    // retrieves data from the ring buffer at a specified index
    void *get(uint8_t index) const;

    // returns the index for the ring buffer oldest data
    uint8_t get_oldest_index() const {
        return _oldest;
    }

    // returns the index for the ring buffer youngest data
    uint8_t get_youngest_index() const {
        return _youngest;
    }

protected:
    const uint8_t elsize;
    void *buffer;
    uint8_t _size,_oldest,_youngest;
    bool _filled;

    void *get_offset(uint8_t idx) const;
};

/*
  template class for more convenient type handling
 */
template <typename element_type>
class EKF_IMU_buffer_t : ekf_imu_buffer
{
public:
    EKF_IMU_buffer_t() :
        ekf_imu_buffer(sizeof(element_type))
        {}

    bool init(uint8_t size) {
        return ekf_imu_buffer::init(size);
    }

    /*
      Writes data to a Ring buffer and advances indices that
     define the location of the newest and oldest data
    */
    void push_youngest_element(element_type element) {
        return ekf_imu_buffer::push_youngest_element(&element);
    }

    // return true if the buffer has been filled at least once
    bool is_filled(void) const {
        return ekf_imu_buffer::is_filled();
    }
    
    // retrieve the oldest data from the ring buffer tail
    element_type get_oldest_element() {
        element_type ret;
        ekf_imu_buffer::get_oldest_element(&ret);
        return ret;
    }

    // writes the same data to all elements in the ring buffer
    void reset_history(element_type element) {
        ekf_imu_buffer::reset_history(&element);
    }

    // zeroes all data in the ring buffer
    void reset() {
        ekf_imu_buffer::reset();
    }

    // retrieves data from the ring buffer at a specified index
    element_type& operator[](uint32_t index) {
        element_type *ret = (element_type *)ekf_imu_buffer::get(index);
        return *ret;
    }

    // returns the index for the ring buffer oldest data
    uint8_t get_oldest_index() {
        return ekf_imu_buffer::get_oldest_index();
    }

    // returns the index for the ring buffer youngest data
    inline uint8_t get_youngest_index() {
        return ekf_imu_buffer::get_youngest_index();
    }
};

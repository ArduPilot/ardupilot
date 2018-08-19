/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
    copied from AP_InertialSensor_Invensense, removed aux bus and FIFO usage
    this driver can be common Invensense driver for boards with connected DataReady pin if HAL API will be extended 
    to support IO_Complete callbacks

  driver for all supported Invensense IMUs, including MPU6000, MPU9250
  ICM-20608 and ICM-20602
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT && defined(INVENSENSE_DRDY_PIN)

#include <assert.h>
#include <utility>
#include <stdio.h>
#include <AP_HAL/Util.h>

#include <AP_HAL_F4Light/GPIO.h>
#include <AP_HAL_F4Light/Scheduler.h>
#include <AP_HAL_F4Light/SPIDevice.h>
#include <AP_Param_Helper/AP_Param_Helper.h>

#include "AP_InertialSensor_Revo.h"
#include "AP_InertialSensor_Invensense_registers.h"

extern const AP_HAL::HAL& hal;

#define debug(fmt, args ...)  do {printf("MPU: " fmt "\n", ## args); } while(0)

/*
  EXT_SYNC allows for frame synchronisation with an external device
  such as a camera. When enabled the LSB of AccelZ holds the FSYNC bit
 */
#ifndef INVENSENSE_EXT_SYNC_ENABLE
#define INVENSENSE_EXT_SYNC_ENABLE 0
#endif


#define MPU_SAMPLE_SIZE 14
#define MPU_FIFO_DOWNSAMPLE_COUNT 8
#define MPU_FIFO_BUFFER_LEN 64// ms of samples

#define int16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

/*
 *  RM-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

#ifdef MPU_DEBUG_LOG
    mpu_log_item AP_InertialSensor_Revo::mpu_log[MPU_LOG_SIZE] IN_CCM;
    uint16_t AP_InertialSensor_Revo::mpu_log_ptr=0;
#endif

/*
 *  RM-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
 *  accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)
 *
 *  See note below about accel scaling of engineering sample MPU6k
 *  variants however
 */

AP_InertialSensor_Revo::AP_InertialSensor_Revo(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                           enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _temp_filter(1000, 1)
    , _rotation(rotation)
    , _dev(std::move(dev))
    , nodata_count(0)
    , accel_len(0)
{
}

AP_InertialSensor_Revo::~AP_InertialSensor_Revo()
{
    if (_fifo_buffer != nullptr) {
        hal.util->free_type(_fifo_buffer, MPU_FIFO_BUFFER_LEN * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_Revo::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                               enum Rotation rotation)
{
        return nullptr;
}


AP_InertialSensor_Backend *AP_InertialSensor_Revo::probe(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                               enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_InertialSensor_Revo *sensor;

    dev->set_read_flag(0x80);

    sensor = new AP_InertialSensor_Revo(imu, std::move(dev), rotation);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    if (sensor->_mpu_type == Invensense_MPU9250) {
        sensor->_id = HAL_INS_MPU9250_SPI;
    } else if (sensor->_mpu_type == Invensense_MPU6500) {
        sensor->_id = HAL_INS_MPU6500;
    } else {
        sensor->_id = HAL_INS_MPU60XX_SPI;
    }

    return sensor;
}

bool AP_InertialSensor_Revo::_init()
{
    _drdy_pin = hal.gpio->channel(INVENSENSE_DRDY_PIN);
    _drdy_pin->mode(INPUT_PULLDOWN);

    bool success = _hardware_init();

    return success;
}


void AP_InertialSensor_Revo::_start(){
    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // setup ODR and on-sensor filtering
    _set_filter_register();

    // set sample rate to 1000Hz and apply a software filter
    // In this configuration, the gyro sample rate is 8kHz
    _register_write(MPUREG_SMPLRT_DIV, 0, true);
    hal.scheduler->delay_microseconds(10);

    // Gyro scale 2000º/s
    _register_write(MPUREG_GYRO_CONFIG, BITS_GYRO_FS_2000DPS, true);
    hal.scheduler->delay_microseconds(10);


    if (_mpu_type == Invensense_MPU6000 &&
        ((product_id == MPU6000ES_REV_C4) ||
         (product_id == MPU6000ES_REV_C5) ||
         (product_id == MPU6000_REV_C4)   ||
         (product_id == MPU6000_REV_C5))) {
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        _register_write(MPUREG_ACCEL_CONFIG,1<<3, true);
        _accel_scale = GRAVITY_MSS / 4096.f;
    } else {
        // Accel scale 16g (2048 LSB/g)
        _register_write(MPUREG_ACCEL_CONFIG,3<<3, true);
        _accel_scale = GRAVITY_MSS / 2048.f;
    }
    hal.scheduler->delay_microseconds(10);

    if (_mpu_type == Invensense_ICM20608 ||
        _mpu_type == Invensense_ICM20602) {
        // this avoids a sensor bug, see description above
		_register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
	}
    
    // configure interrupt to fire when new data arrives
    _register_write(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    hal.scheduler->delay_microseconds(10);

    // clear interrupt on any read, and hold the data ready pin high
    // until we clear the interrupt
    _register_write(MPUREG_INT_PIN_CFG, _register_read(MPUREG_INT_PIN_CFG) | BIT_INT_RD_CLEAR | BIT_LATCH_INT_EN);

    // now that we have initialised, we set the bus speed to high
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

void AP_InertialSensor_Revo::start()
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return;
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // only used for wake-up in accelerometer only low power mode
    _register_write(MPUREG_PWR_MGMT_2, 0x00);
    hal.scheduler->delay(1);

    // never use buggy FIFO
//    _fifo_reset();

    // grab the used instances
    enum DevTypes gdev, adev;
    switch (_mpu_type) {
    case Invensense_MPU9250:
        gdev = DEVTYPE_GYR_MPU9250;
        adev = DEVTYPE_ACC_MPU9250;
        break;
    case Invensense_MPU6000:
    case Invensense_MPU6500:
    case Invensense_ICM20608:
    case Invensense_ICM20602:
    default:
        gdev = DEVTYPE_GYR_MPU6000;
        adev = DEVTYPE_ACC_MPU6000;
        break;
    }

    /*
      setup temperature sensitivity and offset. This varies
      considerably between parts
     */
    switch (_mpu_type) {
    case Invensense_MPU9250:
        temp_zero = 21;
        temp_sensitivity = 1.0/340;
        break;

    case Invensense_MPU6000:
    case Invensense_MPU6500:
        temp_zero = 36.53;
        temp_sensitivity = 1.0/340;
        break;

    case Invensense_ICM20608:
    case Invensense_ICM20602:
        temp_zero = 25;
        temp_sensitivity = 1.0/326.8; 
        break;
    }

    _gyro_instance = _imu.register_gyro(1000, _dev->get_bus_id_devtype(gdev));
    _accel_instance = _imu.register_accel(1000, _dev->get_bus_id_devtype(adev));

    // read and remember the product ID rev c has 1/2 the sensitivity of rev d
    product_id = _register_read(MPUREG_PRODUCT_ID);

    _start(); // start MPU 

    _dev->get_semaphore()->give();

    // setup sensor rotations from probe()
    set_gyro_orientation(_gyro_instance, _rotation);
    set_accel_orientation(_accel_instance, _rotation);

    // allocate fifo buffer
    _fifo_buffer = (uint8_t *)(hal.util->malloc_type((MPU_FIFO_BUFFER_LEN+1) * MPU_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE));
    if (_fifo_buffer == nullptr) {
        AP_HAL::panic("Invensense: Unable to allocate FIFO buffer");
    }

    GPIO::_attach_interrupt(INVENSENSE_DRDY_PIN, Scheduler::get_handler(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Revo::_isr, void)), RISING, MPU_INT_PRIORITY);

    _register_read(MPUREG_INT_STATUS); // reset interrupt request

                                                        // some longer than MPU period
    task_handle = Scheduler::register_timer_task(1010, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Revo::_poll_data, void), NULL); // period just for case, task will be activated by request
//    REVOMINIScheduler::set_task_priority(task_handle, DRIVER_PRIORITY); // like other drivers
}


/*
  publish any pending data
 */
bool AP_InertialSensor_Revo::update()
{
    update_accel(_accel_instance);
    update_gyro(_gyro_instance);

    _publish_temperature(_accel_instance, _temp_filtered);

    return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_Revo::accumulate()
{
    // nothing to do
}


/*
 * Return true if the Invensense has new data available for reading.
 *
 * We use the data ready pin if it is available.  Otherwise, read the
 * status register.
 */
bool AP_InertialSensor_Revo::_data_ready()
{
    return _drdy_pin->read() != 0;
}

/*
    ISR procedure for data read. Ring buffer don't needs to use semaphores for data access
    
    also we don't own a bus semaphore and can't guarantee that bus is free. But in Revo MPU uses personal SPI bus so it is ABSOLUTELY free :)
*/
void AP_InertialSensor_Revo::_isr(){
     uint8_t *data = _fifo_buffer + MPU_SAMPLE_SIZE * write_ptr;
//    _fifo_buffer[write_ptr].time = REVOMINIScheduler::_micros64();

    _dev->register_completion_callback(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Revo::_ioc, void)); // IO completion interrupt
    
    _block_read(MPUREG_ACCEL_XOUT_H, data, MPU_SAMPLE_SIZE); // start SPI transfer 
}

void AP_InertialSensor_Revo::_ioc(){ // io completion ISR, data already in its place
    uint16_t new_wp = write_ptr+1;
    if(new_wp >= MPU_FIFO_BUFFER_LEN) { // move write pointer
        new_wp=0;                         // ring
    }
    if(new_wp == read_ptr) { // buffer overflow
#ifdef MPU_DEBUG
        REVOMINIScheduler::MPU_buffer_overflow(); // count them
        // not overwrite, just skip last data
#endif
    } else {
        write_ptr=new_wp; // move forward
    }

    //_dev->register_completion_callback(NULL); 
// we should release the bus semaphore if we use them 
//    _dev->get_semaphore()->give();            // release


    if(Scheduler::get_current_task() != (void *)task_handle) {
/*
        REVOMINIScheduler::set_task_active(task_handle); // resume task instead of using period. 
        REVOMINIScheduler::context_switch_isr(); // and reschedule tasks after interrupt
*/ 
        Scheduler::task_resume(task_handle); // resume task instead of using period. 
    }
}

/*
 * Timer process to poll for new data from the Invensense. Called from timer's interrupt or from personal thread
 */
void AP_InertialSensor_Revo::_poll_data()
{
    _read_fifo();
}

bool AP_InertialSensor_Revo::_accumulate(uint8_t *samples, uint8_t n_samples)
{
    bool ret=true;
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;
        Vector3f accel, gyro;
        bool fsync_set = false;


        accel = Vector3f(int16_val(data, 1),
                         int16_val(data, 0),
                         -int16_val(data, 2)) * _accel_scale;

        int16_t t2 = int16_val(data, 3);
/*
        if (!_check_raw_temp(t2)) {
            debug("temp reset %d %d i=%d", _raw_temp, t2, i);
            return false; // just skip this sample
        }
*/        
        float temp = t2 * temp_sensitivity + temp_zero;
        
        gyro = Vector3f(int16_val(data, 5),
                        int16_val(data, 4),
                        -int16_val(data, 6)) * GYRO_SCALE;

        _rotate_and_correct_accel(_accel_instance, accel);
        _rotate_and_correct_gyro(_gyro_instance, gyro);

#if 0 // filter out samples if vector length changed by 100% This is cool for debug but drops samples in the case of even weak blows

#define FILTER_KOEF 0.1
        float len = accel.length();
        if(is_zero(accel_len)) {
            accel_len=len;
        } else {
            float d = abs(accel_len-len)/(accel_len+len);
            if(d*100 > 50) { // difference more than 100% from mean value
                debug("accel len error: mean %f got %f", accel_len, len );
                ret= false; //just report
                float k = FILTER_KOEF / (d*10); // 5 and more, so one bad sample never change mean more than 4%
                accel_len = accel_len * (1-k) + len*k; // complimentary filter 1/k on bad samples
            } else {
                accel_len = accel_len * (1-FILTER_KOEF) + len*FILTER_KOEF; // complimentary filter 1/10 on good samples
            }
        }
#endif

        if(ret) {
            uint8_t kG = hal_param_helper->_correct_gyro;
            if(kG){                                  // compensate gyro drift by long-time mean
                float gyro_koef = 1.0 / (kG * 1000); // integrator time constant in seconds
                gyro_mean = gyro_mean * (1-gyro_koef) + gyro*gyro_koef;
            
                gyro -= gyro_mean;
            }
        
            _notify_new_accel_raw_sample(_accel_instance, accel, 0, fsync_set);
            _notify_new_gyro_raw_sample(_gyro_instance, gyro);

            _temp_filtered = _temp_filter.apply(temp);
        }
    }
    return ret;
}

/*
  when doing sensor-rate sampling the sensor gives us 8k samples/second. Every 2nd accel sample is a duplicate.

  To filter this we first apply a 1p low pass filter at 188Hz, then we
  average over 8 samples to bring the data rate down to 1kHz. This
  gives very good aliasing rejection at frequencies well above what
  can be handled with 1kHz sample rates.
 */
bool AP_InertialSensor_Revo::_accumulate_sensor_rate_sampling(uint8_t *samples, uint8_t n_samples)
{
    int32_t tsum = 0;
    const int32_t clip_limit = AP_INERTIAL_SENSOR_ACCEL_CLIP_THRESH_MSS / _accel_scale;
    bool clipped = false;
    bool ret = true;
    
    for (uint8_t i = 0; i < n_samples; i++) {
        const uint8_t *data = samples + MPU_SAMPLE_SIZE * i;

        // use temperatue to detect FIFO corruption
        int16_t t2 = int16_val(data, 3);
/* MPU don't likes such reads
        if (!_check_raw_temp(t2)) {
            debug("temp reset %d %d", _raw_temp, t2);
//            _fifo_reset();
            ret = false;
            break;
        }
*/
        tsum += t2;

        if ((_accum.count & 1) == 0) {
            // accel data is at 4kHz
            Vector3f a(int16_val(data, 1),
                       int16_val(data, 0),
                       -int16_val(data, 2));
            if (fabsf(a.x) > clip_limit ||
                fabsf(a.y) > clip_limit ||
                fabsf(a.z) > clip_limit) {
                clipped = true;
            }
            _accum.accel += _accum.accel_filter.apply(a);
        }

        Vector3f g(int16_val(data, 5),
                   int16_val(data, 4),
                   -int16_val(data, 6));

        _accum.gyro += _accum.gyro_filter.apply(g);
        _accum.count++;

        if (_accum.count == MPU_FIFO_DOWNSAMPLE_COUNT) {
            float ascale = _accel_scale / (MPU_FIFO_DOWNSAMPLE_COUNT/2);
            _accum.accel *= ascale;

            float gscale = GYRO_SCALE / MPU_FIFO_DOWNSAMPLE_COUNT;
            _accum.gyro *= gscale;
            
            _rotate_and_correct_accel(_accel_instance, _accum.accel);
            _rotate_and_correct_gyro(_gyro_instance, _accum.gyro);
            
            _notify_new_accel_raw_sample(_accel_instance, _accum.accel, 0, false);
            _notify_new_gyro_raw_sample(_gyro_instance, _accum.gyro);
            
            _accum.accel.zero();
            _accum.gyro.zero();
            _accum.count = 0;
        }
    }

    if (clipped) {
        increment_clip_count(_accel_instance);
    }

    if (ret) {
        float temp = (static_cast<float>(tsum)/n_samples)*temp_sensitivity + temp_zero;
        _temp_filtered = _temp_filter.apply(temp);
    }
    
    return ret;
}


#define MAX_NODATA_TIME 5000 // 5ms

void AP_InertialSensor_Revo::_read_fifo()
{
    uint32_t now=Scheduler::_micros();

#ifdef MPU_DEBUG_LOG
    uint16_t old_log_ptr=mpu_log_ptr;
    mpu_log_item & p = mpu_log[mpu_log_ptr++];
    if(mpu_log_ptr>=MPU_LOG_SIZE) mpu_log_ptr=0;
    p.t=now;
    p.read_ptr=read_ptr;
    p.write_ptr=write_ptr;
#endif
    
    if(read_ptr == write_ptr) {
        if(_data_ready()){ // no interrupt for some reason?
            _isr();
        }
        if(now - last_sample > MAX_NODATA_TIME) { // something went wrong - data stream stopped
            _start(); // try to restart MPU        
            last_sample=now;
#ifdef MPU_DEBUG
            REVOMINIScheduler::MPU_restarted(); // count them
#endif
        }
        return;
    }

    last_sample=now;

    uint16_t count = 0;
#ifdef MPU_DEBUG
    uint32_t dt    = 0;
    uint32_t t     = now;
#endif
    
    while(read_ptr != write_ptr) { // there are samples
//        uint64_t time = _fifo_buffer[read_ptr++].time; // we can get exact time
        uint8_t *rx = _fifo_buffer + MPU_SAMPLE_SIZE * read_ptr++;  // calculate address and move to next item
        if(read_ptr >= MPU_FIFO_BUFFER_LEN) { // move write pointer
            read_ptr=0;                       // ring
        }


        if (_fast_sampling) {
            if (!_accumulate_sensor_rate_sampling(rx, 1)) {
//                debug("stop at %u of %u", n_samples, bytes_read/MPU_SAMPLE_SIZE);
                // break;  don't break before all items in queue will be readed
                continue;
            }
        } else {
            if (!_accumulate(rx, 1)) {
                // break; don't break before all items in queue will be readed
                continue;
            }
        }
        count++;
    }
    now = Scheduler::_micros();
    last_sample=now;

#ifdef MPU_DEBUG_LOG
    if(count==1) {
        mpu_log_ptr = old_log_ptr;
    }
#endif
#ifdef MPU_DEBUG
    dt= now - t;// time from entry
    REVOMINIScheduler::MPU_stats(count,dt);
#endif

// only wait_for_sample() uses delay_microseconds_boost() so 
// resume main thread then it waits for this sample - sample already got
    Scheduler::resume_boost(); 
}

/*
  fetch temperature in order to detect FIFO sync errors
*/
bool AP_InertialSensor_Revo::_check_raw_temp(int16_t t2)
{
    if (abs(t2 - _raw_temp) < 400) {
        // cached copy OK
        return true;
    }
    uint8_t trx[2];
    if (_block_read(MPUREG_TEMP_OUT_H, trx, 2)) {
        _raw_temp = int16_val(trx, 0);
    }
    return (abs(t2 - _raw_temp) < 400);
}

bool AP_InertialSensor_Revo::_block_read(uint8_t reg, uint8_t *buf,
                                            uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_Revo::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_Revo::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

/*
  set the DLPF filter frequency. Assumes caller has taken semaphore
 */
void AP_InertialSensor_Revo::_set_filter_register(void)
{
    uint8_t config;

#if INVENSENSE_EXT_SYNC_ENABLE
    // add in EXT_SYNC bit if enabled
    config = (MPUREG_CONFIG_EXT_SYNC_AZ << MPUREG_CONFIG_EXT_SYNC_SHIFT);
#else
    config = 0;
#endif

    if (enable_fast_sampling(_accel_instance)) {
        _fast_sampling = (_mpu_type != Invensense_MPU6000 && _dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI);
        if (_fast_sampling) {
#ifdef DEBUG_BUILD
            printf("MPU[%u]: enabled fast sampling\n", _accel_instance);
#endif
            // for logging purposes set the oversamping rate
            _set_accel_oversampling(_accel_instance, MPU_FIFO_DOWNSAMPLE_COUNT/2);
            _set_gyro_oversampling(_gyro_instance, MPU_FIFO_DOWNSAMPLE_COUNT);

            /* set divider for internal sample rate to 0x1F when fast
             sampling enabled. This reduces the impact of the slave
             sensor on the sample rate. It ends up with around 75Hz
             slave rate, and reduces the impact on the gyro and accel
             sample rate, ending up with around 7760Hz gyro rate and
             3880Hz accel rate
             */
            _register_write(MPUREG_I2C_SLV4_CTRL, 0x1F);
        }
    }
    
    if (_fast_sampling) {
        // this gives us 8kHz sampling on gyros and 4kHz on accels
        config |= BITS_DLPF_CFG_256HZ_NOLPF2;
    } else {
        // limit to 1kHz if not on SPI
        config |= BITS_DLPF_CFG_188HZ;
    }

    config |= MPUREG_CONFIG_FIFO_MODE_STOP;
    _register_write(MPUREG_CONFIG, config, true);

    if (_mpu_type != Invensense_MPU6000) {
        if (_fast_sampling) {
            // setup for 4kHz accels
            _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_FCHOICE_B, true);
        } else {
            _register_write(ICMREG_ACCEL_CONFIG2, ICM_ACC_DLPF_CFG_218HZ, true);
        }
    }
}

/*
  check whoami for sensor type
 */
bool AP_InertialSensor_Revo::_check_whoami(void)
{
    uint8_t whoami = _register_read(MPUREG_WHOAMI);
    switch (whoami) {
    case MPU_WHOAMI_6000:
        _mpu_type = Invensense_MPU6000;
        return true;
    case MPU_WHOAMI_6500:
        _mpu_type = Invensense_MPU6500;
        return true;
    case MPU_WHOAMI_MPU9250:
    case MPU_WHOAMI_MPU9255:
        _mpu_type = Invensense_MPU9250;
        return true;
    case MPU_WHOAMI_20608:
        _mpu_type = Invensense_ICM20608;
        return true;
    case MPU_WHOAMI_20602:
        _mpu_type = Invensense_ICM20602;
        return true;
    }
    // not a value WHOAMI result
    return false;
}


bool AP_InertialSensor_Revo::_hardware_init(void)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }

    // setup for register checking
    _dev->setup_checked_registers(7, 20);
    
    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!_check_whoami()) {
        _dev->get_semaphore()->give();
        return false;
    }

    // Chip reset
    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        _last_stat_user_ctrl = _register_read(MPUREG_USER_CTRL);

        /* First disable the master I2C to avoid hanging the slaves on the
         * aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
         * is used */
        if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
            _last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
            _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl);
            hal.scheduler->delay(10);
        }

        /* reset device */
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
        hal.scheduler->delay(100);

        /* bus-dependent initialization */
        if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
            /* Disable I2C bus if SPI selected (Recommended in Datasheet to be
             * done just after the device is reset) */
            _last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
            _register_write(MPUREG_USER_CTRL, _last_stat_user_ctrl);
        }

        /* bus-dependent initialization */
        if ((_dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C) && (_mpu_type == Invensense_MPU9250)) {
            /* Enable I2C bypass to access internal AK8963 */
            _register_write(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN);
        }

        // Wake up device and select GyroZ clock. Note that the
        // Invensense starts up in sleep mode, and it can take some time
        // for it to come out of sleep
        _register_write(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_ZGYRO);
        hal.scheduler->delay(5);

        // check it has woken up
        if (_register_read(MPUREG_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_ZGYRO) {
            break;
        }

        hal.scheduler->delay(10);
        if (_data_ready()) {
            break;
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    if (tries == 5) {
#ifdef DEBUG_BUILD
        printf("Failed to boot Invensense 5 times\n");
#endif
        return false;
    }

	if (_mpu_type == Invensense_ICM20608 ||
        _mpu_type == Invensense_ICM20602) {
        // this avoids a sensor bug, see description above
		_register_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE, true);
	}
    
    return true;
}


#endif // BOARD_REVO

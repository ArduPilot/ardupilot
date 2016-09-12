#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
#include "RCOutput_Bebop.h"

#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"

/* BEBOP BLDC registers description */
#define BEBOP_BLDC_I2C_ADDR 0x08
#define BEBOP_BLDC_STARTPROP 0x40
#define BEBOP_BLDC_SETREFSPEED 0x02

#define BEBOP_BLDC_GETOBSDATA 0x20

struct PACKED bldc_info {
    uint8_t version_maj;
    uint8_t version_min;
    uint8_t type;
    uint8_t n_motors;
    uint16_t n_flights;
    uint16_t last_flight_time;
    uint32_t total_flight_time;
    uint8_t last_error;
};

#define BEBOP_BLDC_TOGGLE_GPIO 0x4d
#define BEBOP_BLDC_GPIO_0       (1 << 0)
#define BEBOP_BLDC_GPIO_1       (1 << 1)
#define BEBOP_BLDC_GPIO_2       (1 << 2)
#define BEBOP_BLDC_GPIO_3       (1 << 3)
#define BEBOP_BLDC_GPIO_POWER   (1 << 4)

#define BEBOP_BLDC_STOP_PROP 0x60

#define BEBOP_BLDC_CLEAR_ERROR 0x80

#define BEBOP_BLDC_PLAY_SOUND 0x82

#define BEBOP_BLDC_GET_INFO 0xA0

#define BEBOP_BLDC_MIN_PERIOD_US 1100
#define BEBOP_BLDC_MAX_PERIOD_US 1900
#define BEBOP_BLDC_MIN_RPM 1000
/* the max rpm speed is different on Bebop 2 */
#define BEBOP_BLDC_MAX_RPM_1 11000
#define BEBOP_BLDC_MAX_RPM_2 12200
#define BEBOP_BLDC_MAX_RPM_DISCO 12500

/* Priority of the thread controlling the BLDC via i2c
 * set to 14, which is the same as the UART
 */
#define RCOUT_BEBOP_RTPRIO 14
/* Set timeout to 500ms */
#define BEBOP_BLDC_TIMEOUT_NS 500000000

enum {
    BEBOP_BLDC_STARTED,
    BEBOP_BLDC_STOPPED,
};

/* values of bottom nibble of the obs data status byte */
enum BLDC_STATUS {
    BEBOP_BLDC_STATUS_STOPPED=1,
    BEBOP_BLDC_STATUS_RAMPUP=2,
    BEBOP_BLDC_STATUS_RUNNING=4,
    BEBOP_BLDC_STATUS_RAMPDOWN=5
};

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

RCOutput_Bebop::RCOutput_Bebop(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
    , _min_pwm(BEBOP_BLDC_MIN_PERIOD_US)
    , _max_pwm(BEBOP_BLDC_MAX_PERIOD_US)
    , _state(BEBOP_BLDC_STOPPED)
{
    memset(_period_us, 0, sizeof(_period_us));
    memset(_request_period_us, 0, sizeof(_request_period_us));
    memset(_rpm, 0, sizeof(_rpm));
}

uint8_t RCOutput_Bebop::_checksum(uint8_t *data, unsigned int len)
{
    uint8_t checksum = data[0];
    unsigned int i;

    for (i = 1; i < len; i++)
        checksum = checksum ^ data[i];

    return checksum;
}

void RCOutput_Bebop::_start_prop()
{
    uint8_t data = BEBOP_BLDC_STARTPROP;

    if (!_dev->get_semaphore()->take(0)) {
        return;
    }

    if (_dev->transfer(&data, sizeof(data), nullptr, 0)) {
        _state = BEBOP_BLDC_STARTED;
    }
    _dev->get_semaphore()->give();
}

void RCOutput_Bebop::_set_ref_speed(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM])
{
    struct PACKED bldc_ref_speed_data {
        uint8_t     cmd;
        uint16_t    rpm[BEBOP_BLDC_MOTORS_NUM];
        uint8_t     enable_security;
        uint8_t     checksum;
    } data {};
    int i;

    data.cmd = BEBOP_BLDC_SETREFSPEED;

    for (i=0; i<BEBOP_BLDC_MOTORS_NUM; i++) {
        data.rpm[i] = htobe16(rpm[i]);
    }

    data.enable_security = 0;
    data.checksum = _checksum((uint8_t *) &data, sizeof(data) - 1);

    if (!_dev->get_semaphore()->take(0)) {
        return;
    }

    _dev->transfer((uint8_t *)&data, sizeof(data), nullptr, 0);

    _dev->get_semaphore()->give();
}

bool RCOutput_Bebop::_get_info(struct bldc_info *info)
{
    if (info == nullptr) {
        return false;
    }

    memset(info, 0, sizeof(struct bldc_info));

    if (!_dev->get_semaphore()->take(0)) {
        return false;
    }
    _dev->read_registers(BEBOP_BLDC_GET_INFO, (uint8_t*)info, sizeof(*info));
    _dev->get_semaphore()->give();
    return true;
}

int RCOutput_Bebop::read_obs_data(BebopBLDC_ObsData &obs)
{
    /*
      the structure returned is different on the Disco from the Bebop
     */
    struct PACKED bldc_obs_data {
        uint16_t    rpm[BEBOP_BLDC_MOTORS_NUM];
        uint16_t    batt_mv;
        uint8_t     status;
        uint8_t     error;
        uint8_t     motors_err;
        uint8_t     temp;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_DISCO
        /*
          bit 0 indicates an overcurrent on the RC receiver port when high
          bits #1-#6 indicate an overcurrent on the #1-#6 PPM servos
         */
        uint8_t     overcurrent;
#endif
        uint8_t     checksum;
    } data;

    memset(&data, 0, sizeof(data));
    if (!_dev->get_semaphore()->take(0)) {
        return -EBUSY;
    }

    _dev->read_registers(BEBOP_BLDC_GETOBSDATA, (uint8_t *)&data, sizeof(data));
    _dev->get_semaphore()->give();

    if (data.checksum != _checksum((uint8_t*)&data, sizeof(data)-1)) {
        return -EBUSY;
    }

    memset(&obs, 0, sizeof(obs));
    
    /* fill obs class */
    for (uint8_t i = 0; i < _n_motors; i++) {
        /* extract 'rpm saturation bit' */
        obs.rpm_saturated[i] = (data.rpm[i] & (1 << 7)) ? 1 : 0;
        /* clear 'rpm saturation bit' */
        data.rpm[i] &= (uint16_t)(~(1 << 7));
        obs.rpm[i] = be16toh(data.rpm[i]);
        if (obs.rpm[i] == 0) {
            obs.rpm_saturated[i] = 0;
        }
#if 0
        printf("rpm %u %u %u %u status 0x%02x temp %u\n",
               obs.rpm[i], _rpm[0], _period_us[0], _period_us_to_rpm(_period_us[0]),
               (unsigned)data.status,
               (unsigned)data.temp);
#endif
    }

    // sync our state from status. This makes us more robust to i2c errors
    enum BLDC_STATUS bldc_status = (enum BLDC_STATUS)(data.status & 0x0F);
    switch (bldc_status) {
    case BEBOP_BLDC_STATUS_STOPPED:
    case BEBOP_BLDC_STATUS_RAMPDOWN:
        _state = BEBOP_BLDC_STOPPED;
        break;
    case BEBOP_BLDC_STATUS_RAMPUP:
    case BEBOP_BLDC_STATUS_RUNNING:
        _state = BEBOP_BLDC_STARTED;
        break;
    }
    
    obs.batt_mv = be16toh(data.batt_mv);
    obs.status = data.status;
    obs.error = data.error;
    obs.motors_err = data.motors_err;
    obs.temperature = data.temp;

    return 0;
}

void RCOutput_Bebop::_toggle_gpio(uint8_t mask)
{
    if (!_dev->get_semaphore()->take(0)) {
        return;
    }

    _dev->write_register(BEBOP_BLDC_TOGGLE_GPIO, mask);
    _dev->get_semaphore()->give();
}

void RCOutput_Bebop::_stop_prop()
{
    uint8_t data = BEBOP_BLDC_STOP_PROP;

    if (!_dev->get_semaphore()->take(0)) {
        return;
    }

    _dev->transfer(&data, sizeof(data), nullptr, 0);
    _dev->get_semaphore()->give();
}

void RCOutput_Bebop::_clear_error()
{
    uint8_t data = BEBOP_BLDC_CLEAR_ERROR;

    if (!_dev->get_semaphore()->take(0)) {
        return;
    }

    _dev->transfer(&data, sizeof(data), nullptr, 0);
    _dev->get_semaphore()->give();
}

void RCOutput_Bebop::_play_sound(uint8_t sound)
{
    if (!_dev->get_semaphore()->take(0)) {
        return;
    }

    _dev->write_register(BEBOP_BLDC_PLAY_SOUND, sound);
    _dev->get_semaphore()->give();
}

uint16_t RCOutput_Bebop::_period_us_to_rpm(uint16_t period_us)
{
    period_us = constrain_int16(period_us, _min_pwm, _max_pwm);
    float period_us_fl = period_us;
    float rpm_fl = (period_us_fl - _min_pwm)/(_max_pwm - _min_pwm) *
                    (_max_rpm - BEBOP_BLDC_MIN_RPM) + BEBOP_BLDC_MIN_RPM;

    return (uint16_t)rpm_fl;
}

void RCOutput_Bebop::init()
{
    int ret=0;
    struct sched_param param = { .sched_priority = RCOUT_BEBOP_RTPRIO };
    pthread_attr_t attr;
    pthread_condattr_t cond_attr;

    /* Initialize thread, cond, and mutex */
    ret = pthread_mutex_init(&_mutex, NULL);
    if (ret != 0) {
        perror("RCout_Bebop: failed to init mutex\n");
        return;
    }

    pthread_mutex_lock(&_mutex);

    pthread_condattr_init(&cond_attr);
    pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
    ret = pthread_cond_init(&_cond, &cond_attr);
    pthread_condattr_destroy(&cond_attr);
    if (ret != 0) {
        perror("RCout_Bebop: failed to init cond\n");
        goto exit;
    }

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        perror("RCOut_Bebop: failed to init attr\n");
        goto exit;
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _control_thread, this);
    if (ret != 0) {
        perror("RCOut_Bebop: failed to create thread\n");
        goto exit;
    }

    _clear_error();

    /* Set an initial dummy frequency */
    _frequency = 50;

    // enable servo power (also receiver power)
    _toggle_gpio(BEBOP_BLDC_GPIO_2 | BEBOP_BLDC_GPIO_POWER);
    
exit:
    pthread_mutex_unlock(&_mutex);
    return;
}

void RCOutput_Bebop::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _frequency = freq_hz;
}

uint16_t RCOutput_Bebop::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_Bebop::enable_ch(uint8_t ch)
{
}

void RCOutput_Bebop::disable_ch(uint8_t ch)
{
    _stop_prop();
}

void RCOutput_Bebop::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= BEBOP_BLDC_MOTORS_NUM) {
        return;
    }

    _request_period_us[ch] = period_us;

    if (!_corking) {
        push();
    }
}

void RCOutput_Bebop::cork()
{
    _corking = true;
}

void RCOutput_Bebop::push()
{
    _corking = false;
    pthread_mutex_lock(&_mutex);
    memcpy(_period_us, _request_period_us, sizeof(_period_us));
    pthread_cond_signal(&_cond);
    pthread_mutex_unlock(&_mutex);
    memset(_request_period_us, 0, sizeof(_request_period_us));
}

uint16_t RCOutput_Bebop::read(uint8_t ch)
{
    if (ch < BEBOP_BLDC_MOTORS_NUM) {
        return _period_us[ch];
    } else {
        return 0;
    }
}

void RCOutput_Bebop::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}

void RCOutput_Bebop::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
    _min_pwm = min_pwm;
    _max_pwm = max_pwm;
}

/* Separate thread to handle the Bebop motors controller */
void* RCOutput_Bebop::_control_thread(void *arg) {
    RCOutput_Bebop* rcout = (RCOutput_Bebop *) arg;

    rcout->_run_rcout();
    return NULL;
}

void RCOutput_Bebop::_run_rcout()
{
    uint16_t current_period_us[BEBOP_BLDC_MOTORS_NUM];
    uint8_t i;
    int ret;
    struct timespec ts;
    struct bldc_info info;
    uint8_t bebop_bldc_channels[BEBOP_BLDC_MOTORS_NUM] {};
    int hw_version;

    memset(current_period_us, 0, sizeof(current_period_us));

    if (!_get_info(&info)) {
        AP_HAL::panic("failed to get BLDC info");
    }

    // remember _n_motors for read_obs_data()
    _n_motors = info.n_motors;

#if CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_DISCO
    uint8_t bebop_bldc_right_front, bebop_bldc_left_front,
            bebop_bldc_left_back, bebop_bldc_right_back;
    /* Set motor order depending on BLDC version.On bebop 1 with version 1
     * keep current order. The order changes from version 2 on bebop 1 and
     * remains the same as this for bebop 2
     */
    if (info.version_maj == 1 || info.version_maj == 5) {
        bebop_bldc_right_front = BEBOP_BLDC_MOTOR_1;
        bebop_bldc_left_front  = BEBOP_BLDC_MOTOR_2;
        bebop_bldc_left_back   = BEBOP_BLDC_MOTOR_3;
        bebop_bldc_right_back  = BEBOP_BLDC_MOTOR_4;
    } else {
        bebop_bldc_right_front = BEBOP_BLDC_MOTOR_2;
        bebop_bldc_left_front  = BEBOP_BLDC_MOTOR_1;
        bebop_bldc_left_back   = BEBOP_BLDC_MOTOR_4;
        bebop_bldc_right_back  = BEBOP_BLDC_MOTOR_3;
    }

    bebop_bldc_channels[0] = bebop_bldc_right_front;
    bebop_bldc_channels[1] = bebop_bldc_left_back;
    bebop_bldc_channels[2] = bebop_bldc_left_front;
    bebop_bldc_channels[3] = bebop_bldc_right_back;
#endif
    
    hw_version = Util::from(hal.util)->get_hw_arm32();
    if (hw_version == UTIL_HARDWARE_BEBOP) {
        _max_rpm = BEBOP_BLDC_MAX_RPM_1;
    } else if (hw_version == UTIL_HARDWARE_BEBOP2) {
        _max_rpm = BEBOP_BLDC_MAX_RPM_2;
    } else if (hw_version == UTIL_HARDWARE_DISCO) {
        _max_rpm = BEBOP_BLDC_MAX_RPM_DISCO;
    } else if (hw_version < 0) {
        AP_HAL::panic("failed to get hw version %s", strerror(hw_version));
    } else {
        AP_HAL::panic("unsupported hw version %d", hw_version);
    }
    printf("Bebop: vers %u/%u type %u nmotors %u n_flights %u last_flight_time %u total_flight_time %u maxrpm %u\n",
           (unsigned)info.version_maj, (unsigned)info.version_min, (unsigned)info.type,
           (unsigned)info.n_motors, (unsigned)be16toh(info.n_flights),
           (unsigned)be16toh(info.last_flight_time), (unsigned)be32toh(info.total_flight_time),
           (unsigned)_max_rpm);

    while (true) {
        pthread_mutex_lock(&_mutex);
        ret = clock_gettime(CLOCK_MONOTONIC, &ts);
        if (ret != 0) {
            pthread_mutex_unlock(&_mutex);
            continue;
        }

        if (ts.tv_nsec > (1000000000 - BEBOP_BLDC_TIMEOUT_NS)) {
            ts.tv_sec += 1;
            ts.tv_nsec = ts.tv_nsec + BEBOP_BLDC_TIMEOUT_NS - 1000000000;
        } else {
            ts.tv_nsec += BEBOP_BLDC_TIMEOUT_NS;
        }

        ret = 0;
        while ((memcmp(_period_us, current_period_us, sizeof(_period_us)) == 0) && (ret == 0)) {
            ret = pthread_cond_timedwait(&_cond, &_mutex, &ts);
        }

        memcpy(current_period_us, _period_us, sizeof(_period_us));
        pthread_mutex_unlock(&_mutex);

        /* start propellers if the speed of the 4 motors is >= min speed
         * min speed set to min_pwm + 50*/
        for (i = 0; i < _n_motors; i++) {
            if (current_period_us[i] <= _min_pwm + 50) {
                break;
            }
            _rpm[bebop_bldc_channels[i]] = _period_us_to_rpm(current_period_us[i]);
        }

        if (i < _n_motors) {
            /* one motor pwm value is at minimum (or under)
             * if the motors are started, stop them*/
            if (_state == BEBOP_BLDC_STARTED) {
                _stop_prop();
                _clear_error();
            }
        } else {
            /* all the motor pwm values are higher than minimum
             * if the bldc is stopped, start it*/
            if (_state == BEBOP_BLDC_STOPPED) {
                _start_prop();
            }
            _set_ref_speed(_rpm);
        }
    }
}

#endif

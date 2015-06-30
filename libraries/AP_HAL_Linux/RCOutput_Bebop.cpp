#include <AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
#include "RCOutput_Bebop.h"
#include <endian.h>
#include <stdio.h>

/* BEBOP BLDC motor controller address and registers description */
#define BEBOP_BLDC_I2C_ADDR 0x08

#define BEBOP_BLDC_STARTPROP 0x40
#define     BEBOP_BLDC_CLOCKWISE 1
#define     BEBOP_BLDC_COUNTERCLOCKWISE 0

static const uint8_t bebop_motors_bitmask = (BEBOP_BLDC_COUNTERCLOCKWISE  <<
                            (BEBOP_BLDC_MOTORS_NUM - 1 - BEBOP_BLDC_LEFT_BACK)) |
                            (BEBOP_BLDC_CLOCKWISE << (BEBOP_BLDC_MOTORS_NUM - 1 - BEBOP_BLDC_RIGHT_BACK))|
                            (BEBOP_BLDC_COUNTERCLOCKWISE  << (BEBOP_BLDC_MOTORS_NUM - 1 - BEBOP_BLDC_RIGHT_FRONT))|
                            (BEBOP_BLDC_CLOCKWISE << (BEBOP_BLDC_MOTORS_NUM - 1 - BEBOP_BLDC_LEFT_FRONT));


#define BEBOP_BLDC_SETREFSPEED 0x02
struct bldc_ref_speed_data {
    uint8_t     cmd;
    uint16_t    rpm[BEBOP_BLDC_MOTORS_NUM];
    uint8_t     enable_security;
    uint8_t     checksum;
}__attribute__((packed));

#define BEBOP_BLDC_GETOBSDATA 0x20
struct bldc_obs_data {
    uint16_t    rpm[BEBOP_BLDC_MOTORS_NUM];
    uint16_t    batt_mv;
    uint8_t     status;
    uint8_t     errno;
    uint8_t     motors_err;
    uint8_t     temp;
    uint8_t     checksum;
}__attribute__((packed));

/* description of the bldc status */
#define BEBOP_BLDC_STATUS_INIT          0
#define BEBOP_BLDC_STATUS_IDLE          1
#define BEBOP_BLDC_STATUS_RAMPING       2
#define BEBOP_BLDC_STATUS_SPINNING_1    3
#define BEBOP_BLDC_STATUS_SPINNING_2    4
#define BEBOP_BLDC_STATUS_STOPPING      5
#define BEBOP_BLDC_STATUS_CRITICAL      6

/* description of the bldc errno */
#define BEBOP_BLDC_ERRNO_EEPROM         1
#define BEBOP_BLDC_ERRNO_MOTOR_STALLED  2
#define BEBOP_BLDC_ERRNO_PROP_SECU      3
#define BEBOP_BLDC_ERRNO_COM_LOST       4
#define BEBOP_BLDC_ERRNO_BATT_LEVEL     9
#define BEBOP_BLDC_ERRNO_LIPO           10
#define BEBOP_BLDC_ERRNO_MOTOR_HW       11

#define BEBOP_BLDC_TOGGLE_GPIO 0x4d
#define     BEBOP_BLDC_GPIO_RESET   (1 << 0)
#define     BEBOP_BLDC_GPIO_RED     (1 << 1)
#define     BEBOP_BLDC_GPIO_GREEN   (1 << 2)

#define BEBOP_BLDC_STOP_PROP 0x60

#define BEBOP_BLDC_CLEAR_ERROR 0x80

#define BEBOP_BLDC_PLAY_SOUND 0x82

#define BEBOP_BLDC_GET_INFO 0xA0

/* Bebop is a Quad X so the channels are :
 * 1 = Front Right
 * 2 = Back Left
 * 3 = Front Left
 * 4 = Back Right
 *
 * but the channels start at 0 so it is channel num - 1
*/
static const uint8_t bebop_bldc_motors[BEBOP_BLDC_MOTORS_NUM] = { BEBOP_BLDC_RIGHT_FRONT,
                                                                  BEBOP_BLDC_LEFT_BACK,
                                                                  BEBOP_BLDC_LEFT_FRONT,
                                                                  BEBOP_BLDC_RIGHT_BACK };

#define BEBOP_BLDC_MIN_PERIOD_US 1100
#define BEBOP_BLDC_MAX_PERIOD_US 1900
#define BEBOP_BLDC_MIN_RPM 3000
#define BEBOP_BLDC_MAX_RPM 11000

enum {
    BEBOP_BLDC_STARTED,
    BEBOP_BLDC_STOPPED,
};

using namespace Linux;

static const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

LinuxRCOutput_Bebop::LinuxRCOutput_Bebop():
    _i2c_sem(NULL),
    _min_pwm(BEBOP_BLDC_MIN_PERIOD_US),
    _max_pwm(BEBOP_BLDC_MAX_PERIOD_US),
    _state(BEBOP_BLDC_STOPPED)
{
    memset(_period_us, 0, sizeof(_period_us));
    memset(_rpm, 0, sizeof(_rpm));
}

uint8_t LinuxRCOutput_Bebop::_checksum(uint8_t *data, unsigned int len)
{
    uint8_t checksum = data[0];
    unsigned int i;

    for(i = 1; i < len; i++)
        checksum = checksum ^ data[i];

    return checksum;
}

void LinuxRCOutput_Bebop::_start_prop()
{
    uint8_t data = BEBOP_BLDC_STARTPROP;

    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->write(BEBOP_BLDC_I2C_ADDR, 1, &data);

    _i2c_sem->give();
    _state = BEBOP_BLDC_STARTED;
}

void LinuxRCOutput_Bebop::_set_ref_speed(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM])
{
    struct bldc_ref_speed_data data;
    int i;

    data.cmd = BEBOP_BLDC_SETREFSPEED;

    for(i=0; i<BEBOP_BLDC_MOTORS_NUM; i++)
        data.rpm[i] = htobe16(rpm[i]);

    data.enable_security = 0;
    data.checksum = _checksum((uint8_t *) &data, sizeof(data) - 1);

    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->write(BEBOP_BLDC_I2C_ADDR, sizeof(data), (uint8_t *)&data);

    _i2c_sem->give();
}

void LinuxRCOutput_Bebop::_get_obs_data(uint16_t rpm[BEBOP_BLDC_MOTORS_NUM],
                                        uint16_t *batt_mv,
                                        uint8_t *status,
                                        uint8_t *errno,
                                        uint8_t *motors_err,
                                        uint8_t *temp)
{
    struct bldc_obs_data data;
    int i;

    memset(&data, 0, sizeof(data));

    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->readRegisters(BEBOP_BLDC_I2C_ADDR,
                            BEBOP_BLDC_GETOBSDATA,
                            sizeof(data),
                            (uint8_t *)&data);

    if(data.checksum != _checksum((uint8_t *)&data, sizeof(data)))
        hal.console->println_P(PSTR("RCOutput_Bebop: bad checksum in obs data"));

    if(rpm != NULL) {
        for(i=0; i<BEBOP_BLDC_MOTORS_NUM; i++)
            rpm[i] = be16toh(data.rpm[i]);
    }

    if(batt_mv != NULL)
        *batt_mv = be16toh(data.batt_mv);

    if(status != NULL)
        *status = data.status;

    if(errno != NULL)
        *errno = data.errno;

    if(motors_err != NULL)
        *motors_err = data.motors_err;

    if(temp != NULL)
        *temp = data.temp;
}

void LinuxRCOutput_Bebop::_toggle_gpio(uint8_t mask)
{
    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->writeRegister(BEBOP_BLDC_I2C_ADDR, BEBOP_BLDC_TOGGLE_GPIO, mask);

    _i2c_sem->give();
}

void LinuxRCOutput_Bebop::_stop_prop()
{
    uint8_t data = BEBOP_BLDC_STOP_PROP;
    _state = BEBOP_BLDC_STOPPED;

    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->write(BEBOP_BLDC_I2C_ADDR, 1, &data);

    _i2c_sem->give();
}

void LinuxRCOutput_Bebop::_clear_error()
{
    uint8_t data = BEBOP_BLDC_CLEAR_ERROR;

    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->write(BEBOP_BLDC_I2C_ADDR, 1, &data);

    _i2c_sem->give();
}

void LinuxRCOutput_Bebop::_play_sound(uint8_t sound)
{
    if(!_i2c_sem->take(10))
        return;

    hal.i2c1->writeRegister(BEBOP_BLDC_I2C_ADDR, BEBOP_BLDC_PLAY_SOUND, sound);

    _i2c_sem->give();
}

uint16_t LinuxRCOutput_Bebop::_period_us_to_rpm(uint16_t period_us)
{
    float period_us_fl = period_us;

    float rpm_fl = (period_us_fl - _min_pwm)/(_max_pwm - _min_pwm) *
                    (BEBOP_BLDC_MAX_RPM - BEBOP_BLDC_MIN_RPM) + BEBOP_BLDC_MIN_RPM;

    return (uint16_t)rpm_fl;
}

void LinuxRCOutput_Bebop::init(void* dummy)
{
    _i2c_sem = hal.i2c1->get_semaphore();
    if (_i2c_sem == NULL) {
        hal.scheduler->panic(PSTR("RCOutput_Bebop: can't get i2c sem"));
        return; /* never reached */
    }

    _clear_error();

    /* Set an initial dummy frequency */
    _frequency = 50;
}

void LinuxRCOutput_Bebop::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    _frequency = freq_hz;
}

uint16_t LinuxRCOutput_Bebop::get_freq(uint8_t ch)
{
    return _frequency;
}

void LinuxRCOutput_Bebop::enable_ch(uint8_t ch)
{
}

void LinuxRCOutput_Bebop::disable_ch(uint8_t ch)
{
    _stop_prop();
}

void LinuxRCOutput_Bebop::write(uint8_t ch, uint16_t period_us)
{
    unsigned int i;
    uint8_t motor = bebop_bldc_motors[ch];

    _period_us[ch] = period_us;
    _rpm[motor] = _period_us_to_rpm(period_us);

    /* start propellers if the speed of the 4 motors is >= min speed
     * not clear when the motors are supposed to start, so let's put it to min_pwm*/
    for(i = 0; i < BEBOP_BLDC_MOTORS_NUM; i++) {
        if(_period_us[i] <= _min_pwm + 50)
            break;
    }

    if(i < BEBOP_BLDC_MOTORS_NUM) {
        /* one motor pwm value is at minimum (or under)
         * if the motors are started, stop them*/
        if(_state == BEBOP_BLDC_STARTED)
            _stop_prop();
    }
    else {
        /* all the motor pwm values are higher than minimum
         * if the bldc is stopped, start it*/
        if(_state == BEBOP_BLDC_STOPPED)
            _start_prop();
    }
    _set_ref_speed(_rpm);
}

void LinuxRCOutput_Bebop::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        write(ch + i, period_us[i]);
}

uint16_t LinuxRCOutput_Bebop::read(uint8_t ch)
{
    if(ch < BEBOP_BLDC_MOTORS_NUM) {
        return _period_us[ch];
    }
    else {
        return 0;
    }
}

void LinuxRCOutput_Bebop::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++)
        period_us[i] = read(0 + i);
}

void LinuxRCOutput_Bebop::set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm)
{
    _min_pwm = min_pwm;
    _max_pwm = max_pwm;
}
#endif

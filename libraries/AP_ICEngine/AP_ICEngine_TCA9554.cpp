#include "AP_ICEngine_config.h"

/*
  support for TCA9554 for starter control on I2C
 */

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
#include "AP_ICEngine.h"

extern const AP_HAL::HAL& hal;

/*
 * TCA9554 output register mapping for PMB Rev E
 * P0 = PMU_EN - PMU output ON/OFF  (CN6 pin 2)
 * P1 = ECU_EN - Unused (previously Engine Kill Switch)
 * P2 = I2C_P2 - Unused
 * P3 = LED (active low)
 * P4 = PMU_START - Crank Direction (CN6 pin 5)
 * P5 = PMU_ARM  - Crank Signal (CN6 pin 6)
 * P6 = PMU_STAT_IN - Unused
 * P7 = PMU_STAT - Unused
 */
#define TCA9554_I2C_BUS      1
#define TCA9554_I2C_ADDR     0x20
#define TCA9554_OUTPUT       0x01  // Output Port register address. Outgoing logic levels
#define TCA9554_OUT_DEFAULT  0x30  // 0011 0000
#define TCA9554_CONF         0x03  // Configuration Port register address [0 = Output]
#define TCA9554_PINS         0xC2  // Set all used ports to outputs = 1100 0010

/*
  initialise TCA9554
 */
bool AP_ICEngine_TCA9554::TCA9554_init()
{
    dev_TCA9554 = std::move(hal.i2c_mgr->get_device(TCA9554_I2C_BUS, TCA9554_I2C_ADDR));
    if (!dev_TCA9554) {
        return false;
    }
    WITH_SEMAPHORE(dev_TCA9554->get_semaphore());

    // setup 1 checked registers
    dev_TCA9554->setup_checked_registers(1);

    dev_TCA9554->set_retries(10);

    // set outputs
    bool ret = dev_TCA9554->write_register(TCA9554_OUTPUT, TCA9554_OUT_DEFAULT);
    if (!ret) {
        return false;
    }
    ret = dev_TCA9554->write_register(TCA9554_CONF, TCA9554_PINS, true);
    if (!ret) {
        return false;
    }

    dev_TCA9554->set_retries(1);
    return true;
}

/*
  set the state of the i2c controller
 */
void AP_ICEngine_TCA9554::TCA9554_set(TCA9554_state_t value)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_reg_check_ms > 100) {
        /*
          register checking at 10Hz allows us to cope with the i2c
          device being power cycled after boot
         */
        last_reg_check_ms = now_ms;
        WITH_SEMAPHORE(dev_TCA9554->get_semaphore());
        dev_TCA9554->check_next_register();
    }

    if (value != last_state) {
        WITH_SEMAPHORE(dev_TCA9554->get_semaphore());
        // set outputs and status leds
        if (dev_TCA9554->write_register(TCA9554_OUTPUT, (~(value<<2) & 0x0C) | value)) {
            last_state = value;
        }
    }
}

void AP_ICEngine_TCA9554::set_starter(bool on, bool crank_dir_reverse)
{
    if (!initialised) {
        initialised = TCA9554_init();
        if (!initialised) {
            // waiting for power to PMU
            return;
        }
    }
    if (!crank_dir_reverse) {
        TCA9554_set(on? STARTER_FORWARD : STARTER_OFF);
    } else {
        TCA9554_set(on? STARTER_REVERSE : STARTER_OFF);
    }
}

#endif // AP_ICENGINE_TCA9554_STARTER_ENABLED

#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_MS56XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_MS5611_I2C_ADDR
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5611_I2C_ADDR2
#define HAL_BARO_MS5611_I2C_ADDR2 0x76
#endif

#ifndef HAL_BARO_MS5607_I2C_ADDR
#define HAL_BARO_MS5607_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5837_I2C_ADDR
#define HAL_BARO_MS5837_I2C_ADDR 0x76
#endif

#ifndef HAL_BARO_MS5637_I2C_ADDR
#define HAL_BARO_MS5637_I2C_ADDR 0x76
#endif

#if AP_BARO_MS5837_ENABLED
// Determined in https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
#define MS5837_30BA_02BA_SELECTION_THRESHOLD 37000
#endif

class AP_Baro_MS56XX : public AP_Baro_Backend
{
public:
    void update() override;

    AP_Baro_MS56XX(AP_Baro &baro, AP_HAL::Device &dev);

protected:

    // convenience methods for derivative classes to call.  Will free
    // sensor if it can't init it.
    static AP_Baro_Backend *_probe(AP_Baro &baro, AP_Baro_MS56XX *sensor);

    virtual bool _init();

    bool _read_prom_5611(uint16_t prom[8]);
    bool _read_prom_5637(uint16_t prom[8]);

    virtual const char *name() const = 0;
    virtual DevTypes devtype() const = 0;

    uint8_t _instance;

    /* Last compensated values from accumulated sample */
    float _D1, _D2;

    // Internal calibration registers
    struct {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;

private:

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);

    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    void _timer();

    AP_HAL::Device *_dev;

    /* Shared values between thread sampling the HW and main thread */
    struct {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    uint8_t _state;

    bool _discard_next;

    virtual bool _read_prom(uint16_t *prom) = 0;
    virtual void _calculate() = 0;
};

#if AP_BARO_MS5607_ENABLED
class AP_Baro_MS5607 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);
protected:
    const char *name() const override { return "MS5607"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5611(prom); }
    DevTypes devtype() const override { return DEVTYPE_BARO_MS5607; }
    void _calculate() override;
};
#endif  // AP_BARO_MS5607_ENABLED

#if AP_BARO_MS5611_ENABLED
class AP_Baro_MS5611 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);
protected:
    const char *name() const override { return "MS5611"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5611(prom); }
    DevTypes devtype() const override { return DEVTYPE_BARO_MS5611; }
    void _calculate() override;
};
#endif  // AP_BARO_MS5611_ENABLED

#if AP_BARO_MS5637_ENABLED
class AP_Baro_MS5637 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);
protected:
    const char *name() const override { return "MS5637"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5637(prom); }
    DevTypes devtype() const override { return DEVTYPE_BARO_MS5637; }
    void _calculate() override;
};
#endif  // AP_BARO_MS5637_ENABLED

#if AP_BARO_MS5837_ENABLED
class AP_Baro_MS5837 : public AP_Baro_MS56XX
{
public:
    using AP_Baro_MS56XX::AP_Baro_MS56XX;
    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::Device &dev);
protected:
    const char *name() const override { return "MS5837"; }
    bool _read_prom(uint16_t *prom) override { return _read_prom_5637(prom); }
    DevTypes devtype() const override;
    bool _init() override;
    void _calculate() override;
    void _calculate_5837_02ba();
    void _calculate_5837_30ba();

    DevTypes _subtype;
};
#endif  // AP_BARO_MS5837_ENABLED

#endif  // AP_BARO_MS56XX_ENABLED

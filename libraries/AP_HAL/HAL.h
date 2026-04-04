#pragma once

class AP_Param;

#include "AP_HAL_Namespace.h"

#include "AnalogIn.h"
#include "GPIO.h"
#include "RCInput.h"
#include "RCOutput.h"
#include "SPIDevice.h"
#include "WSPIDevice.h"
#include "Storage.h"
#include "UARTDriver.h"
#include "system.h"
#include "OpticalFlow.h"
#include "DSP.h"
#include "CANIface.h"


class AP_HAL::HAL {
public:
    HAL(AP_HAL::UARTDriver* _serial0, // console
        AP_HAL::UARTDriver* _serial1, // telem1
        AP_HAL::UARTDriver* _serial2, // telem2
        AP_HAL::UARTDriver* _serial3, // 1st GPS
        AP_HAL::UARTDriver* _serial4, // 2nd GPS
        AP_HAL::UARTDriver* _serial5, // extra1
        AP_HAL::UARTDriver* _serial6, // extra2
        AP_HAL::UARTDriver* _serial7, // extra3
        AP_HAL::UARTDriver* _serial8, // extra4
        AP_HAL::UARTDriver* _serial9, // extra5
        AP_HAL::I2CDeviceManager* _i2c_mgr,
        AP_HAL::SPIDeviceManager* _spi,
        AP_HAL::WSPIDeviceManager* _wspi,
        AP_HAL::AnalogIn*   _analogin,
        AP_HAL::Storage*    _storage,
        AP_HAL::UARTDriver* _console,
        AP_HAL::GPIO*       _gpio,
        AP_HAL::RCInput*    _rcin,
        AP_HAL::RCOutput*   _rcout,
        AP_HAL::Scheduler*  _scheduler,
        AP_HAL::Util*       _util,
        AP_HAL::OpticalFlow*_opticalflow,
        AP_HAL::Flash*      _flash,
#if AP_SIM_ENABLED
        // note that not much of _simstate is used on AP_HAL_SITL, but
        // more is being moved in!
        class AP_HAL::SIMState*   _simstate,
#endif
#if HAL_WITH_DSP
        AP_HAL::DSP*        _dsp,
#endif
#if HAL_NUM_CAN_IFACES > 0
        AP_HAL::CANIface* _can_ifaces[HAL_NUM_CAN_IFACES])
#else
        AP_HAL::CANIface** _can_ifaces)
#endif
        :
        i2c_mgr(_i2c_mgr),
        spi(_spi),
        wspi(_wspi),
        analogin(_analogin),
        storage(_storage),
        console(_console),
        gpio(_gpio),
        rcin(_rcin),
        rcout(_rcout),
        scheduler(_scheduler),
        util(_util),
        opticalflow(_opticalflow),
        flash(_flash),
#if HAL_WITH_DSP
        dsp(_dsp),
#endif
        serial_array{
            _serial0,
            _serial1,
            _serial2,
            _serial3,
            _serial4,
            _serial5,
            _serial6,
            _serial7,
            _serial8,
            _serial9}
#if AP_SIM_ENABLED
            ,simstate(_simstate)
#endif
    {
#if HAL_NUM_CAN_IFACES > 0
        if (_can_ifaces == nullptr) {
            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
                can[i] = nullptr;
        } else {
            for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++)
                can[i] = _can_ifaces[i];
        }
#endif

        AP_HAL::init();
    }

    struct Callbacks {
        virtual void setup() = 0;
        virtual void loop() = 0;
    };

    struct FunCallbacks : public Callbacks {
        FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void));

        void setup() override { _setup(); }
        void loop() override { _loop(); }

    private:
        void (*_setup)(void);
        void (*_loop)(void);
    };

    virtual void run(int argc, char * const argv[], Callbacks* callbacks) const = 0;

public:
    AP_HAL::I2CDeviceManager* i2c_mgr;
    AP_HAL::SPIDeviceManager* spi;
    AP_HAL::WSPIDeviceManager* wspi;
    AP_HAL::AnalogIn*   analogin;
    AP_HAL::Storage*    storage;
    AP_HAL::UARTDriver* console;
    AP_HAL::GPIO*       gpio;
    AP_HAL::RCInput*    rcin;
    AP_HAL::RCOutput*   rcout;
    AP_HAL::Scheduler*  scheduler;
    AP_HAL::Util        *util;
    AP_HAL::OpticalFlow *opticalflow;
    AP_HAL::Flash       *flash;
    AP_HAL::DSP         *dsp;
#if HAL_NUM_CAN_IFACES > 0
    AP_HAL::CANIface* can[HAL_NUM_CAN_IFACES];
#else
    AP_HAL::CANIface** can;
#endif

    // access to serial ports using SERIALn_ numbering
    UARTDriver* serial(uint8_t sernum) const;

    static constexpr uint8_t num_serial = 10;

private:
    // UART drivers in SERIALn_ order
    AP_HAL::UARTDriver* serial_array[num_serial];

public:
#if AP_SIM_ENABLED
    AP_HAL::SIMState *simstate;
#endif

#ifndef HAL_CONSOLE_DISABLED
# define DEV_PRINTF(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while(0)
#else
# define DEV_PRINTF(fmt, args ...)
#endif

};

// access serial ports using SERIALn numbering
inline AP_HAL::UARTDriver* AP_HAL::HAL::serial(uint8_t sernum) const
{
    if (sernum >= ARRAY_SIZE(serial_array)) {
        return nullptr;
    }
    return serial_array[sernum];
}

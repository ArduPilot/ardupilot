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
    HAL(AP_HAL::UARTDriver* _uartA, // console
        AP_HAL::UARTDriver* _uartB, // 1st GPS
        AP_HAL::UARTDriver* _uartC, // telem1
        AP_HAL::UARTDriver* _uartD, // telem2
        AP_HAL::UARTDriver* _uartE, // 2nd GPS
        AP_HAL::UARTDriver* _uartF, // extra1
        AP_HAL::UARTDriver* _uartG, // extra2
        AP_HAL::UARTDriver* _uartH, // extra3
        AP_HAL::UARTDriver* _uartI, // extra4
        AP_HAL::UARTDriver* _uartJ, // extra5
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
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        class AP_HAL::SIMState*   _simstate,
#endif
        AP_HAL::DSP*        _dsp,
#if HAL_NUM_CAN_IFACES > 0
        AP_HAL::CANIface* _can_ifaces[HAL_NUM_CAN_IFACES])
#else
        AP_HAL::CANIface** _can_ifaces)
#endif
        :
        uartA(_uartA),
        uartB(_uartB),
        uartC(_uartC),
        uartD(_uartD),
        uartE(_uartE),
        uartF(_uartF),
        uartG(_uartG),
        uartH(_uartH),
        uartI(_uartI),
        uartJ(_uartJ),
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
#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
        simstate(_simstate),
#endif
        dsp(_dsp)
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

private:
    // the uartX ports must be contiguous in ram for the serial() method to work
    AP_HAL::UARTDriver* uartA;
    AP_HAL::UARTDriver* uartB UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartC UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartD UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartE UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartF UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartG UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartH UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartI UNUSED_PRIVATE_MEMBER;
    AP_HAL::UARTDriver* uartJ UNUSED_PRIVATE_MEMBER;

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

#if AP_SIM_ENABLED && CONFIG_HAL_BOARD != HAL_BOARD_SITL
    AP_HAL::SIMState *simstate;
#endif

#ifndef HAL_CONSOLE_DISABLED
# define DEV_PRINTF(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while(0)
#else
# define DEV_PRINTF(fmt, args ...)
#endif

};

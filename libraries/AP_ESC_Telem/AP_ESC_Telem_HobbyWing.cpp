#include "AP_ESC_Telem_config.h"

#if AP_ESC_TELEM_HOBBYWING_ENABLED

#include "AP_ESC_Telem_HobbyWing.h"
#include "AP_HobbyWing_ESC_Platinum_PRO_v3.h"
#include "AP_HobbyWing_ESC_Platinum_v4.h"
#include "AP_HobbyWing_ESC_XRotor_v4.h"

#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ESC_Telem_HobbyWing::var_info[] = {
#if AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED
    // @Param: V3_MASK
    // @DisplayName: HobbyWingESC v3 Channel Bitmask
    // @Description: Mask of which channels are returning data on serial ports
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("V3_MASK",  2, AP_ESC_Telem_HobbyWing, channel_mask_v3, 0),

    // @Param: V3_POLES
    // @DisplayName: HobbyWingESC v3 Motor Poles
    // @Description: This allows calculation of true RPM from ESC's eRPM. The default is 14.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("V3_POLES",  3, AP_ESC_Telem_HobbyWing, motor_poles_v3, 14),
#endif  // AP_HOBBYWING_PLATINUM_PRO_V3_ENABLED

#if AP_HOBBYWING_PLATINUM_V4_ENABLED
    // @Param: V4_MASK
    // @DisplayName: HobbyWingESC v4 Channel Bitmask
    // @Description: Mask of which channels are returning data on serial ports
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("V4_MASK",  12, AP_ESC_Telem_HobbyWing, channel_mask_v4, 0),

    // @Param: V4_POLES
    // @DisplayName: HobbyWingESC v4 Motor Poles
    // @Description: This allows calculation of true RPM from ESC's eRPM. The default is 14.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("V4_POLES",  13, AP_ESC_Telem_HobbyWing, motor_poles_v4, 14),
#endif  // AP_HOBBYWING_PLATINUM_V4_ENABLED

#if AP_HOBBYWING_XROTOR_V4_ENABLED
    // @Param: X4_MASK
    // @DisplayName: HobbyWingESC XRotor v4 Channel Bitmask
    // @Description: Mask of which channels are returning data on serial ports
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("X4_MASK",  22, AP_ESC_Telem_HobbyWing, channel_mask_x4, 0),

    // @Param: X4_POLES
    // @DisplayName: HobbyWingESC XRotor v4 Motor Poles
    // @Description: This allows calculation of true RPM from ESC's eRPM. The default is 14.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("X4_POLES",  23, AP_ESC_Telem_HobbyWing, motor_poles_x4, 14),
#endif  // AP_HOBBYWING_XROTOR_V4_ENABLED

    AP_GROUPEND
};

// constructor
AP_ESC_Telem_HobbyWing::AP_ESC_Telem_HobbyWing()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ESC_Telem_HobbyWing::thread_main()
{
    // initialise all escs; this will set the UART up in the thread
    for (uint8_t i=0; i<num_escs; i++) {
        escs[i]->init();
    }

    while (true) {
        hal.scheduler->delay_microseconds(1500);
        for (uint8_t i=0; i<num_escs; i++) {
            escs[i]->update();
        }
    }
}

void AP_ESC_Telem_HobbyWing::init()
{
    for (uint8_t i=0; i<32 && num_escs<ARRAY_SIZE(escs); i++) {
        // work out our servo channel offset.  First instance of this
        // class gets the channel corresponding to the first bit set in
        // the mask etc
        uint8_t version = 0;
        uint8_t poles;
        if (((uint32_t)channel_mask_v3 & (1U<<i))) {
            version = 3;
            poles = motor_poles_v3;
        } else if (((uint32_t)channel_mask_v4 & (1U<<i))) {
            version = 4;
            poles = motor_poles_v4;
        } else if (((uint32_t)channel_mask_x4 & (1U<<i))) {
            version = 5;
            poles = motor_poles_x4;
        } else {
            continue;
        }

        // see if there's another serial port configured for the correct protocol:
        AP_HAL::UARTDriver *uart { AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_HobbyWing, num_escs) };
        if (uart == nullptr) {
            break;
        }
        switch (version) {
        case 3:
            escs[num_escs] = new AP_HobbyWing_Platinum_PRO_v3(*uart);
            break;
        case 4:
            escs[num_escs] = new AP_HobbyWing_Platinum_v4(*uart);
            break;
        case 5:
            escs[num_escs] = new AP_HobbyWing_XRotor_v4(*uart);
            break;
        }
        if (escs[num_escs] == nullptr) {
            break;
        }
        escs[num_escs]->set_poles(poles);
        servo_channel[num_escs++] = i+1;
    }

    if (num_escs == 0) {
        return;
    }

    // start a thread to handle the reading from all of those UARTs:
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND_MEMBER(&AP_ESC_Telem_HobbyWing::thread_main, void),
            "HobbyWing",
            512, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        DEV_PRINTF("Failed to create HobbyWing thread\n");
    }
}

void AP_ESC_Telem_HobbyWing::update_telemetry()
{
    for (uint8_t i=0; i<num_escs; i++) {
        AP_HobbyWing_ESC &esc = *escs[i];
        // FIXME: packet freshness?
        AP_HobbyWing_ESC::HWESC decoded {
            rpm: nanf(""),
            voltage: nanf(""),
            phase_current: nanf(""),
            current: nanf(""),
            temperature: nanf("")
        };
        if (!esc.get_telem(decoded)) {
            continue;
        }

        TelemetryData t;
        uint16_t data_mask = 0;
        if (!isnan(decoded.temperature)) {
            t.temperature_cdeg = int16_t(decoded.temperature * 100);
            data_mask |= TelemetryType::TEMPERATURE;
        }
        if (!isnan(decoded.voltage)) {
            t.voltage = decoded.voltage;
            data_mask |= TelemetryType::VOLTAGE;
        }
        if (!isnan(decoded.current)) {
            t.current = decoded.current;
            data_mask |= TelemetryType::CURRENT;
        }

        // ESC telem library wants an offset, not a channel number, thus -1:
        const uint8_t motor = servo_channel[i] - 1;

        update_telem_data(motor, t, data_mask);

        if (!isnan(decoded.rpm)) {
            update_rpm(motor, decoded.rpm, 0);
        }
    }
}
#endif  // AP_ESC_TELEM_HOBBYWING_ENABLED

#include "SIM_config.h"

#if AP_SIM_RF_BENEWAKE_TFMINIPLUS_ENABLED

#include "SIM_RF_Benewake_TFMiniPlus.h"

#include <stdio.h>

void SITL::Benewake_TFMiniPlus::init()
{
}

int SITL::Benewake_TFMiniPlus::rdwr(I2C::i2c_rdwr_ioctl_data *&data)
{
    if (data->nmsgs != 1) {
        AP_HAL::panic("Only ever expect a single message from driver");
    }
    auto msg = data->msgs[0];
    if (msg.flags == I2C_M_RD) {
        // an attempt to retrieve data which was previously requested
        const auto data_type_requested_for_processing = data_type_requested;
        data_type_requested = DataTypeRequested::NONE;
        switch (data_type_requested_for_processing) {
        case DataTypeRequested::NONE:
            AP_HAL::panic("Request to read data when none requested");
        case DataTypeRequested::FW_VERSION: {
            if (msg.len != 7) {
                AP_HAL::panic("Invalid read length");
            }
            msg.buf[0] = 0x5A;
            msg.buf[1] = 0x07;
            msg.buf[2] = 0x01;
            msg.buf[3] = 1;
            msg.buf[4] = 2;
            msg.buf[5] = 3;
            msg.buf[6] = crc_sum_of_bytes(msg.buf, 6);
            msg.len = 7;
            return 0;
        }
        case DataTypeRequested::READING: {
            if (msg.len != 11) {
                AP_HAL::panic("Invalid read length");
            }
            if (!running_config.data_output_enabled) {
                AP_HAL::panic("Request for data before enabling output?!");
            }
            float tmp_range = range;
            if (!running_config.output_format_is_cm) {
                tmp_range /= 4.5;  // convert to decicubits
            }
            const uint16_t strength = 100;
            uint16_t dist_cm = tmp_range * 100;
            msg.buf[0] = 0x59;
            msg.buf[1] = 0x59;
            msg.buf[2] = dist_cm & 0xff;
            msg.buf[3] = dist_cm >> 8;
            msg.buf[4] = strength & 0xff;
            msg.buf[5] = strength >> 8;
            msg.buf[6] = 0;  // timestamp
            msg.buf[7] = 0;  // timestamp
            msg.buf[8] = 0;  // timestamp
            msg.buf[9] = 0;  // timestamp
            msg.buf[10] = crc_sum_of_bytes(msg.buf, 10);
            msg.len = 11;
            return 0;
        }
        }
        return -1;
    }

    // an attempt to retrieve data which was previously requested
    if (msg.buf[0] != 0x5A) {
        AP_HAL::panic("Unexpected header");
    }
    const uint8_t len = msg.buf[1];
    if (len > msg.len) {
        AP_HAL::panic("Length too long");
    }
    if (crc_sum_of_bytes(msg.buf, len-1) != msg.buf[len-1]) {
        AP_HAL::panic("Invalid checksum");
    }
    switch (Command(msg.buf[2])) {
    case Command::FW_VERSION:
        data_type_requested = DataTypeRequested::FW_VERSION;
        break;
    case Command::SYSTEM_RESET:
        reset();
        break;
    case Command::OUTPUT_FORMAT_CM:
        new_config.output_format_is_cm = (msg.buf[3] == 1);
        break;
    case Command::ENABLE_DATA_OUTPUT:
        new_config.data_output_enabled = (msg.buf[3] == 1);
        break;
    case Command::SET_FRAME_RATE:
        new_config.frame_rate = (msg.buf[3] | msg.buf[4]<<8);
        break;
    case Command::SAVE_SETTINGS:
        // whatever
        break;
    case Command::READ_MEASUREMENT:
        if (msg.buf[3] != 0x07) {
            AP_HAL::panic("Invalid number which I don't know anything about");
        }
        data_type_requested = DataTypeRequested::READING;
        break;
    }
    return 0;
}

#endif  // AP_SIM_RF_BENEWAKE_TFMINIPLUS_ENABLED

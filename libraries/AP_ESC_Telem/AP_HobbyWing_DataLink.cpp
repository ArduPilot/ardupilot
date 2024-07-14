/*
  ESC Telemetry for Hobbywing DataLink
 */
#include "AP_HobbyWing_DataLink.h"

#if AP_HOBBYWING_DATALINK_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

const AP_Param::GroupInfo AP_HobbyWing_DataLink::var_info[] = {
    // @Param: _OFS
    // @DisplayName: HobbyWing DataLink ESC offset
    // @Description: Used to match incoming ESC telemetry with servo channel number.  If this is 8 then the ESC telemetry coming in from the device plugged into the DataLink device's "1" pins will be matched against ArduPilot's SERVO9 outputs.
    // @Range: 0 18
    // @User: Advanced
    AP_GROUPINFO("_OFS",  10, AP_HobbyWing_DataLink, offset, 0),

    // @Param: _PLS
    // @DisplayName: HobbyWing DataLink motor pole count
    // @Description: This allows calculation of true RPM from ESC's eRPM. The default is 14.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
   AP_GROUPINFO("_PLS",  11, AP_HobbyWing_DataLink, motor_poles, 14),

    AP_GROUPEND
};

void AP_HobbyWing_DataLink::init()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AP_HobbyWing_DataLink::init");
    uart.begin(115200);
    uart.set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX);

    // initialise flag values in decoded data:
    for (uint8_t i=0; i<ARRAY_SIZE(decoded); i++) {
        decoded[i].rpm = nanf("");
        decoded[i].voltage = nanf("");
        decoded[i].phase_current = nanf("");
        decoded[i].current = nanf("");
        decoded[i].temperature = nanf("");
    }
}

void AP_HobbyWing_DataLink::update()
{
    uint8_t* data = (uint8_t*)&packet;

    if(header_read <= 0)
    {
        header_read = readHeader();

        if(header_read <= 0)
        {
            //header not found
            return;
        }
    }

    // header read successfully
    uint32_t bytes_to_read = PACKAGE_SIZE - header_read;
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Header read=%d bytes to read=%d", header_read, bytes_to_read);

    // header read successfully, read rest of the packet
    auto available = uart.available();
    if(available < bytes_to_read)
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Available bytes less then bytes to read expected=%d available=%ld", bytes_to_read, available);
        return;
    }
    auto bytes_read = uart.read(&data[header_read], bytes_to_read);
    if(bytes_read != (int)bytes_to_read) {
        header_read = 0; // header read not successfully
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Bytes read not succesfully expected=%ld actual=%ld", (long int)bytes_to_read, (long int)bytes_read);
        return;
    }

    //Set header read 0, to show we must find header again
    header_read = 0;

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Bytes read succesfully");
    uint16_t packet_checksum = packet.calc_checksum();
    if (packet_checksum != packet.crc) {
        // checksum failure
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Checksum failure expected=%d actual=%d", packet.crc, packet_checksum);
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Bytes in the end of the package [155]=%02x [156]=%02x [157]=%02x [158]=%02x [159]=%02x",
            // data[155], data[156],  data[157],  data[158],  data[159]);
        return;
    }

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Update telemetry");

    // extract packet sequence number and update count of dropped frames
    check_seq(be16toh(packet.counter));

    WITH_SEMAPHORE(decoded_sem);

    for (uint8_t i=0; i<ARRAY_SIZE(packet.esc_data); i++) {
        const auto &esc_data = packet.esc_data[i];
        auto &info = esc_info[i];
        if (esc_data.counter == 0 && !info.seen) {
            continue;
        }
        info.seen = true;

        // decoded.counter = be32toh(pkt.counter);
        // decoded.throttle_req = be16toh(pkt.throttle_req);
        // decoded.throttle = be16toh(pkt.throttle);
        decoded[i].rpm = be16toh(esc_data.rpm) / motor_poles * 10;
        decoded[i].voltage = be16toh(esc_data.voltage) * 0.1;
        decoded[i].phase_current = int16_t(be16toh(esc_data.phase_current)) * 0.01;
        decoded[i].current = int16_t(be16toh(esc_data.current)) * 0.01;
        decoded[i].temperature = MAX(AP_ESC_Telem_Backend::temperature_decode(esc_data.mos_temperature),
                                     AP_ESC_Telem_Backend::temperature_decode(esc_data.cap_temperature));
        const uint16_t status = be16toh(esc_data.status);
        if (status != 0) {
            decoded[i].error_count++;
        }
    }
    decoded_received_us = AP_HAL::micros();
}

void AP_HobbyWing_DataLink::update_telemetry()
{
    HWESC decodeds[8];
    AP_HobbyWing_DataLink::ESCInfo infos[8];
    if (!get_telem(decodeds, infos)) {
        return;
    }
    for (uint8_t i=0; i<ARRAY_SIZE(infos); i++) {
        const auto &info = infos[i];
        if (!info.seen) {
            continue;
        }
        const auto &decoded_data = decodeds[i];

        update_motor_data_from_HWESC(offset + i, decoded_data);
    }
}

bool AP_HobbyWing_DataLink::get_telem(HWESC hwesc[8], AP_HobbyWing_DataLink::ESCInfo info[8])
{
    WITH_SEMAPHORE(decoded_sem);
    if (decoded_received_us == 0) {
        return false;
    }
    // only use once:
    decoded_received_us = 0;
    memcpy(hwesc, decoded, 8 * sizeof(HWESC));
    memcpy(info, esc_info, 8 * sizeof(AP_HobbyWing_DataLink::ESCInfo));
    return true;
}

void AP_HobbyWing_DataLink::check_seq(uint16_t this_seq)
{
    const uint32_t packet_delta = this_seq - last_seq;
    if (packet_delta > 1) {
        if (last_seq != 0) {  // reboot might see high initial seqno
            lost_count += packet_delta - 1;
        }
    }
    last_seq = this_seq;
}

int AP_HobbyWing_DataLink::readHeader()
{
    int headerPosition = -1;

    uint32_t available = uart.available();
    if(available < HEADER_SIZE)
    {
        return 0;
    }

    // if first chunk not filled, read first chunk
    if(!first_chunk_filled)
    {
        auto bytes_read = uart.read(header_first_chunk, HEADER_SIZE);
        if(bytes_read == HEADER_SIZE)
        {
            first_chunk_filled = true;
        }
        else
        {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "First chunk read not succesfully expected=%d actual=%ld", HEADER_SIZE, (long int)bytes_read);
        }
        return 0;
    }

    // Read second chunk
    auto bytes_read = uart.read(header_second_chunk, HEADER_SIZE);
    if(bytes_read != HEADER_SIZE)
    {
        //read false, read from the beginning
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Second chunk read not succesfully expected=%d actual=%d", HEADER_SIZE, (long int)bytes_read);
        first_chunk_filled = false;
        return 0;
    }

    // Second shunk read successfully, find header
    uint8_t data[HEADER_SIZE * 2];
    for(uint8_t i=0; i<HEADER_SIZE; i++) {
        data[i] = header_first_chunk[i];
        data[HEADER_SIZE + i] = header_second_chunk[i];
    }

    //find header
    for(uint8_t i=0; i<= HEADER_SIZE; i++) {
        if(data[i] == HEADER_START_BYTE_VALUE &&
            data[i+1] == HEADER_PACKAGE_LENGTH_BYTE_VALUE &&
            data[i+2] == HEADER_PACKAGE_PROTOCOL_BYTE_VALUE &&
            data[i+3] == HEADER_PACKAGE_REAL_DATA_BYTE_VALUE)
        {
            //header found
            headerPosition = i;
            break;
        }
    }

    if(headerPosition >= 0) {
        //header found,
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "header found position=%d", headerPosition);
        uint8_t *bytes = (uint8_t*)&packet;
        for(int i = headerPosition; i < HEADER_SIZE * 2; i++)
        {
            //Copy header and rest of the package package
            int byte_position = i - headerPosition;
            bytes[byte_position] = data[i];
        }
        //how many bytes copied to packet
        // set first chunk not filled
        first_chunk_filled = false;
        return HEADER_SIZE * 2 - headerPosition;
    }

    //Header position not found, copy second chunk to the first one
    for(uint8_t i = 0; i< HEADER_SIZE; i++)
    {
        header_first_chunk[i] = header_second_chunk[i];
        header_second_chunk[i] = 0;
    }
    return 0;

}

void AP_HobbyWing_DataLink::resetChunks()
{
    for(uint8_t i = 0; i< HEADER_SIZE; i++) {
        header_first_chunk[i] = 0;
        header_second_chunk[i] = 0;
    }
}

#endif  // AP_HOBBYWING_DATALINK_ENABLED

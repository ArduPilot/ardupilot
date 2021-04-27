#include "AP_BattMonitor_MPPT_PacketDigital.h"

#if HAL_MPPT_PACKETDIGITAL_CAN_ENABLE
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// read - read the voltage and current
void AP_BattMonitor_MPPT_PacketDigital::read()
{
    WITH_SEMAPHORE(_sem_static);

    const uint32_t now_ms = AP_HAL::millis();
    if (device_count > 0 && now_ms - logger_last_ms >= 1000) {
        logger_last_ms = now_ms;
        perform_logging();
    }

    // get output voltage and current but allow for getting input if serial number is negative
    // Using _params._serial_number == 0 will give you the average output of all devices
    if (get_voltage_and_current_and_temp(_params._serial_number, _state.voltage, _state.current_amps, _state.temperature)) {
        _state.temperature_time = now_ms;
        _state.last_time_micros = AP_HAL::micros();
        _state.healthy = true;
        return;
    }

    _state.voltage = 0;
    _state.current_amps = 0;
    _state.healthy = false;
}

void AP_BattMonitor_MPPT_PacketDigital::perform_logging() const
{
#ifndef HAL_BUILD_AP_PERIPH
    if (device_count == 0) {
        // nothing to log
        return;
    }

    AP_Logger *logger = AP_Logger::get_singleton();
    if (!logger || !logger->logging_enabled()) {
        return;
    }

    // log to AP_Logger
    // @LoggerMessage: MPPT
    // @Description: Information about the Maximum Power Point Tracker sensor
    // @Field: TimeUS: Time since system startup
    // @Field: Inst: Driver Instance
    // @Field: SN: Serial number
    // @Field: F: Faults
    // @FieldBits: F: Over-Voltage,Under-Voltage,Over-Current,Over-Temperature
    // @Field: Temp: Temperature
    // @Field: InV: Input Voltage
    // @Field: InC: Input Current
    // @Field: InP: Input Power
    // @Field: OutV: Output Voltage
    // @Field: OutC: Output Current
    // @Field: OutP: Output Power
    
    for (uint8_t i=0; i<device_count; i++) {
        AP::logger().Write("MPPT", "TimeUS,Inst,SN,F,Temp,InV,InC,InP,OutV,OutC,OutP",
                           "s#--OVAWVAW",
                           "F----------",
                           "QBHBbffffff",
                           AP_HAL::micros64(),
                           i,
                           MPPT_devices[i].serialnumber,
                           (uint8_t)MPPT_devices[i].faults,
                           MPPT_devices[i].temperature,
                           (double)MPPT_devices[i].input.voltage,
                           (double)MPPT_devices[i].input.current,
                           (double)MPPT_devices[i].input.power,
                           (double)MPPT_devices[i].output.voltage,
                           (double)MPPT_devices[i].output.current,
                           (double)MPPT_devices[i].output.power);
    }
#endif
}

// parse inbound frames
void AP_BattMonitor_MPPT_PacketDigital::handle_frame(AP_HAL::CANFrame &frame)
{
    const uint16_t serialnumber = frame.id & 0x0000FFFF;
    if (serialnumber == 0) {
        // This is for broadcast and I don't think we should allow this inbound.
        return;
    }

    WITH_SEMAPHORE(_sem_static);

    uint8_t index = get_device_index(serialnumber);
    if (index == UINT8_MAX) {
        // we don't know this device
        if (device_count >= ARRAY_SIZE(MPPT_devices)) {
            // we don't have any room to remember it
            return;
        }
        // add it
        index = device_count;
        device_count++;
        MPPT_devices[index].serialnumber = serialnumber;
        MPPT_devices[index].sequence = frame.data[0];
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "PDCAN: %u New device", serialnumber);

        send_command(PacketType::VOLTAGE_GET, serialnumber);
        send_command(PacketType::ALGORITHM_GET, serialnumber);
        send_command(PacketType::CVT_GET, serialnumber);
    } else if (index < device_count) {
        MPPT_devices[index].sequence = frame.data[0];
    } else {
        // not sure how this can happen, but lets protect the array bounds just in case
        return;
    }

    switch ((PacketType)frame.data[1]) {
    case PacketType::STREAM_FAULT:
        MPPT_devices[index].faults = (FaultFlags)frame.data[2];
        MPPT_devices[index].temperature = frame.data[3];
        break;

    case PacketType::STREAM_INPUT:
        MPPT_devices[index].input.voltage = fixed2float(UINT16_VALUE(frame.data[2], frame.data[3]));
        MPPT_devices[index].input.current = fixed2float(UINT16_VALUE(frame.data[4], frame.data[5]));
        MPPT_devices[index].input.power = fixed2float(UINT16_VALUE(frame.data[6], frame.data[7]), 4); // NOTE: this is using 12:4 fixed point
        break;
    case PacketType::STREAM_OUTPUT:
        MPPT_devices[index].output.voltage = fixed2float(UINT16_VALUE(frame.data[2], frame.data[3]));
        MPPT_devices[index].output.current = fixed2float(UINT16_VALUE(frame.data[4], frame.data[5]));
        MPPT_devices[index].output.power = fixed2float(UINT16_VALUE(frame.data[6], frame.data[7]), 4); // NOTE: this is using 12:4 fixed point
        break;

    case PacketType::FAULT: {
        // This msg is received when a new fault event happens. It contains the bitfield of all faults.
        // We use this event to compare against existing faults to notify the user of just the new fault
        const uint8_t all_current_faults = frame.data[2];
        const uint8_t prev_faults = (uint8_t)MPPT_devices[index].faults;
        const uint8_t new_single_fault = (~prev_faults & all_current_faults);
        if (new_single_fault != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "PDCAN: %u New Fault! %d: %s", serialnumber, (int)new_single_fault, get_fault_code_string((FaultFlags)new_single_fault));
        }
        MPPT_devices[index].faults = (FaultFlags)frame.data[2];
        }
        break;

    case PacketType::ACK:
        break;
    case PacketType::NACK:
        //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PDCAN: %u NACK 0x%2X", serialnumber, (unsigned)packet_sent_prev);
        break;

    case PacketType::ALGORITHM_SET:
        MPPT_devices[index].algorithm = frame.data[2];
        break;

    case PacketType::VOLTAGE_SET:
        MPPT_devices[index].output_voltage_fixed = fixed2float(UINT16_VALUE(frame.data[2], frame.data[3]));
        break;

    case PacketType::CVT_SET:
        MPPT_devices[index].cvt = fixed2float(UINT16_VALUE(frame.data[2], frame.data[3]));
        break;

    case PacketType::ALGORITHM_GET: // this is a request so we never expect it inbound
    case PacketType::VOLTAGE_GET:   // this is a request so we never expect it inbound
    case PacketType::CVT_GET:       // this is a request so we never expect it inbound
    case PacketType::PING:          // this is never received. When sent it generates an ACK
        // nothing to do
        return;
    }
    MPPT_devices[index].timestamp_ms = AP_HAL::millis();
}

void AP_BattMonitor_MPPT_PacketDigital::send_command(const PacketType type, const uint16_t serialnumber, const float data)
{
    AP_HAL::CANFrame txFrame;

    switch (type) {
    case PacketType::STREAM_FAULT:
    case PacketType::STREAM_INPUT:
    case PacketType::STREAM_OUTPUT:
    case PacketType::FAULT:
    case PacketType::ACK:
    case PacketType::NACK:
        // we only receive these
        return;

    case PacketType::ALGORITHM_GET:
    case PacketType::VOLTAGE_GET:
    case PacketType::CVT_GET:
    case PacketType::PING:
        txFrame.dlc = 0;
        break;

    case PacketType::ALGORITHM_SET:
        txFrame.dlc = 1;
        txFrame.data[2] = data;
        break;

    case PacketType::VOLTAGE_SET:
    case PacketType::CVT_SET:
        {
        txFrame.dlc = 2;
        const uint16_t value = float2fixed(data);
        txFrame.data[2] = HIGHBYTE(value);
        txFrame.data[3] = LOWBYTE(value);
        }
        break;
    }

    if (serialnumber == 0) {
        // send to all
        txFrame.id = 0x00240000;
    } else {
        txFrame.id = 0x00210000 | (uint32_t)serialnumber;
    }

    txFrame.id |= AP_HAL::CANFrame::FlagEFF;

    const uint8_t index = get_device_index(serialnumber);
    uint8_t sequence = 0;
    if (index < ARRAY_SIZE(MPPT_devices)) {
        txFrame.data[0] = ++MPPT_devices[index].sequence;
    }

    txFrame.data[0] = sequence;
    txFrame.data[1] = (uint8_t)type;
    txFrame.dlc += 2;

    if (write_frame(txFrame, 50000)) {
        // keep track of what we sent last in case we get an ACK/NACK
        packet_sent_prev = type;
    }
}

// get MPPT_device index by serial number.
// if serial number found in MPP_devices list, return the 0 indexed value
// else return UINT8_MAX
uint8_t AP_BattMonitor_MPPT_PacketDigital::get_device_index(const uint16_t serial_number) const
{
    for (uint8_t i=0; i<device_count; i++) {
        if (MPPT_devices[i].serialnumber == serial_number) {
            return i;
        }
    }
    return UINT8_MAX;
}


// return fault code as string
const char* AP_BattMonitor_MPPT_PacketDigital::get_fault_code_string(const FaultFlags fault) const
{
    switch (fault) {
    case FaultFlags::OVER_VOLTAGE:
        return "Over-Voltage";
    case FaultFlags::UNDER_VOLTAGE:
        return "Under-Voltage";
    case FaultFlags::OVER_CURRENT:
        return "Over-Current";
    case FaultFlags::OVER_TEMPERATURE:
        return "Over-Temperature";
    default:
        return "Unknown";
    }
}

// get the voltage and current and temp of the input or the output MPPT device when returning true
// when returning false, no values were changed.
bool AP_BattMonitor_MPPT_PacketDigital::get_voltage_and_current_and_temp(const int32_t serialnumber, float &voltage, float &current, float &temperature) const
{
    if (device_count == 0) {
        return false;
    }

    if (serialnumber <= 0) {
        // take the average output of all healthy devices
        int8_t count = 0;
        float voltage_out_avg = 0.0f;
        float current_out_avg = 0.0f;
        float temperature_avg = 0.0f;

        for (uint8_t i=0; i<device_count; i++) {
            if (!MPPT_devices[i].is_healthy()) {
                continue;
            }
            count++;
            voltage_out_avg += MPPT_devices[i].output.voltage;
            current_out_avg += MPPT_devices[i].output.current;
            temperature_avg += MPPT_devices[i].temperature;
        }
        if (count > 0) {
            voltage = voltage_out_avg / count;
            current = current_out_avg / count;
            temperature = (float)temperature_avg / count;
            // average OUTPUTs of all healthy devices
            return true;
        }
        // no healthy devices found
        return false;
    }


    // average
    const uint8_t index = get_device_index(serialnumber);
    if (!is_healthy_by_index(index)) {
        return false;
    }

    // we only report output energy
    voltage = MPPT_devices[index].output.voltage;
    current = MPPT_devices[index].output.current;
    temperature = MPPT_devices[index].temperature;
    return true;
}


#endif // HAL_MPPT_PACKETDIGITAL_CAN_ENABLE

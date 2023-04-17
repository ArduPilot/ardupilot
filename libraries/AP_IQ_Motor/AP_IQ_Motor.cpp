#include "AP_IQ_Motor.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

// #include <AP_Logger/AP_Logger.h>

#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_IQ_Motor::var_info[] = {

    // @Param: CTRL_LEN
    // @DisplayName: Number of Motors
    // @Description: This sets how many motors will be sent throttle commands starting from 0 and being contiguous
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("LEN", 1, AP_IQ_Motor, _broadcast_length, 0),

    // @Param: TELEM_MASK
    // @DisplayName: Motor Telemetry Bitmask
    // @Description: This sets which motors are sending data back to the flight controller
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("TELEM", 2, AP_IQ_Motor, _telemetry_bitmask, 0),

    // @Param: MOT_DIR
    // @DisplayName: Motor Direction Bitmask
    // @Description: This sets the direction of each motor. 0 is CCW, 1 is CW.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("DIR", 3, AP_IQ_Motor, _motor_dir_bitmask, 0),

    AP_GROUPEND
};

AP_IQ_Motor::AP_IQ_Motor(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IQ_Motor::init(void)
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (!serial_manager)
    {
        return;
    }
    iq_uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_IQ, 0);
    // check if the uart exists
    if (iq_uart)
    {
        const int8_t length = _broadcast_length.get();
        _total_channels = NUM_SERVO_CHANNELS > length ? length : NUM_SERVO_CHANNELS;
        // // loop through all servos
        // for (uint8_t servo = 0; servo < _total_channels; ++servo)
        // {
        //     add_client(&_motors[servo]);
        // }

        // find the last motor for telemetry
        const uint8_t bitmask = _telemetry_bitmask.get();
        bool found_first = false;
        for (uint8_t motor = 0; motor < NUM_SERVO_CHANNELS; ++motor)
        {
            if (bitmask & 1 << motor)
            {
                if (found_first)
                {
                    _last_telem_motor = motor;
                } else{
                    _first_telem_motor = motor;
                    found_first = true;
                }
            }
        }
    }
}

uint8_t AP_IQ_Motor::add_client(ClientAbstract *new_client)
{
    if (_client_size < _client_limit) // TODO check if this client type already exists and don't add if it does. Instead return the pointer?
    {
        _clients[_client_size] = new_client;
        _client_size++;
        return 1;
    }

    return 0;
}

uint8_t AP_IQ_Motor::find_client(ClientAbstract **set_client, uint8_t client_type, uint8_t obj_idn)
{
    for (int client = 0; client < _client_size; ++client)
    {
        if (_clients[client]->obj_idn_ == obj_idn && _clients[client]->type_idn_ == client_type)
        {
            *set_client = _clients[client];
            return 1;
        }
    }
    return 0;
}

void AP_IQ_Motor::update(void)
{
    if (!_initialized)
    {
        _initialized = true; // check if init worked
        init();
    }

    if (iq_uart == nullptr)
    {
        return;
    }

    // TODO: generate general broadcast output message to esc prop from channels array
    // TODO: send telemetry request for current telemetry motor
    read();
    update_motor_outputs();
    if (_telemetry_bitmask.get() != 0)
    {
        update_telemetry();
    }
    write();
    
}

void AP_IQ_Motor::update_motor_outputs()
{
    // const uint8_t bitmask = _motor_dir_bitmask.get();
    if (hal.util->get_soft_armed())
    {
        uint16_t raw_control_values[_total_channels];
        for (unsigned ii = 0; ii < _total_channels; ++ii) {
            SRV_Channel *c = SRV_Channels::srv_channel(ii);
            if (c == nullptr) {
                continue;
            }

            uint16_t output = uint16_t(constrain_float((c->get_output_norm() + 1.0) * 0.5, 0, 1) * 65535);
            raw_control_values[ii] = output;
        }
        _motor_interface.BroadcastPackedControlMessage(_com, raw_control_values, _total_channels, _telem_request_id);
        _telem_request_id = 255;
    } else
    {
        uint8_t tx_msg[2]; // must fit outgoing message
        tx_msg[0] = kSubCtrlCoast;
        tx_msg[1] = (kBroadcastID<<2) | kSet; // high six | low two
        _com.SendPacket(kTypePropellerMotorControl, tx_msg, 2);
    }
}

#if HAL_WITH_ESC_TELEM 
void AP_IQ_Motor::update_telemetry() 
{
    bool got_reply = false;
    if (_motor_interface.telemetry_.IsFresh())
    {
        IFCITelemetryData motor_telem = _motor_interface.telemetry_.get_reply();
        float velocity = abs(motor_telem.speed * M_1_PI * 0.5 * 60);
        update_rpm(_motor_interface.get_last_telemetry_receeived_id(), velocity, 0.0);
        TelemetryData t {};
        t.temperature_cdeg = motor_telem.mcu_temp;
        t.motor_temp_cdeg = motor_telem.coil_temp;
        t.voltage = motor_telem.voltage * 0.01;
        t.current = motor_telem.current * 0.01;
        t.consumption_mah = motor_telem.consumption;
        t.usage_s = motor_telem.uptime;
        update_telem_data(
            _motor_interface.get_last_telemetry_receeived_id(),
            t,
            TelemetryType::TEMPERATURE|
            TelemetryType::MOTOR_TEMPERATURE|
            TelemetryType::VOLTAGE|
            TelemetryType::CURRENT|
            TelemetryType::CONSUMPTION|
            TelemetryType::USAGE
        );
        got_reply = true;
    }
    // if (_motors[_telem_motor_id].obs_velocity_.IsFresh())
    // {
    //     float velocity = abs(_motors[_telem_motor_id].obs_velocity_.get_reply());
    //     update_rpm(_telem_motor_id, velocity * M_1_PI * 0.5 * 60, 0.0);
    //     TelemetryData t {};
    //     t.temperature_cdeg = _motors[_telem_motor_id].uc_temp_.get_reply() * 100;
    //     t.voltage = _motors[_telem_motor_id].volts_.get_reply();
    //     t.current = _motors[_telem_motor_id].amps_.get_reply();
    //     t.consumption_mah = _motors[_telem_motor_id].joules_.get_reply() * 0.00027777777; // watt hours TODO fix to milliamp hours
    //     t.usage_s = _motors[_telem_motor_id].time_.get_reply();
    //     t.motor_temp_cdeg = _motors[_telem_motor_id].temp_.get_reply() * 100;
    //     update_telem_data(
    //         _telem_motor_id,
    //         t,
    //         TelemetryType::TEMPERATURE|
    //         TelemetryType::MOTOR_TEMPERATURE|
    //         TelemetryType::VOLTAGE|
    //         TelemetryType::CURRENT|
    //         TelemetryType::CONSUMPTION|
    //         TelemetryType::USAGE
    //     );
    //     got_reply = true;
    // }
    bool timeout = AP_HAL::millis() - _last_request_time > _telem_timeout;
    if (got_reply || timeout)
    {
        find_next_telem_motor();
        _telem_request_id = _telem_motor_id;
        // _motors[_telem_motor_id].uc_temp_.get(_com);
        // _motors[_telem_motor_id].volts_.get(_com);
        // _motors[_telem_motor_id].amps_.get(_com);
        // _motors[_telem_motor_id].joules_.get(_com);
        // _motors[_telem_motor_id].time_.get(_com);
        // _motors[_telem_motor_id].temp_.get(_com);
        // _motors[_telem_motor_id].obs_velocity_.get(_com);
        _last_request_time = AP_HAL::millis();
    }
}
#endif

void AP_IQ_Motor::find_next_telem_motor()
{
    const uint8_t bitmask = _telemetry_bitmask.get();

    while(bitmask != 0)
    {
        ++_telem_motor_id;
        if (_telem_motor_id > _last_telem_motor)
        {
            _telem_motor_id = _first_telem_motor;
        }
        if (_telemetry_bitmask & 1 << _telem_motor_id)
        {
            return;
        }
    }

}

void AP_IQ_Motor::read()
{
    _ser_length = iq_uart->available();
    if (_ser_length > 0)
    {
        _ser_length = iq_uart->read(_rx_buf, _ser_length);
        _com.SetRxBytes(_rx_buf, _ser_length);
        uint8_t *packet_buf_point = _rx_buf;
        while(_com.PeekPacket(&packet_buf_point, &_ser_length) == 1)
        {
            _motor_interface.ReadTelemetry(packet_buf_point, _ser_length);
            for (int client = 0; client < _client_size; ++client)
            {
                _clients[client]->ReadMsg(packet_buf_point, _ser_length);
            }
            _com.DropPacket();
        }
        
    }
    
}

void AP_IQ_Motor::write()
{
    _writing = true;
    while(_com.GetTxBytes(_tx_buf, _ser_length))
    {
        iq_uart->write(_tx_buf, _ser_length);
    }
    _writing = false;
}
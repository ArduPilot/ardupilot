#include "AP_IQMotor.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

// #include <AP_Logger/AP_Logger.h>

#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_IQMotor::var_info[] = {

    // @Param: CTRL_LEN
    // @DisplayName: Number of Motors
    // @Description: This sets how many motors will be sent throttle commands starting from 0 and being contiguous
    // @Range: 0 16
    // @User: Advanced
    AP_GROUPINFO("LEN", 1, AP_IQMotor, broadcast_length, 0),

    // @Param: TELEM_MASK
    // @DisplayName: Motor Telemetry Bitmask
    // @Description: This sets which motors are sending data back to the flight controller
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("TELEM", 2, AP_IQMotor, telemetry_bitmask, 0),

    // @Param: MOT_DIR
    // @DisplayName: Motor Direction Bitmask
    // @Description: This sets the direction of each motor. 0 is CCW, 1 is CW.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("DIR", 3, AP_IQMotor, motor_dir_bitmask, 0),

    AP_GROUPEND
};

AP_IQMotor::AP_IQMotor(void)
{
    // AP_Param::setup_object_defaults(this, var_info);
}

void AP_IQMotor::init(void)
{
    AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
    if (!serial_manager)
    {
        return;
    }
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "found serial manager");
    iq_uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_IQ, 0);
    // check if the uart exists
    if (iq_uart)
    {
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "found iq_uart");
        const int8_t length = broadcast_length.get();
        total_channels = NUM_SERVO_CHANNELS > length ? length : NUM_SERVO_CHANNELS;
        // loop through all servos
        for (uint8_t servo = 0; servo < total_channels; ++servo)
        {
            add_client(&motors[servo]);
        }

        // find the last motor for telemetry
        const uint8_t bitmask = telemetry_bitmask.get();
        bool found_first = false;
        for (uint8_t motor = 0; motor < NUM_SERVO_CHANNELS; ++motor)
        {
            if (bitmask & 1 << motor)
            {
                if (found_first)
                {
                    last_telem_motor = motor;
                } else{
                    first_telem_motor = motor;
                    found_first = true;
                }
            }
        }
    }
}

uint8_t AP_IQMotor::add_client(ClientAbstract *new_client)
{
    if (client_size < client_limit) // TODO check if this client type already exists and don't add if it does. Instead return the pointer?
    {
        clients[client_size] = new_client;
        client_size++;
        return 1;
    }

    return 0;
}

uint8_t AP_IQMotor::find_client(ClientAbstract **set_client, uint8_t client_type, uint8_t obj_idn)
{
    for (int client = 0; client < client_size; ++client)
    {
        if (clients[client]->obj_idn_ == obj_idn && clients[client]->type_idn_ == client_type)
        {
            *set_client = clients[client];
            return 1;
        }
    }
    return 0;
}

void AP_IQMotor::update(void)
{
    if (!initialized)
    {
        initialized = true; // check if init worked
        init();
    }

    if (iq_uart == nullptr)
    {
        return;
    }

    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "updating iq_uart");

    // TODO: generate general broadcast output message to esc prop from channels array
    // TODO: send telemetry request for current telemetry motor
    read();
    update_motor_outputs();
    if (telemetry_bitmask.get() != 0)
    {
        update_telemetry();
    }
    write();
    
}

void AP_IQMotor::update_motor_outputs()
{
    const uint8_t bitmask = motor_dir_bitmask.get();
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Total channels: %u", total_channels);
    for (unsigned ii = 0; ii < total_channels; ++ii) {
        SRV_Channel *c = SRV_Channels::srv_channel(ii);
        if (c == nullptr) {
            continue;
        }
        float dir = 1.0;
        if (bitmask & 1 << ii)
        {
            dir = -1.0;
        }
        float output = constrain_float((c->get_output_norm() + 1.0) * 0.5, 0, 1);
        output *= dir;
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Motor Outs %u: %f", ii, output);
        motors[ii].drive_spin_pwm_.set(com, output);
    }
}

void AP_IQMotor::update_telemetry() 
{
    bool got_reply = false;
    if (motors[telem_motor_id].obs_velocity_.IsFresh())
    {
        // float supply_volts = motors[telem_motor_id].obs_supply_volts_.get_reply();
        // float velocity = motors[telem_motor_id].obs_velocity_.get_reply();
        // AP::logger().Write_ESC(telem_motor_id,
        //               AP_HAL::micros64(), // Time stamp
        //               int32_t(velocity), // RPM
        //               uint16_t(supply_volts),    // voltage
        //               0,    // current
        //               0,    // esc temp
        //               0,    // amp hours?
        //               0,    // motor temp
        //               0);   // error rate
        got_reply = true;
    }
    bool timeout = AP_HAL::millis() - last_request_time > TELEM_TIMEOUT;
    if (got_reply || timeout)
    {
        // if (timeout)
        // {
        //     GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "timed out on channel %u", telem_motor_id);
        // }
        
        find_next_telem_motor();
        // out_flag = true;
        motors[telem_motor_id].obs_supply_volts_.get(com);
        motors[telem_motor_id].obs_velocity_.get(com);
        last_request_time = AP_HAL::millis();
    }
}

void AP_IQMotor::find_next_telem_motor()
{
    const uint8_t bitmask = telemetry_bitmask.get();

    while(bitmask != 0)
    {
        ++telem_motor_id;
        if (telem_motor_id > last_telem_motor)
        {
            telem_motor_id = first_telem_motor;
        }
        if (telemetry_bitmask & 1 << telem_motor_id)
        {
            return;
        }
    }

}

void AP_IQMotor::read()
{
    ser_length = iq_uart->available();
    if (ser_length > 0)
    {
        ser_length = iq_uart->read(rx_buf, ser_length);
        // if (out_flag)
        // {
        //     GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Reading");
        //     for (int ii = 0; ii < ser_length; ++ii)
        //     {
        //         GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "%u", rx_buf[ii]);
        //     }
        //     out_flag = false;
        // }
        com.SetRxBytes(rx_buf, ser_length);
        uint8_t *packet_buf_point = rx_buf; // what?
        while(com.PeekPacket(&packet_buf_point, &ser_length) == 1)
        {
            // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Peeked");
            // loop through the added clients and read messages
            for (int client = 0; client < client_size; ++client)
            {
                clients[client]->ReadMsg(packet_buf_point, ser_length);
            }
            com.DropPacket();
        }
        
    }
    
}

void AP_IQMotor::write()
{
    // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Writing");
    writing = true;
    while(com.GetTxBytes(tx_buf, ser_length))
    {
        iq_uart->write(tx_buf, ser_length);
    }
    writing = false;
}
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <AP_Mount/BP_STorM32.h>

//OW
//#include "../ArduCopter/Copter.h"
//extern Copter copter;


//******************************************************
// BP_STorM32 class functions
//******************************************************


/// Constructor
BP_STorM32::BP_STorM32() :
    _serial_is_initialised(false),
    _storm32link_seq(0)
{
    _serial_in.state = SERIALSTATE_IDLE;
}


//------------------------------------------------------
// send stuff
//------------------------------------------------------

// 26 bytes = 2257us @ 115200bps
void BP_STorM32::send_attitude(const AP_AHRS_TYPE &ahrs)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tSTorM32Link) +2) {
        return;
    }

    Quaternion quat;
    quat.from_rotation_matrix( ahrs.get_rotation_body_to_ned() );

    uint8_t status = 0;
    //it seems these two states are exclusive, see e.g. AP_Module::call_hook_AHRS_update()
    if (!ahrs.initialised()) status |= 0x01; //is initialising
    if (!ahrs.healthy())     status |= 0x02; //is unhealthy
//OW
//    if (!copter.letmeget_ekf_filter_status()) status |= 0x04;
//    if (copter.letmeget_pream_check()) status |= 0x40;
//    if (copter.letmeget_motors_armed()) status |= 0x80;

    tSTorM32Link t;
    t.stx = 0xF9;
    t.len = 0x15;
    t.cmd = 0xD9;
    t.seq = _storm32link_seq; _storm32link_seq++;
    t.status = status;
    t.spare = 0;
    t.yawratecmd = 0;
    t.q1 = quat.q1;
    t.q2 = quat.q2;
    t.q3 = quat.q3;
    t.q4 = quat.q4;
    t.crc = crc_calculate(&(t.len), sizeof(tSTorM32Link)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tSTorM32Link), PRIORITY_HIGHEST );
}


// 19 bytes = 1650us @ 115200bps
void BP_STorM32::send_cmd_setangles(float pitch_deg, float roll_deg, float yaw_deg, uint16_t flags)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdSetAngles) +2) {
        return;
    }

    tCmdSetAngles t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x0E;
    t.cmd = 0x11;
    t.pitch = pitch_deg;
    t.roll = roll_deg;
    t.yaw = yaw_deg;
    t.flags = flags;
    t.type = 0;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdSetAngles)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdSetAngles) );
}


// 11 bytes = 955us @ 115200bps
void BP_STorM32::send_cmd_setpitchrollyaw(uint16_t pitch, uint16_t roll, uint16_t yaw)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdSetPitchRollYaw) +2) {
        return;
    }

    tCmdSetPitchRollYaw t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x06;
    t.cmd = 0x12;
    t.pitch = pitch;
    t.roll = roll;
    t.yaw = yaw;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdSetPitchRollYaw)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdSetPitchRollYaw) );
}


// 11 bytes = 955us @ 115200bps
void BP_STorM32::send_cmd_recentercamera(void)
{
    send_cmd_setpitchrollyaw(0, 0, 0);
}


// 11 bytes = 955us @ 115200bps
void BP_STorM32::send_cmd_docamera(uint16_t camera_cmd)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdDoCamera) +2) {
        return;
    }

    tCmdDoCamera t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x06;
    t.cmd = 0x0F;
    t.dummy1 = 0;
    t.camera_cmd = camera_cmd;
    t.dummy2 = 0;
    t.dummy3 = 0;
    t.dummy4 = 0;
    t.dummy5 = 0;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdDoCamera)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdDoCamera) );
}


// 28 bytes = 2431us @ 115200bps
void BP_STorM32::send_cmd_setinputs(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdSetInputs) +2) {
        return;
    }

    uint8_t status = 0;

    tCmdSetInputs t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x17;
    t.cmd = 0x16;
    t.channel0 = _rcin_read(0);
    t.channel1 = _rcin_read(1);
    t.channel2 = _rcin_read(2);
    t.channel3 = _rcin_read(3);
    t.channel4 = _rcin_read(4);
    t.channel5 = _rcin_read(5);
    t.channel6 = _rcin_read(6);
    t.channel7 = _rcin_read(7);
    t.channel8 = _rcin_read(8);
    t.channel9 = _rcin_read(9);
    t.channel10 = _rcin_read(10);
    t.channel11 = _rcin_read(11);
    t.channel12 = _rcin_read(12);
    t.channel13 = _rcin_read(13);
    t.channel14 = _rcin_read(14);
    t.channel15 = _rcin_read(15);
    t.status = status;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdSetInputs)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdSetInputs) );
}


// 19 bytes = 1650us @ 115200bps
void BP_STorM32::send_cmd_sethomelocation(const AP_AHRS_TYPE &ahrs)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdSetHomeTargetLocation) +2) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    if (ahrs.get_position(location)) {
        status = 0x0001; //= LOCATION_VALID
    }

    tCmdSetHomeTargetLocation t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x0E;
    t.cmd = 0x17;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdSetHomeTargetLocation)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdSetHomeTargetLocation) );
}


// 19 bytes = 1650us @ 115200bps
void BP_STorM32::send_cmd_settargetlocation(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdSetHomeTargetLocation) +2) {
        return;
    }

    uint16_t status = 0; //= LOCATION_INVALID
    struct Location location = {};

    tCmdSetHomeTargetLocation t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x0E;
    t.cmd = 0x18;
    t.latitude = location.lat;
    t.longitude = location.lng;
    t.altitude = location.alt;
    t.status = status;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdSetHomeTargetLocation)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdSetHomeTargetLocation) );
}


// 7 bytes = 608us @ 115200bps
void BP_STorM32::send_cmd_getdatafields(uint16_t flags)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdGetDataFields) +2) {
        return;
    }

    tCmdGetDataFields t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x02;
    t.cmd = 0x06;
    t.flags = flags;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdGetDataFields)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdGetDataFields) );
}


// 5 bytes = 434us @ 115200bps
void BP_STorM32::send_cmd_getversionstr(void)
{
    if (!_serial_is_initialised) {
        return;
    }

    if (_serial_txspace() < sizeof(tCmdGetVersionStr) +2) {
        return;
    }

    tCmdGetVersionStr t;
    t.stx = 0xF9; //0xFA; //0xF9 to suppress response
    t.len = 0x00;
    t.cmd = 0x02;
    t.crc = crc_calculate(&(t.len), sizeof(tCmdGetVersionStr)-3);

    _serial_write( (uint8_t*)(&t), sizeof(tCmdGetVersionStr) );
}


//------------------------------------------------------
// receive stuff, only relevant for a real serial, not CAN
//------------------------------------------------------

void BP_STorM32::receive_reset(void)
{
    _serial_in.state = SERIALSTATE_IDLE;
}


void BP_STorM32::receive_reset_wflush(void)
{
    while (_serial_available() > 0) _serial_read();
    _serial_in.state = SERIALSTATE_IDLE;
}


//reads in one char and processes it
// there should/could be some timeout handling
// there should/could be some error handling
// but flush_rx() will take care of both of that
void BP_STorM32::do_receive_singlechar(void)
{
    uint8_t c;

    if (_serial_available() <= 0) return;  //this should never happen, but play it safe

    switch (_serial_in.state) {
        case SERIALSTATE_IDLE:
            c = _serial_read();

            if (c == 0xFB) { //the outcoming RCCMD start sign was received
                _serial_in.stx = c;
                _serial_in.state = SERIALSTATE_RECEIVE_PAYLOAD_LEN;
            }
            break;

        case SERIALSTATE_RECEIVE_PAYLOAD_LEN:
            c = _serial_read();

            _serial_in.len = c;
            _serial_in.state = SERIALSTATE_RECEIVE_CMD;
            break;

        case SERIALSTATE_RECEIVE_CMD:
            c = _serial_read();

            _serial_in.cmd = c;
            _serial_in.payload_cnt = 0;
            _serial_in.state = SERIALSTATE_RECEIVE_PAYLOAD;
            break;

        case SERIALSTATE_RECEIVE_PAYLOAD:
            c = _serial_read();

            if (_serial_in.payload_cnt >= SERIAL_RECEIVE_BUFFER_SIZE) {
                _serial_in.state = SERIALSTATE_IDLE; //error, get out of here
                return;
            }

            _serial_in.buf[_serial_in.payload_cnt++] = c;

            if (_serial_in.payload_cnt >= _serial_in.len + 2) { //do expect always a crc
              uint16_t crc = 0; //XX ignore crc for the moment
              if (crc == 0) {
                  _serial_in.state = SERIALSTATE_MESSAGERECEIVED;
              }
            }
            break;

        case SERIALSTATE_MESSAGERECEIVED:case SERIALSTATE_MESSAGERECEIVEDANDDIGESTED:
            c = _serial_read();
            break;
    }
}


//reads in as many chars as there are there
void BP_STorM32::do_receive(void)
{
    while (_serial_available() > 0) { do_receive_singlechar(); }

// serial state is reset by a flush_rx() and receive_reset()
}


bool BP_STorM32::message_received(void)
{
    if (_serial_in.state == SERIALSTATE_MESSAGERECEIVED) {
        _serial_in.state = SERIALSTATE_MESSAGERECEIVEDANDDIGESTED;
        return true;
    }
    return false;
}


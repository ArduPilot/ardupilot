/* #################################################################################################################
 * LightTelemetry protocol (LTM)
 *
 * Ghettostation one way telemetry protocol for really low bitrates (2400 bauds). 
 *			   
 * Protocol details: 3 different frames, little endian.
 *   G Frame (GPS position) (2 Hz): 18BYTES
 *    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0
 *     $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
 *   A Frame (Attitude) (5 Hz): 10BYTES
 *     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0
 *      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
 *   S Frame (Sensors) (2 Hz): 11BYTES
 *     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0
 *      $     T   S   VBAT(mV)  Current(mA)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
 * ################################################################################################################# */

#include "AP_LTM_Telem.h"

#if AP_LTM_TELEM_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_RSSI/AP_RSSI.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

// init - perform required initialisation
void AP_LTM_Telem::init()
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    
    // check for LTM_Port
    if ((_port = serial_manager.find_serial(
            AP_SerialManager::SerialProtocol_LTM_Telem, 0))) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        // initialise UART
        _port->begin(0, AP_SERIALMANAGER_LTM_BUFSIZE_RX,
        AP_SERIALMANAGER_LTM_BUFSIZE_TX);
        hal.scheduler->register_io_process(
                FUNCTOR_BIND_MEMBER(&AP_LTM_Telem::tick, void));
    }
}

void AP_LTM_Telem::send_LTM(uint8_t lt_packet[], uint8_t lt_packet_size)
{
    // check space before write
    if (_port->txspace() < lt_packet_size) {
        return;
    }
    // calculate checksum
    uint8_t lt_crc = 0x00;
    for (uint8_t i = 3; i < lt_packet_size - 1; i++) {
        lt_crc ^= lt_packet[i];
    }
    lt_packet[lt_packet_size - 1] = lt_crc;
    _port->write(lt_packet, lt_packet_size);
}

// GPS frame
void AP_LTM_Telem::send_Gframe(void)
{
    const AP_GPS &gps = AP::gps();
    const uint8_t sats_visible = gps.num_sats();
    const uint8_t fix_type = (uint8_t)gps.status();
    int32_t lat = 0;                        // latitude
    int32_t lon = 0;                        // longtitude
    uint8_t gndspeed = 0;                   // gps ground speed (m/s)
    int32_t alt = 0;
#if AP_AHRS_ENABLED
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        float alt_ahrs;
        ahrs.get_relative_position_D_home(alt_ahrs);
        alt = (int32_t) roundf(-alt_ahrs * 100.0); // altitude (cm)
        Location loc;
        if (ahrs.get_location(loc)) {
            lat = loc.lat;
            lon = loc.lng;
            gndspeed = (uint8_t) roundf(gps.ground_speed());
        }
    }
#endif

    uint8_t lt_buff[LTM_GFRAME_SIZE];
    // protocol: START(2 bytes)FRAMEID(1byte)LAT(cm,4 bytes)LON(cm,4bytes)SPEED(m/s,1bytes)ALT(cm,4bytes)SATS(6bits)FIX(2bits)CRC(xor,1byte)
    // START
    lt_buff[0] = 0x24; //$
    lt_buff[1] = 0x54; //T
    // FRAMEID
    lt_buff[2] = 0x47; // G ( gps frame at 5hz )
    // PAYLOAD
    lt_buff[3] = (lat >> 8 * 0) & 0xFF;
    lt_buff[4] = (lat >> 8 * 1) & 0xFF;
    lt_buff[5] = (lat >> 8 * 2) & 0xFF;
    lt_buff[6] = (lat >> 8 * 3) & 0xFF;
    lt_buff[7] = (lon >> 8 * 0) & 0xFF;
    lt_buff[8] = (lon >> 8 * 1) & 0xFF;
    lt_buff[9] = (lon >> 8 * 2) & 0xFF;
    lt_buff[10] = (lon >> 8 * 3) & 0xFF;
    lt_buff[11] = (gndspeed >> 8 * 0) & 0xFF;
    lt_buff[12] = (alt >> 8 * 0) & 0xFF;
    lt_buff[13] = (alt >> 8 * 1) & 0xFF;
    lt_buff[14] = (alt >> 8 * 2) & 0xFF;
    lt_buff[15] = (alt >> 8 * 3) & 0xFF;
    lt_buff[16] = ((sats_visible << 2) & 0xFF)
            | ((fix_type > 3 ? 3 : fix_type) & 0b00000011); // last 6 bits: sats number, first 2:fix type (0, 1, 2, 3)
    send_LTM(lt_buff, LTM_GFRAME_SIZE);
    _ltm_scheduler++;
}

// Sensors frame
void AP_LTM_Telem::send_Sframe(void)
{
    const AP_BattMonitor &battery = AP::battery();
    const uint16_t volt = (uint16_t) roundf(battery.voltage() * 1000.0f);              // battery voltage (expects value in mV)
    float current;
    if (!battery.current_amps(current)) {
        current = 0;
    }
    // note: max. current value we can send is 65.536 A
    const uint16_t amp = (uint16_t) roundf(current * 100.0f);                          // current sensor (expects value in hundredth of A)

    // airspeed in m/s if available and enabled - even if not used - otherwise send 0
    uint8_t airspeed = 0; // airspeed sensor (m/s)
#if AP_AIRSPEED_ENABLED
    const AP_Airspeed *aspeed = AP::airspeed();
    if (aspeed && aspeed->enabled()) {
        airspeed = (uint8_t) roundf(aspeed->get_airspeed());
    }
#endif

    const uint8_t flightmode = AP_Notify::flags.flight_mode; // flight mode

    uint8_t rssi = 0; // radio RSSI (%a)
    AP_RSSI *ap_rssi = AP_RSSI::get_singleton();
    if (ap_rssi) {
        rssi = ap_rssi->read_receiver_rssi_uint8();
    }

    const uint8_t armstat = AP_Notify::flags.armed;                                     // 0: disarmed, 1: armed
    const uint8_t failsafe = AP_Notify::flags.failsafe_radio;                           // 0: normal,   1: failsafe

    uint8_t lt_buff[LTM_SFRAME_SIZE];
    // START
    lt_buff[0] = 0x24; //$
    lt_buff[1] = 0x54; //T
    // FRAMEID
    lt_buff[2] = 0x53; //S 
    // PAYLOAD
    lt_buff[3] = (volt >> 8 * 0) & 0xFF; // VBAT converted to mV
    lt_buff[4] = (volt >> 8 * 1) & 0xFF;
    lt_buff[5] = (amp >> 8 * 0) & 0xFF; // actual current instead of consumed mAh in regular LTM
    lt_buff[6] = (amp >> 8 * 1) & 0xFF;
    lt_buff[7] = (rssi >> 8 * 0) & 0xFF;
    lt_buff[8] = (airspeed >> 8 * 0) & 0xFF;
    lt_buff[9] = ((flightmode << 2) & 0xFF)
            | ((failsafe << 1) & 0b00000010) | (armstat & 0b00000001); // last 6 bits: flight mode, 2nd bit: failsafe, 1st bit: arm status.
    send_LTM(lt_buff, LTM_SFRAME_SIZE);
    _ltm_scheduler++;
}

// Attitude frame
void AP_LTM_Telem::send_Aframe(void)
{
    int16_t pitch;
    int16_t roll;
    int16_t heading;
#if AP_AHRS_ENABLED
    {
        AP_AHRS &ahrs = AP::ahrs();
        WITH_SEMAPHORE(ahrs.get_semaphore());
        pitch = roundf(ahrs.pitch_sensor * 0.01); // attitude pitch in degrees
        roll = roundf(ahrs.roll_sensor * 0.01);   // attitude roll in degrees
        heading = roundf(ahrs.yaw_sensor * 0.01); // heading in degrees
    }
#else
    pitch = 0;
    roll = 0;
    heading = 0;
#endif

    uint8_t lt_buff[LTM_AFRAME_SIZE];
    // A Frame: $T(2 bytes)A(1byte)PITCH(2 bytes)ROLL(2bytes)HEADING(2bytes)CRC(xor,1byte)
    // START
    lt_buff[0] = 0x24; //$
    lt_buff[1] = 0x54; //T
    // FRAMEID
    lt_buff[2] = 0x41; //A 
    // PAYLOAD
    lt_buff[3] = (pitch >> 8 * 0) & 0xFF;
    lt_buff[4] = (pitch >> 8 * 1) & 0xFF;
    lt_buff[5] = (roll >> 8 * 0) & 0xFF;
    lt_buff[6] = (roll >> 8 * 1) & 0xFF;
    lt_buff[7] = (heading >> 8 * 0) & 0xFF;
    lt_buff[8] = (heading >> 8 * 1) & 0xFF;
    send_LTM(lt_buff, LTM_AFRAME_SIZE);
    _ltm_scheduler++;
}

// send LTM
void AP_LTM_Telem::generate_LTM(void)
{
    if (_ltm_scheduler & 1) {   // is odd
        send_Aframe();
    } else {                    // is even
        if (_ltm_scheduler % 4 == 0) {
            send_Sframe();
        } else {
            send_Gframe();
        }
    }
    if (_ltm_scheduler > 9) {
        _ltm_scheduler = 1;
    }
}

void AP_LTM_Telem::tick(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - _last_frame_ms >= 100) {
        _last_frame_ms = now;
        generate_LTM();
    }
}

#endif  // AP_LTM_TELEM_ENABLED

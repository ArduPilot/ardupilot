#pragma once

#include <AP_HAL/utility/BetterStream.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_SerialDevice_UART;

class AP_SerialDevice : public AP_HAL::BetterStream
{
public:

    enum class Protocol {
        None = -1,
        Console = 0, // unused
        MAVLink = 1,
        MAVLink2 = 2,                 // do not use - use MAVLink and provide instance of 1
        FrSky_D = 3,                  // FrSky D protocol (D-receivers)
        FrSky_SPort = 4,              // FrSky SPort protocol (X-receivers)
        GPS = 5,
        GPS2 = 6,                     // do not use - use GPS and provide instance of 1
        AlexMos = 7,
        SToRM32 = 8,
        Rangefinder = 9,
        FrSky_SPort_Passthrough = 10, // FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
        Lidar360 = 11,                // Lightware SF40C, TeraRanger Tower or RPLidarA2
        Aerotenna_USD1      = 12, // Ulanding support - deprecated, users should use Rangefinder
        Beacon = 13,
        Volz = 14,                    // Volz servo protocol
        Sbus1 = 15,
        ESCTelemetry = 16,
        Devo_Telem = 17,
        OpticalFlow = 18,
        Robotis = 19,
        NMEAOutput = 20,
        WindVane = 21,
        SLCAN = 22,
        RCIN = 23,
        EFI_MS = 24,                   // MegaSquirt EFI serial protocol
        LTM_Telem = 25,
        RunCam = 26,
        Hott = 27,
        Scripting = 28,
        CRSF = 29,
        Generator = 30,
        Winch = 31,
        MSP = 32,
        DJI_FPV = 33,
        AirSpeed = 34,
        ADSB = 35,
        AHRS = 36,
        SmartAudio = 37,
        FETtecOneWire = 38,
        Torqeedo = 39,
        AIS = 40,
        CoDevESC = 41,
        MSP_DisplayPort = 42,
        MAVLinkHL = 43,
        NumProtocols                    // must be the last value
    };

    AP_SerialDevice(Protocol _prot, uint8_t _instance) :
        _using_protocol{_prot},
        instance{_instance}
        { }

    // write to a locked port. If port is locked and key is not
    // correct then 0 is returned and write is discarded
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key);

    // read from a locked port. If port is locked and key is not
    // correct then 0 is returned
    int16_t read_locked(uint32_t key);

    // lock a port for exclusive use. Use a key of 0 to unlock
    bool lock_port(uint32_t write_key, uint32_t read_key);

    // check data available on a locked port. If port is locked and
    // key is not correct then 0 is returned
    uint32_t available_locked(uint32_t key);

    virtual AP_SerialDevice_UART *get_serialdevice_uart() { return nullptr; }

    virtual void begin() = 0;
    virtual void end() = 0;
    void flush();

    virtual int16_t read() override = 0;

    ssize_t read(uint8_t *buffer, uint16_t count) override = 0;

    virtual bool tx_pending() const = 0;

    virtual uint64_t receive_time_constraint_us(uint16_t nbytes) = 0;

    Protocol using_protocol() const { return _using_protocol; }

    // OK, one pass-through for beginning with a baud rate...
    void begin(uint32_t baud);
    virtual void begin_locked(uint32_t baud, uint32_t key) = 0;

    // should_forward_mavlink_telemetry - returns true if this port should forward telemetry
    virtual bool should_forward_mavlink_telemetry() const = 0;

    bool get_mavlink_channel(mavlink_channel_t &_mav_chan) const;

    // protocol_match - returns true if the protocols match
    bool protocol_match(Protocol protocol1, Protocol protocol2) const;

    // setup any special options
    virtual void set_options();

    virtual uint32_t bw_in_kilobytes_per_second() const = 0;

    mavlink_channel_t mav_chan;
private:

    Protocol _using_protocol;
    uint8_t instance;

};

#pragma once

#include "AP_SensorHub_Debug.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

#if HAL_SENSORHUB_ENABLED
namespace SensorHub {

enum class msgid_t : uint8_t;
enum class decode_t { SUCCESS = 1, FAIL_ADV = -1, FAIL_CON = -2};

/* Packet structure */
class Packet {
public:
    static const uint8_t MAGIC = 0xFC;
    static const uint8_t VERSION = 0;

    // NOTE: This should take into account the rate at which packets are sent.
    typedef uint16_t seq_t;
    static const seq_t MAX_SEQ_COUNT = UINT16_MAX;

    static seq_t sequence_counter;

    // float conforms to IEEE 754.
    // NOTE: May not be entirely reliable, but should be sufficient.
    // https://stackoverflow.com/questions/5777484/how-to-check-if-c-compiler-uses-ieee-754-floating-point-standard
    static_assert(std::numeric_limits<float>::is_iec559, "Not IEEE 754 Float");

    typedef struct PACKED {
        uint8_t magic;
        uint8_t ver;
        uint8_t len; // TODO: We could get rid of the length if needed.
        msgid_t id;
        seq_t seq;
#if SENSORHUB_DEBUG_TIMESTAMP
        uint32_t timestamp;
#endif
    } hdr_t;

    typedef uint16_t crc_t;

    /*
     * Packet Structure.
     * NOTE: Data lives in each message's class.
     */
    typedef struct PACKED {
        hdr_t hdr;
        uint8_t *data;
        crc_t crc;
    } packet_t;


    static const size_t MAX_DATA_LEN = UINT8_MAX;

    static const size_t MAX_PACKET_LEN =
        sizeof(hdr_t) + MAX_DATA_LEN + sizeof(crc_t);

    static const size_t EMPTY_PACKET_LEN =
        sizeof(hdr_t) + sizeof(crc_t);

    static size_t length(packet_t *p)
    {
        return sizeof(hdr_t) + p->hdr.len + sizeof(crc_t);
    }

    /*
     * Encodes a message of type T into a packet.
     */
    template <class T> static void encode(packet_t *p);

    /*
     * Writes the packet to the specified buffer.
     */
    static inline void commit(packet_t *p, uint8_t *buf, size_t len)
    {
        memcpy(buf, &p->hdr, sizeof(hdr_t));
        memcpy(buf + sizeof(hdr_t), p->data, p->hdr.len);
        memcpy(buf + sizeof(hdr_t) + p->hdr.len, &p->crc, sizeof(crc_t));
    }

    /* Verifies that a packet_t is valid. */
    static bool verify(packet_t *p);

    /* Pre-checks the header information before verifying the packet. */
    static bool headerCheck(hdr_t *hdr);

    /*
     * Decodes packet from raw buffer.
     * NOTE: Packet data will point to data inside of buffer.
     */
    static int decode(packet_t *p, uint8_t *buf, size_t len, size_t &decoded_len);

    static msgid_t id(packet_t *p)
    {
        return p->hdr.id;
    }

private:
    /*
     * Find first instance of MAGIC.
     */
    static int findMagic(uint8_t *buf, size_t offset, size_t len);
};

/*
 * Message Types
 * NOTE: These should be ordered from most to least frequent.
 */
enum class msgid_t : uint8_t {
    GYRO,
    ACCEL,
    COMPASS,
    BARO,
    GPS,
    PARAM,
    UNKNOWN,
    LAST_MSG_ENTRY,
};

static const msgid_t MSGID_START = msgid_t::GYRO;
static const msgid_t MSGID_END = msgid_t::LAST_MSG_ENTRY;

static const char *msgid_t_names[] __attribute__((unused)) = {"Gyro", "Accel", "Compass", "Baro",
                                      "GPS", "Param", "Unknown"};

class Message {
public:
    /*
     * Encodes message into packet.
     */
    virtual Packet::packet_t *encode() = 0;

    /*
     * Verify if a valid packet is actually of type T
     */
    template <class T> static bool verify(Packet::packet_t *packet)
    {
        return packet->hdr.id == T::ID &&
               packet->hdr.len == sizeof(typename T::data_t);
    }

    /*
     * Decode message data from packet.
     */
    template <class T>
    static inline void decode(Packet::packet_t *packet, typename T::data_t *data)
    {
        memcpy(data, packet->data, sizeof(typename T::data_t));
    }

protected:
    Packet::packet_t _packet;

    // TODO: Consider templates or using wider datatype if necessary.
    static float linearScale(float x, float min, float max, float scale_min, float scale_max) {
        return ((scale_max - scale_min)*(x - min))/(max - min) + scale_min;
    }
};

class UnknownMessage : public Message {
public:
    static const msgid_t ID = msgid_t::UNKNOWN;
    typedef uint8_t data_t;

    virtual Packet::packet_t *encode()
    {
        return nullptr;
    }
};

class BaroMessage : public Message {
public:

    using temp_t = int16_t;
    using pressure_t = uint16_t;

    static const pressure_t PRESSURE_T_MAX = UINT16_MAX;
    static const pressure_t PRESSURE_T_MIN = 0;

    static const temp_t TEMP_T_MAX = INT16_MAX;
    static const temp_t TEMP_T_MIN = INT16_MIN;

    typedef struct PACKED {
        temp_t temperature;
        pressure_t pressure;
        uint8_t instance;
    } data_t;

    // NOTE: If this fails, need to change len data type
    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN, "PACKET DATA LENGTH EXCEEDED");

    static const size_t DATA_LENGTH = sizeof(data_t);
    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + DATA_LENGTH;

    static const msgid_t ID = msgid_t::BARO;

    static const uint16_t PRESSURE_SCALE = 100;
    static constexpr float PRESSURE_SCALE_ENCODE = 1.0/PRESSURE_SCALE;
    static constexpr float PRESSURE_SCALE_DECODE = PRESSURE_SCALE;

    // NOTE: in mbar
    static constexpr pressure_t PRESSURE_MIN = 10;
    static constexpr pressure_t PRESSURE_MAX = 1200;

    static const uint16_t TEMPERATURE_SCALE = 100;
    static constexpr float TEMPERATURE_SCALE_ENCODE = TEMPERATURE_SCALE;
    static constexpr float TEMPERATURE_SCALE_DECODE = 1.0/TEMPERATURE_SCALE;

    // NOTE: in C*100
    static const temp_t TEMPERATURE_MIN = -40 * TEMPERATURE_SCALE_ENCODE;
    static const temp_t TEMPERATURE_MAX = 85 * TEMPERATURE_SCALE_ENCODE;

    BaroMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setInstance(uint8_t instance)
    {
        _data.instance = instance;
    }

    void setPressure(float pressure)
    {
        auto scaled_pressure = pressureScaleToPacket(pressure);
        _data.pressure = scaled_pressure;
    }

    void setTemperature(float temperature)
    {
        auto scaled_temp = temperatureScaleToPacket(temperature);
        _data.temperature = scaled_temp;
    }

    virtual Packet::packet_t *encode()
    {
        Packet::encode<BaroMessage>(&_packet);
        return &_packet;
    }

    static float pressureScaleFromPacket(pressure_t data) {
        return Message::linearScale(data, PRESSURE_T_MIN, PRESSURE_T_MAX, PRESSURE_MIN, PRESSURE_MAX) * PRESSURE_SCALE_DECODE;
    }

    static pressure_t pressureScaleToPacket(float data) {
        return static_cast<pressure_t>(Message::linearScale(data * PRESSURE_SCALE_ENCODE, PRESSURE_MIN, PRESSURE_MAX, PRESSURE_T_MIN, PRESSURE_T_MAX));
    }

    static float temperatureScaleFromPacket(temp_t data) {
        return Message::linearScale(data, TEMP_T_MIN, TEMP_T_MAX, TEMPERATURE_MIN, TEMPERATURE_MAX) * TEMPERATURE_SCALE_DECODE;
    }

    static temp_t temperatureScaleToPacket(float data) {
        return static_cast<temp_t>(Message::linearScale(data * TEMPERATURE_SCALE_ENCODE, TEMPERATURE_MIN, TEMPERATURE_MAX, TEMP_T_MIN, TEMP_T_MAX));
    }

private:
    data_t _data;
};

class GyroMessage : public Message {
public:

    using gyro_t = int16_t;
    static const gyro_t GYRO_T_MAX = INT16_MAX;
    static const gyro_t GYRO_T_MIN = INT16_MIN;

    typedef struct PACKED {
        float dt;
        gyro_t gyrox;
        gyro_t gyroy;
        gyro_t gyroz;
        uint8_t devtype;
        uint8_t instance;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN, "PACKET DATA LENGTH EXCEEDED");

    static constexpr float GYRO_SCALE = 1000;
    static constexpr float GYRO_SCALE_ENCODE = GYRO_SCALE;
    static constexpr float GYRO_SCALE_DECODE = 1.0/GYRO_SCALE;

    // NOTE: +/- 2000 degrees
    static constexpr float GYRO_MAX = 34.906 * GYRO_SCALE_ENCODE;
    static constexpr float GYRO_MIN = -34.906 * GYRO_SCALE_ENCODE;

    static const size_t DATA_LENGTH = sizeof(data_t);
    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + DATA_LENGTH;

    static const msgid_t ID = msgid_t::GYRO;

    GyroMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setInstance(uint8_t instance)
    {
        _data.instance = instance;
    }

    void setId(uint32_t id)
    {
        _data.devtype = id >> 16;
    }

    void setGyro(const Vector3f &vec)
    {
        auto scaled_x = scaleToPacket(vec.x);
        auto scaled_y = scaleToPacket(vec.y);
        auto scaled_z = scaleToPacket(vec.z);
        _data.gyrox = scaled_x;
        _data.gyroy = scaled_y;
        _data.gyroz = scaled_z;
    }

    void setDt(float dt)
    {
        _data.dt = dt;
    }

    virtual Packet::packet_t *encode()
    {
        Packet::encode<GyroMessage>(&_packet);
        return &_packet;
    }

    static float scaleFromPacket(gyro_t gyro) {
        return Message::linearScale(gyro, GYRO_T_MIN, GYRO_T_MAX, GYRO_MIN, GYRO_MAX) * GYRO_SCALE_DECODE;
    }

    static gyro_t scaleToPacket(float gyro) {
        return static_cast<gyro_t>(Message::linearScale(gyro * GYRO_SCALE_ENCODE, GYRO_MIN, GYRO_MAX, GYRO_T_MIN, GYRO_T_MAX));
    }

private:
    data_t _data;
};

class AccelMessage : public Message {
public:
    using accel_t = int16_t;
    static const accel_t ACCEL_T_MAX = INT16_MAX;
    static const accel_t ACCEL_T_MIN = INT16_MIN;

    typedef struct PACKED {
        float dt;
        accel_t accelx;
        accel_t accely;
        accel_t accelz;
        uint8_t devtype;
        uint8_t instance;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN, "PACKET DATA LENGTH EXCEEDED");

    static constexpr float ACCEL_SCALE = 1000;
    static constexpr float ACCEL_SCALE_ENCODE = ACCEL_SCALE;
    static constexpr float ACCEL_SCALE_DECODE = 1.0/ACCEL_SCALE;

    // NOTE: +/- 16g
    static constexpr float ACCEL_MAX = 156.8 * ACCEL_SCALE_ENCODE;
    static constexpr float ACCEL_MIN = -156.8 * ACCEL_SCALE_ENCODE;

    static const size_t DATA_LENGTH = sizeof(data_t);
    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + DATA_LENGTH;

    static const msgid_t ID = msgid_t::ACCEL;

    AccelMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setInstance(uint8_t instance)
    {
        _data.instance = instance;
    }

    void setId(uint32_t id)
    {
        _data.devtype = id >> 16;
    }

    void setAccel(const Vector3f &vec)
    {
        auto scaled_x = scaleToPacket(vec.x);
        auto scaled_y = scaleToPacket(vec.y);
        auto scaled_z = scaleToPacket(vec.z);
        _data.accelx = scaled_x;
        _data.accely = scaled_y;
        _data.accelz = scaled_z;
    }

    void setDt(float dt)
    {
        _data.dt = dt;
    }

    virtual Packet::packet_t *encode()
    {
        Packet::encode<AccelMessage>(&_packet);
        return &_packet;
    }

    static float scaleFromPacket(accel_t accel) {
        return Message::linearScale(accel, ACCEL_T_MIN, ACCEL_T_MAX, ACCEL_MIN, ACCEL_MAX) * ACCEL_SCALE_DECODE;
    }

    static accel_t scaleToPacket(float accel) {
        return static_cast<int16_t>(Message::linearScale(accel * ACCEL_SCALE_ENCODE, ACCEL_MIN, ACCEL_MAX, ACCEL_T_MIN, ACCEL_T_MAX));
    }

private:
    data_t _data;
};

class CompassMessage : public Message {
public:

    typedef struct {
        float magx;
        float magy;
        float magz;
        uint8_t instance;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN, "PACKET DATA LENGTH EXCEEDED");

    static const size_t DATA_LENGTH = sizeof(data_t);
    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + DATA_LENGTH;

    static const msgid_t ID = msgid_t::COMPASS;

    CompassMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setInstance(uint8_t instance) {
        _data.instance = instance;
    }

    // NOTE: These are the raw fields in body frame.
    void setField(const Vector3f &field) {
        _data.magx = field.x;
        _data.magy = field.y;
        _data.magz = field.z;
    }

    virtual Packet::packet_t *encode()
    {
        Packet::encode<CompassMessage>(&_packet);
        return &_packet;
    }

private:
    data_t _data;
};

class GPSMessage : public Message {
public:

    // TODO: Optimize sizes here! its 50Hz in copter.
    typedef struct PACKED {
        float ground_speed;
        float ground_course;
        float velocityx;
        float velocityy;
        float velocityz;
        float speed_accuracy;
        float horizontal_accuracy;
        float vertical_accuracy;
        Location location;
        uint32_t time_week_ms;
        uint16_t time_week;
        uint16_t hdop;
        uint16_t vdop;
        AP_GPS::GPS_Status status;
        uint8_t num_sats;
        uint8_t instance;
        bool have_vertical_velocity:1;
        bool have_speed_accuracy:1;
        bool have_horizontal_accuracy:1;
        bool have_vertical_accuracy:1;
    } data_t;

    static_assert(sizeof(data_t) < Packet::MAX_DATA_LEN, "PACKET DATA LENGTH EXCEEDED");

    static const size_t DATA_LENGTH = sizeof(data_t);
    static const size_t PACKET_LENGTH = Packet::EMPTY_PACKET_LEN + DATA_LENGTH;

    static const msgid_t ID = msgid_t::GPS;

    GPSMessage() {
        _packet.data = reinterpret_cast<uint8_t *>(&_data);
    }

    void setInstance(uint8_t instance)
    {
        _data.instance = instance;
    }

    void setState(const AP_GPS::GPS_State &state)
    {
        _data.status                   = state.status;
        _data.time_week_ms             = state.time_week_ms;
        _data.time_week                = state.time_week;
        _data.location                 = state.location;
        _data.ground_speed             = state.ground_speed;
        _data.ground_course            = state.ground_course;
        _data.hdop                     = state.hdop;
        _data.vdop                     = state.vdop;
        _data.num_sats                 = state.num_sats;
        _data.velocityx                = state.velocity.x;
        _data.velocityy                = state.velocity.y;
        _data.velocityz                = state.velocity.z;
        _data.speed_accuracy           = state.speed_accuracy;
        _data.horizontal_accuracy      = state.horizontal_accuracy;
        _data.vertical_accuracy        = state.vertical_accuracy;
        _data.have_vertical_velocity   = state.have_vertical_velocity;
        _data.have_speed_accuracy      = state.have_speed_accuracy;
        _data.have_horizontal_accuracy = state.have_horizontal_accuracy;
        _data.have_vertical_accuracy   = state.have_vertical_accuracy;
    }

    virtual Packet::packet_t *encode()
    {
        Packet::encode<GPSMessage>(&_packet);
        return &_packet;
    }

private:
    data_t _data;
};

}
#endif
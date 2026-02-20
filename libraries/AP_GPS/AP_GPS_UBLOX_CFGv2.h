/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "AP_GPS_UBLOX_CFG_List.h"
#include <AP_Common/AP_Common.h>
#include "AP_GPS_config.h"

#if AP_GPS_UBLOX_CFGV2_ENABLED

// this is number of epochs per output. A higher value will reduce
// the uart bandwidth needed and allow for higher latency
#define RTK_MB_RTCM_RATE 1

#define UBX_NAVSPG_DYNMODEL_AIR4G 8

#define RATE_PVT 1


class AP_GPS_UBLOX;

// -----------------------------------------------------------------------------
// UBX CFG key size helpers (Bits 30..28 encode the value storage size)
// 0x01: one bit (stored in one byte; only LSB significant)
// 0x02: one byte
// 0x03: two bytes
// 0x04: four bytes
// 0x05: eight bytes
// -----------------------------------------------------------------------------
#define UBX_CFG_KEY_SIZE_SHIFT            (28U)
#define UBX_CFG_KEY_SIZE_MASK             (0x7LU << UBX_CFG_KEY_SIZE_SHIFT)
#define UBX_CFG_KEY_SIZE_CODE_BIT         (0x1U)
#define UBX_CFG_KEY_SIZE_CODE_1B          (0x2U)
#define UBX_CFG_KEY_SIZE_CODE_2B          (0x3U)
#define UBX_CFG_KEY_SIZE_CODE_4B          (0x4U)
#define UBX_CFG_KEY_SIZE_CODE_8B          (0x5U)
#define UBX_CFG_KEY_SIZE_CODE(KEY_)       (((uint32_t)(KEY_) >> UBX_CFG_KEY_SIZE_SHIFT) & 0x7U)

// Convenience macro to declare a key/value pair without using STL
// Enforces the correct value type per key via UBXCfgKV<>, usable with blob.push(...)
#define UBX_CFG_KV(KEY_, VAL_) \
    AP_GPS_UBLOX_CFGv2::UBXCfgKV<KEY_>( (typename AP_GPS_UBLOX_CFGv2::UBXCfgKV<KEY_>::ValueType)(VAL_) )


#define UBX_CFG_BLOB_SIZE_FROM_KV(...) \
    (UBXCfgBlobSizeFromKVExpr(__VA_ARGS__))

//---------------------------------------------------------------------------------
// AP_GPS_UBLOX_CFGv2 class used to configure modern ublox units with configuration
// key value pair method of configuration
//----------------------------------------------------------------------------------
class AP_GPS_UBLOX_CFGv2 {
    using ConfigKey = AP::UBXConfigKey;
private:
    friend class AP_GPS_UBLOX;

public:
    AP_GPS_UBLOX_CFGv2(AP_GPS_UBLOX &_ubx_backend);

    // -----------------------------------------------------------------------------
    // UBX key/value blob builder for dynamic configuration. Produces a packed sequence:
    //   [key:uint32][value:len] repeated for all pairs, where len depends on bits 30..28 of key
    // Access results via get_blob() and get_size().
    // -----------------------------------------------------------------------------
    class UBXPackedCfg {
    public:
        UBXPackedCfg(uint8_t *buf, size_t capacity) : _buf(buf), _size(0), _capacity(capacity) {}

        const uint8_t *get() const { return _buf; }
        size_t get_size() const { return _size; }

        // Reset the packed config to empty state
        void reset();

        // Following templates enforce correct type for key at compile time
        // unconditional push
        template <ConfigKey Key, typename T>
        bool push(const T value) {
            static_assert(config_key_size(Key) == sizeof(T),
                          "UBXCfgBlob: invalid type for key");
            // do not accept signed types or floating point types
            static_assert(std::is_integral<T>::value && !std::is_signed<T>::value,
                          "UBXCfgBlob: only unsigned integral types supported, cast before push");
            return push(Key, value);
        }

        // conditional push
        template <ConfigKey Key, typename T>
        bool push(const T curr_value, const T desired_value) {
            if (curr_value != desired_value) {
                return push<Key>(desired_value);
            }
            return true;
        }

        // common constexpr helper with compile time type size inference
        template <typename T>
        static void append_value_le_bytes(uint8_t *dest, size_t &index, T value) {
            append_value_le_bytes(dest, index, (uint64_t)value, sizeof(T));
        }

        // get value by key
        bool get(ConfigKey key, uint64_t &value) const;

        // set value by key
        bool set(ConfigKey key, uint64_t value);

        // get key at byte offset, returns next offset or 0 if end
        bool get_key_at_offset(uint16_t &offset, ConfigKey &key) const;

        // Push raw value for a key (length is inferred from key's size bits)
        bool push(const ConfigKey key, const uint64_t value);

        void set_size(size_t size) {
            if (size <= _capacity) {
                _size = size;
            }
        }
    private:

        static void append_value_le_bytes(uint8_t *dest, size_t &index, uint64_t value, size_t size);

        // return size of a config key, or 0 if unknown
        static constexpr uint8_t config_key_size(ConfigKey key) {
            // step over the value
            return (((uint32_t(key) >> 28) & 0x07) == 0x1) ? 1 : // bit
                (((uint32_t(key) >> 28) & 0x07) == 0x2) ? 1 : // byte
                (((uint32_t(key) >> 28) & 0x07) == 0x3) ? 2 : // 2 bytes
                (((uint32_t(key) >> 28) & 0x07) == 0x4) ? 4 : // 4 bytes
                (((uint32_t(key) >> 28) & 0x07) == 0x5) ? 8 : // 8 bytes
                0; // invalid
        }
        uint8_t *_buf;
        size_t _size{0};
        size_t _capacity{0};
    };

    enum States : uint8_t {
        IDENTIFY_MODULE,
        RESET_MODULE,
        IDENTIFY_SIGNALS,
        CONFIGURE_GNSS,
        IDENTIFY_UART_PORT,
        FETCH_COMMON_CONFIG,
        SET_COMMON_CONFIG,
        CONFIGURED,
        FAILED,
    };

    enum class Module  : uint8_t {
        // Units with single UART
        UNKNOWN,
        M9,
        M10,
        F10,
        SINGLE_UART_LAST,
        // Units with dual UARTs
        NEO_F9,
        ZED_F9,
        F20,
        X20,
    };

    // run CFGv2 configuration state machine
    void update();

    // process a single byte from a VALGET response payload
    void process_valget_byte(uint8_t byte);
    // called when VALGET response is fully received or timed out
    void process_valget_complete(bool success);
    bool valget_active() const { return _valget_in_progress; }

    // handle ACK/NACK for CFG-VALSET/VALDEL messages
    void handle_cfg_ack(uint8_t msg_id);
    void handle_cfg_nack(uint8_t msg_id);

    // delete all config keys and reset module to defaults
    bool delete_and_reset_config();

    static void override_ubx_cfg(uint8_t instance, uint8_t *cfg, size_t size) {
        _override_common_cfg[instance] = new UBXPackedCfg(cfg, size);
        _override_common_cfg[instance]->set_size(size);
    }
private:
    struct PACKED config_list {
        AP_GPS_UBLOX_CFGv2::ConfigKey key;
        // support up to 4 byte values, assumes little-endian
        uint32_t value;
    };

    bool using_legacy_config() const;
    bool skip_cfgv2() const;

    // List of All signals supported by u-blox
    // Used for identifying supported signals and configuring them
    enum SupportedSignals: uint32_t {
        #define SIG_X(NAME, INDEX) NAME##_SIG = INDEX,
        UBLOX_SIGNALS(SIG_X)
        #undef SIG_X
        SIGNALS_LAST
    };

    // Constellation names aligned with AP_GPS_UBLOX::GNSS_* indices
    static const char* const CONSTELLATION_NAMES[];

    AP_GPS_UBLOX &ubx_backend;
    States curr_state;

    uint32_t _last_request_time;

    bool _identify_module();
    Module module;

    struct {
        // bit masks of supported and enabled constellations and signals
        uint8_t supported_gnss;
        uint8_t enabled_gnss;
        uint32_t supported_signals;
        uint32_t enabled_signals;

        // common constant config values
        #define COMMON_CFG(KEY_CLASS, KEY, TYPE, VAL) TYPE KEY##_val;
        UBX_CFG_COMMON_UART1(COMMON_CFG)
        #undef COMMON_CFG

        // variable config values
        uint16_t meas_rate;
        uint8_t dynmodel;
        int8_t min_elevation;
        bool gps_l5_health_ovrd;
        bool gps_l5_health_ovrd_exists;
    } _cfg;

    void _handle_valget_kv(ConfigKey key, uint64_t value, uint8_t value_len);
    void _parse_signal_kv(ConfigKey key, uint64_t value, uint8_t value_len);
    void _parse_common_cfg(ConfigKey key, uint64_t value, uint8_t value_len);
    void _publish_supported_constellations();
    // send GNSS constellation enables based on gnss_mode param, returns true if config was sent
    bool _send_gnss_cfg();

    // Signal names aligned with UBLOX_SIGNALS indices
    static const char* const SIGNAL_NAMES[];
    bool _request_common_cfg();
    bool set_common_cfg();
    bool is_common_cfg_needed();

    // Common config packed buffer management
    bool _init_common_cfg_list(bool check_only = false, uint32_t key_to_check = 0);
    bool fetch_all_config();

    bool _valget_in_progress;
    bool _valget_abort;                  // if true, ignore all bytes until process_valget_complete()
    uint8_t _valget_hdr_count;           // bytes of VALGET header seen (version/layers/reserved)
    uint8_t _valget_key_count;           // bytes of current key seen
    uint8_t _valget_value_expected;      // expected value length for current key
    uint8_t _valget_value_count;         // bytes of current value seen
    uint8_t _valget_kv_count;            // count of key-value pairs in current response
    uint16_t _valget_position;           // position for next VALGET request (increments by 64)
    uint32_t _valget_key;                // current key (little-endian)
    uint64_t _valget_value;              // current value (little-endian)

    // Common config management - single source of truth
    static constexpr size_t COMMON_CFG_BUFFER_SIZE = 100;
    uint8_t _common_cfg_buffer[COMMON_CFG_BUFFER_SIZE];
    UBXPackedCfg _common_cfg;
    static UBXPackedCfg *_override_common_cfg[GPS_MAX_INSTANCES];
    uint16_t _common_cfg_fetch_index;    // tracks progress through config list building

    // Buffer to track mismatched config that needs to be set
    uint8_t _common_cfg_to_set_buffer[COMMON_CFG_BUFFER_SIZE];
    UBXPackedCfg _common_cfg_to_set;

    enum ConfigLayer : uint8_t {
        CFG_LAYER_RAM = 0x00,
        CFG_LAYER_BBR = 0x01,
        CFG_LAYER_FLASH = 0x02,
        CFG_LAYER_DEFAULT = 0x07,
        CFG_LAYER_ALL = 0x07,
    };

    bool _request_cfg_group(ConfigKey group, ConfigLayer layer);
    // send a CFG-VALSET made of repeated [key:uint32][value:N] pairs in 'bytes'
    // streams without allocating a new combined buffer
    bool _send_valset_bytes(const uint8_t *bytes, uint16_t size, ConfigLayer layers);

    bool _send_reset(uint16_t navBbrMask, uint8_t resetMode);

    static bool _module_reset[GPS_MAX_INSTANCES];
    bool _is_reset_required;
    ConfigLayer fetch_config_layer = ConfigLayer::CFG_LAYER_BBR;

    bool _single_key_valset;  // send one key at a time on NACK fallback
    uint16_t _valset_send_offset; // progress through packed buffer in single-key mode
    uint8_t portId {0xFF}; // port ID for dual-UART modules, 0xFF means not detected yet
};
#endif // AP_GPS_UBLOX_CFGV2_ENABLED

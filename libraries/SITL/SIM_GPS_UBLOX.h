#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_UBlox : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_UBlox);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

    // read and act on UBX configuration messages from the autopilot
    void update_read() override;

    // receiver models, selected with SIM_GPSn_UBXMDL.  All but
    // SIMPLE implement enough of the UBX configuration protocol to
    // be configured by the autopilot, and only output the messages
    // they have been configured to output
    enum class Model : uint8_t {
        SIMPLE = 0,      // outputs everything, ignores configuration
        UBLOX6 = 1,      // u-blox 6, FW 7.03, ROM
        M8N_FW201 = 2,   // NEO-M8N, FW 2.01, flash
        M8_ROM301 = 3,   // M8, ROM CORE 3.01, no flash
        M8N_SPG301 = 4,  // NEO-M8N, SPG 3.01, flash
        M9N_SPG404 = 5,  // NEO-M9N, SPG 4.04, flash
        M10_SPG510 = 6,  // M10, ROM SPG 5.10
    };

private:
    enum RELPOSNED {
        gnssFixOK          = 1U << 0,
        diffSoln           = 1U << 1,
        relPosValid        = 1U << 2,
        carrSolnFloat      = 1U << 3,

        carrSolnFixed      = 1U << 4,
        isMoving           = 1U << 5,
        refPosMiss         = 1U << 6,
        refObsMiss         = 1U << 7,

        relPosHeadingValid = 1U << 8,
        relPosNormalized   = 1U << 9
    };
    struct PACKED ubx_nav_relposned {
        uint8_t version;
        uint8_t reserved1;
        uint16_t refStationId;
        uint32_t iTOW;
        int32_t relPosN;
        int32_t relPosE;
        int32_t relPosD;
        int32_t relPosLength;
        int32_t relPosHeading;
        uint8_t reserved2[4];
        int8_t relPosHPN;
        int8_t relPosHPE;
        int8_t relPosHPD;
        int8_t relPosHPLength;
        uint32_t accN;
        uint32_t accE;
        uint32_t accD;
        uint32_t accLength;
        uint32_t accHeading;
        uint8_t reserved3[4];
        uint32_t flags;
    };

    uint32_t _next_timegps_send_ms;

    void update_relposned(ubx_nav_relposned &relposned, uint32_t tow_ms, float yaw_deg);
    void send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
    void send_ubx(uint8_t msgclass, uint8_t msgid, const uint8_t *buf, uint16_t size);

    /*
      configuration simulation
     */

    // messages whose output rate we track
    struct MsgID {
        uint8_t msg_class;
        uint8_t msg_id;
    };
    static const MsgID tracked_msgs[14];

    // CFG-VALGET/VALSET keys we support, other than CFG-NAVSPG-DYNMODEL
    // which is kept in the NAV5 settings
    struct ConfigKeyInfo {
        uint32_t key;
        uint8_t default_value;
    };
    static const ConfigKeyInfo config_keys[10];

    // receiver configuration; RAM and saved copies of this are kept,
    // and the saved copy is stored in a file so it survives a reboot
    struct PACKED Config {
        uint8_t rate[ARRAY_SIZE(tracked_msgs)];  // rates on our port
        uint16_t measure_rate_ms;
        uint16_t nav_rate;
        uint16_t timeref;
        uint8_t nav5[36];
        uint8_t sbas[8];
        uint8_t gnss_len;
        uint8_t gnss[4+8*7];
        uint8_t key_value[ARRAY_SIZE(config_keys)];
    };
    struct PACKED SavedConfig {
        uint32_t magic;
        uint8_t model;
        Config config;
    };

    Model model() const;
    bool config_simulated() const { return model() != Model::SIMPLE; }
    bool has_flash() const;
    bool supports(uint8_t msg_class, uint8_t msg_id) const;
    int8_t tracked_index(uint8_t msg_class, uint8_t msg_id) const;
    void factory_config(Config &cfg) const;
    void load_saved_config();
    void store_saved_config() const;
    void config_filename(char *buf, uint8_t buflen) const;
    bool should_send(uint8_t msg_class, uint8_t msg_id);

    void handle_ubx(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len);
    void handle_cfg(uint8_t msg_id, const uint8_t *payload, uint16_t len);
    bool supports_valset() const;
    uint8_t *config_value(Config &cfg, uint32_t key);
    void handle_valget(const uint8_t *payload, uint16_t len);
    void handle_valset(const uint8_t *payload, uint16_t len);
    void send_ack(uint8_t msg_class, uint8_t msg_id, bool ack);
    void send_mon_ver();
    void send_cfg_prt();
    void send_svinfo(uint32_t tow_ms, uint8_t num_sats);

    bool config_loaded;
    Model loaded_model;
    Config ram;
    Config saved;
    uint32_t nav_count;  // count of navigation solutions output

    // UBX parser state
    struct {
        uint8_t step;
        uint8_t msg_class;
        uint8_t msg_id;
        uint16_t len;
        uint16_t count;
        uint8_t ck_a;
        uint8_t ck_b;
        uint8_t payload[256];
    } parser;
};

};

#endif  // AP_SIM_GPS_UBLOX_ENABLED

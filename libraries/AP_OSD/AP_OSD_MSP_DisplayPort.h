#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_MSP/AP_MSP.h>

#if HAL_WITH_MSP_DISPLAYPORT

class AP_OSD_MSP_DisplayPort : public AP_OSD_Backend
{
    using AP_OSD_Backend::AP_OSD_Backend;
public:
    static AP_OSD_Backend *probe(AP_OSD &osd);

    //initialize display port and underlying hardware
    bool init() override;

    //draw given text to framebuffer
    void write(uint8_t x, uint8_t y, const char* text) override;

    //flush framebuffer to screen
    void flush() override;

    //clear framebuffer
    void clear() override;

    // copy the backend specific symbol set to the OSD lookup table
    void init_symbol_set(uint8_t *lookup_table, const uint8_t size) override;

    // called by the OSD thread once
    // used to initialize the uart in the correct thread
    void osd_thread_run_once() override;

    // return a correction factor used to display angles correctly
    float get_aspect_ratio_correction() const override;
    
    bool is_compatible_with_backend_type(AP_OSD::osd_types type) const override {
        switch(type) {
        case AP_OSD::osd_types::OSD_MSP:
        case AP_OSD::osd_types::OSD_MSP_DISPLAYPORT:
            return false;
        case AP_OSD::osd_types::OSD_NONE:
        case AP_OSD::osd_types::OSD_TXONLY:
        case AP_OSD::osd_types::OSD_MAX7456:
        case AP_OSD::osd_types::OSD_SITL:
            return true;
        }
        return false;
    }

    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_MSP_DISPLAYPORT;
    }
protected:
    uint8_t format_string_for_osd(char* dst, uint8_t size, bool decimal_packed, const char *fmt, va_list ap) override;

private:
    void setup_defaults(void);

    AP_MSP_Telem_Backend* _displayport;

    // MSP DisplayPort symbols
    static const uint8_t SYM_M = 0x0C;
    static const uint8_t SYM_KM = 0x7D;
    static const uint8_t SYM_FT = 0x0F;
    static const uint8_t SYM_MI = 0x7E;
    static const uint8_t SYM_ALT_M = 0x0C;
    static const uint8_t SYM_ALT_FT = 0x0F;
    static const uint8_t SYM_BATT_FULL = 0x90;
    static const uint8_t SYM_RSSI = 0x01;

    static const uint8_t SYM_VOLT = 0x06;
    static const uint8_t SYM_AMP = 0x9A;
    static const uint8_t SYM_MAH = 0x07;
    static const uint8_t SYM_MS = 0x9F;
    static const uint8_t SYM_FS = 0x99;
    static const uint8_t SYM_KMH = 0x9E;
    static const uint8_t SYM_MPH = 0x9D;
    static const uint8_t SYM_DEGR = 0x1D;
    static const uint8_t SYM_PCNT = 0x25;
    static const uint8_t SYM_RPM = 0x12;
    static const uint8_t SYM_ASPD = 0x41;
    static const uint8_t SYM_GSPD = 0x70;
    static const uint8_t SYM_WSPD = 0x1B;
    static const uint8_t SYM_VSPD = 0x7F;
    static const uint8_t SYM_WPNO = 0x23;
    static const uint8_t SYM_WPDIR = 0xE6;
    static const uint8_t SYM_WPDST = 0xE7;
    static const uint8_t SYM_FTMIN = 0xE8;
    static const uint8_t SYM_FTSEC = 0x99;

    static const uint8_t SYM_SAT_L = 0x1E;
    static const uint8_t SYM_SAT_R = 0x1F;
    static const uint8_t SYM_HDOP_L = 0x11;
    static const uint8_t SYM_HDOP_R = 0x08;

    static const uint8_t SYM_HOME = 0x05;
    static const uint8_t SYM_WIND = 0x57;

    static const uint8_t SYM_ARROW_START = 0x60;
    static const uint8_t SYM_ARROW_COUNT = 16;
    static const uint8_t SYM_AH_H_START = 0x80;
    static const uint8_t SYM_AH_H_COUNT = 9;

    static const uint8_t SYM_AH_V_START = 0x82;
    static const uint8_t SYM_AH_V_COUNT = 6;

    static const uint8_t SYM_AH_CENTER_LINE_LEFT = 0x84;
    static const uint8_t SYM_AH_CENTER_LINE_RIGHT = 0x84;
    static const uint8_t SYM_AH_CENTER = 0x2B;

    static const uint8_t SYM_HEADING_N = 0x18;
    static const uint8_t SYM_HEADING_S = 0x19;
    static const uint8_t SYM_HEADING_E = 0x1A;
    static const uint8_t SYM_HEADING_W = 0x1B;
    static const uint8_t SYM_HEADING_DIVIDED_LINE = 0x1C;
    static const uint8_t SYM_HEADING_LINE = 0x1D;

    static const uint8_t SYM_UP_UP = 0x68;
    static const uint8_t SYM_UP = 0x68;
    static const uint8_t SYM_DOWN = 0x60;
    static const uint8_t SYM_DOWN_DOWN = 0x60;

    static const uint8_t SYM_DEGREES_C = 0x0E;
    static const uint8_t SYM_DEGREES_F = 0x0D;
    static const uint8_t SYM_GPS_LAT = 0x68;
    static const uint8_t SYM_GPS_LONG = 0x6C;
    static const uint8_t SYM_ARMED = 0x08;
    static const uint8_t SYM_DISARMED = 0x08;
    static const uint8_t SYM_ROLL0 = 0x2D;
    static const uint8_t SYM_ROLLR = 0x64;
    static const uint8_t SYM_ROLLL = 0x6C;
    static const uint8_t SYM_PTCH0 = 0x7C;
    static const uint8_t SYM_PTCHUP = 0x68;
    static const uint8_t SYM_PTCHDWN = 0x60;
    static const uint8_t SYM_XERR = 0xEE;
    static const uint8_t SYM_KN = 0xF0;
    static const uint8_t SYM_NM = 0xF1;
    static const uint8_t SYM_DIST = 0x22;
    static const uint8_t SYM_FLY = 0x9C;
    static const uint8_t SYM_EFF = 0xF2;
    static const uint8_t SYM_AH = 0xF3;
    static const uint8_t SYM_MW = 0xF4;
    static const uint8_t SYM_CLK = 0x08;
    static const uint8_t SYM_KILO = 0x4B;
    static const uint8_t SYM_TERALT = 0xEF;
    static const uint8_t SYM_FENCE_ENABLED = 0xF5;
    static const uint8_t SYM_FENCE_DISABLED = 0xF6;
    static const uint8_t SYM_RNGFD = 0x72;
    static const uint8_t SYM_LQ = 0xF8;

    static constexpr uint8_t symbols[AP_OSD_NUM_SYMBOLS] {
        SYM_M,
        SYM_KM,
        SYM_FT,
        SYM_MI,
        SYM_ALT_M,
        SYM_ALT_FT,
        SYM_BATT_FULL,
        SYM_RSSI,
        SYM_VOLT,
        SYM_AMP,
        SYM_MAH,
        SYM_MS,
        SYM_FS,
        SYM_KMH,
        SYM_MPH,
        SYM_DEGR,
        SYM_PCNT,
        SYM_RPM,
        SYM_ASPD,
        SYM_GSPD,
        SYM_WSPD,
        SYM_VSPD,
        SYM_WPNO,
        SYM_WPDIR,
        SYM_WPDST,
        SYM_FTMIN,
        SYM_FTSEC,
        SYM_SAT_L,
        SYM_SAT_R,
        SYM_HDOP_L,
        SYM_HDOP_R,
        SYM_HOME,
        SYM_WIND,
        SYM_ARROW_START,
        SYM_ARROW_COUNT,
        SYM_AH_H_START,
        SYM_AH_H_COUNT,
        SYM_AH_V_START,
        SYM_AH_V_COUNT,
        SYM_AH_CENTER_LINE_LEFT,
        SYM_AH_CENTER_LINE_RIGHT,
        SYM_AH_CENTER,
        SYM_HEADING_N,
        SYM_HEADING_S,
        SYM_HEADING_E,
        SYM_HEADING_W,
        SYM_HEADING_DIVIDED_LINE,
        SYM_HEADING_LINE,
        SYM_UP_UP,
        SYM_UP,
        SYM_DOWN,
        SYM_DOWN_DOWN,
        SYM_DEGREES_C,
        SYM_DEGREES_F,
        SYM_GPS_LAT,
        SYM_GPS_LONG,
        SYM_ARMED,
        SYM_DISARMED,
        SYM_ROLL0,
        SYM_ROLLR,
        SYM_ROLLL,
        SYM_PTCH0,
        SYM_PTCHUP,
        SYM_PTCHDWN,
        SYM_XERR,
        SYM_KN,
        SYM_NM,
        SYM_DIST,
        SYM_FLY,
        SYM_EFF,
        SYM_AH,
        SYM_MW,
        SYM_CLK,
        SYM_KILO,
        SYM_TERALT,
        SYM_FENCE_ENABLED,
        SYM_FENCE_DISABLED,
        SYM_RNGFD,
        SYM_LQ,
    };

    bool _blink_on;
};
#endif

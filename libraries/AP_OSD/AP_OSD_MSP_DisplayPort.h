#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_MSP/AP_MSP.h>

#if HAL_WITH_MSP_DISPLAYPORT

#define DISPLAYPORT_WRITE_BUFFER_MAX_LEN 30

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
    char displayport_write_buffer[DISPLAYPORT_WRITE_BUFFER_MAX_LEN]; // terminator

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
    static const uint8_t SYM_DEGR = 0x08;
    static const uint8_t SYM_PCNT = 0x25;
    static const uint8_t SYM_RPM = 0x12;
    static const uint8_t SYM_ASPD = 0x41;
    static const uint8_t SYM_GSPD = 0x47;
    static const uint8_t SYM_WSPD = 0x57;
    static const uint8_t SYM_VSPD = 0x5E;
    static const uint8_t SYM_WPNO = 0x23;
    static const uint8_t SYM_WPDIR = 0xE6;
    static const uint8_t SYM_WPDST = 0xE7;
    static const uint8_t SYM_FTMIN = 0xE8;
    static const uint8_t SYM_FTSEC = 0x99;

    static const uint8_t SYM_SAT_L = 0x1E;
    static const uint8_t SYM_SAT_R = 0x1F;
    static const uint8_t SYM_HDOP_L = 0x48;
    static const uint8_t SYM_HDOP_R = 0x44;

    static const uint8_t SYM_HOME = 0x11;
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
    static const uint8_t SYM_GPS_LAT = 0x89;
    static const uint8_t SYM_GPS_LONG = 0x98;
    static const uint8_t SYM_ARMED = 0x00;
    static const uint8_t SYM_DISARMED = 0x2A;
    static const uint8_t SYM_ROLL0 = 0x2D;
    static const uint8_t SYM_ROLLR = 0x64;
    static const uint8_t SYM_ROLLL = 0x6C;
    static const uint8_t SYM_PTCH0 = 0x7C;
    static const uint8_t SYM_PTCHUP = 0x68;
    static const uint8_t SYM_PTCHDWN = 0x60;
    static const uint8_t SYM_XERR = 0x21;
    static const uint8_t SYM_KN = 0xF0;
    static const uint8_t SYM_NM = 0xF1;
    static const uint8_t SYM_DIST = 0x04;
    static const uint8_t SYM_FLY = 0x9C;
    static const uint8_t SYM_EFF = 0xF2;
    static const uint8_t SYM_AH = 0xF3;
    static const uint8_t SYM_MW = 0xF4;
    static const uint8_t SYM_CLK = 0x08;
    static const uint8_t SYM_KILO = 0x4B;
    static const uint8_t SYM_TERALT = 0x7F;
    static const uint8_t SYM_FENCE_ENABLED = 0xF5;
    static const uint8_t SYM_FENCE_DISABLED = 0xF6;
    static const uint8_t SYM_RNGFD = 0x7F;
    static const uint8_t SYM_LQ = 0xF8;

    static const uint8_t SYM_SIDEBAR_L_ARROW = 0x02;
    static const uint8_t SYM_SIDEBAR_R_ARROW = 0x03;
    static const uint8_t SYM_SIDEBAR_A = 0x13;
    static const uint8_t SYM_SIDEBAR_B = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_C = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_D = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_E = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_F = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_G = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_H = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_I = SYM_SIDEBAR_A;
    static const uint8_t SYM_SIDEBAR_J = SYM_SIDEBAR_A;


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
        SYM_SIDEBAR_L_ARROW,
        SYM_SIDEBAR_R_ARROW,
        SYM_SIDEBAR_A,
        SYM_SIDEBAR_B,
        SYM_SIDEBAR_C,
        SYM_SIDEBAR_D,
        SYM_SIDEBAR_E,
        SYM_SIDEBAR_F,
        SYM_SIDEBAR_G,
        SYM_SIDEBAR_H,
        SYM_SIDEBAR_I,
        SYM_SIDEBAR_J,
    };
#if AP_MSP_INAV_FONTS_ENABLED
    static constexpr uint8_t ap_to_inav_symbols_map[256][2] = {
        {0x00,0},
        {0x01,0},
        {0x02,0},
        {0x03,0},
        {0x04,0},
        {0x05,0},
        {0x1F,0},
        {0x99,0},
        {0x08,0},
        {0x3E,0},
        {0x3C,0},
        {0x0B,0},
        {0x0C,0},
        {0x96,0},
        {0x97,0},
        {0x74,0},
        {0x10,0},
        {0x2E,1},
        {0x31,1},
        {0x2E,1},
        {0x31,1},
        {0x2E,1},
        {0x16,0},
        {0x17,0},
        {0x4E,0},
        {0x53,0},
        {0x45,0},
        {0x57,0},
        {0x7C,0},
        {0x7C,0},
        {0x09,0},
        {0xE0,0},
        // 00-1F above        
        {0x20,0},
        {0x21,0},
        {0x75,0},
        {0xD0,0},
        {0x24,0},
        {0x25,0},
        {0x5F,1},
        {0x5A,1},
        {0x28,0},
        {0x29,0},
        {0x2A,0},
        {0x2B,0},
        {0x2C,0},
        {0xAE,0},
        {0x2E,0},
        {0x2F,0},
        {0x30,0},
        {0x31,0},
        {0x32,0},
        {0x33,0},
        {0x34,0},
        {0x35,0},
        {0x36,0},
        {0x37,0},
        {0x38,0},
        {0x39,0},
        {0x3A,0},
        {0x3B,0},
        {0x3C,0},
        {0x3D,0},
        {0x3E,0},
        {0x3F,0},
        //20-3F above
        {0x40,0},
        {0x41,0},
        {0x42,0},
        {0x43,0},
        {0x44,0},
        {0x45,0},
        {0x46,0},
        {0x47,0},
        {0x48,0},
        {0x49,0},
        {0x4A,0},
        {0x4B,0},
        {0x4C,0},
        {0x4D,0},
        {0x4E,0},
        {0x4F,0},
        {0x50,0},
        {0x51,0},
        {0x52,0},
        {0x53,0},
        {0x54,0},
        {0x55,0},
        {0x56,0},
        {0x57,0},
        {0x58,0},
        {0x59,0},
        {0x5A,0},
        {0x5B,0},
        {0x5C,0},
        {0x5D,0},
        {0x5E,0},
        {0x5F,0},
        //40-5F above
        {0x3C,1},
        {0x3D,1},
        {0x3E,1},
        {0x3F,1},
        {0x40,1},
        {0x41,1},
        {0x42,1},
        {0x43,1},
        {0x44,1},
        {0x45,1},
        {0x46,1},
        {0x47,1},
        {0x48,1},
        {0x49,1},
        {0x4A,1},
        {0x4B,0},
        {0x70,0},
        {0x71,0},
        {0x72,0},
        {0x73,0},
        {0x74,0},
        {0x75,0},
        {0x76,0},
        {0x77,0},
        {0x78,0},
        {0x79,0},
        {0x20,0},
        {0x7B,0},
        {0xAE,0},
        {0x7D,0},
        {0x66,1},
        {0x7F,0},
        // 60-7F above
        {0x4C,1},
        {0x4D,1},
        {0x4E,1},
        {0x4F,1},
        {0x50,1},
        {0x51,1},
        {0x52,1},
        {0x53,1},
        {0x54,1},
        {0x0C,0},
        {0x8A,0},
        {0x8B,0},
        {0x8C,0},
        {0x8D,0},
        {0x8E,0},
        {0x8F,0},
        {0x63,0},
        {0x91,0},
        {0x92,0},
        {0x93,0},
        {0x94,0},
        {0x95,0},
        {0x96,0},
        {0xDD,0},
        {0x98,0},
        {0x8D,0},
        {0x6A,0},
        {0x9B,0},
        {0x9F,0},
        {0x9D,0},
        {0x9E,0},
        {0x8F,0},
        //80-9F above
        {0xA0,0},
        {0x90,0},
        {0x55,1},
        {0x56,1},
        {0x57,1},
        {0x58,1},
        {0x03,0},
        {0x04,0},
        {0x0B,0},
        {0x52,0},
        {0x20,0},
        {0xAB,0},
        {0xAC,0},
        {0xAD,0},
        {0x71,0},
        {0x50,0},
        {0x91,0},
        {0x76,0},
        {0xB2,0},
        {0x78,0},
        {0xB4,0},
        {0xB5,0},
        {0xB6,0},
        {0xB7,0},
        {0xB8,0},
        {0x82,0},
        {0x83,0},
        {0x84,0},
        {0xA0,0},
        {0x0E,0},
        {0x0F,0},
        {0x10,0},
        //A0-BF above
        {0xA1,0},
        {0xA2,0},
        {0xA3,0},
        {0xA4,0},
        {0xA5,0},
        {0xA6,0},
        {0xA7,0},
        {0xA8,0},
        {0xA9,0},
        {0xAA,0},
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0x5A,1},
        {0xB1,0},
        {0xB2,0},
        {0xB3,0},
        {0xB4,0},
        {0xB5,0},
        {0xB6,0},
        {0xB7,0},
        {0xB8,0},
        {0xB9,0},
        {0xBA,0},
        {0x2E,1},
        {0x2E,1},
        {0x31,1},
        {0x31,1},
        {0x31,1},
        {0xBC,0},
        //C0-DF above
        {0x8B,0},
        {0x8C,0},
        {0x20,0},
        {0x86,0},
        {0x5E,0},
        {0x57,0},
        {0x20,0},
        {0x20,0},
        {0x8E,0},
        {0x65,1},
        {0xAF,0},
        {0xAD,0},
        {0x15,0},
        {0x16,0},
        {0xDC,0},
        {0x59,1},
        {0x92,0},
        {0x85,0},
        {0x20,0},
        {0xD3,0},
        {0x72,0},
        {0x91,1},
        {0x20,0},
        {0xBA,1},
        {0x02,0},
        {0x12,0},
        {0x13,0},
        {0x14,0},
        {0x59,0},
        {0x40,1},
        {0x48,1},
        {0xFF,0}
        //E0-FF above
    };
#endif //AP_MSP_INAV_FONTS_ENABLED
    bool _blink_on;
};
#endif

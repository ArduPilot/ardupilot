#include <AP_OSD/AP_OSD_Backend.h>
#include <AP_MSP/AP_MSP.h>

class AP_OSD_MSP : public AP_OSD_Backend
{
    using AP_OSD_Backend::AP_OSD_Backend;
public:
    static AP_OSD_Backend *probe(AP_OSD &osd);

    //initilize display port and underlying hardware
    bool init() override;

    //draw given text to framebuffer
    void write(uint8_t x, uint8_t y, const char* text) override {};

    //flush framebuffer to screen
    void flush() override {};

    //clear framebuffer
    void clear() override {};

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
        return AP_OSD::osd_types::OSD_MSP;
    }
private:
    void setup_defaults(void);
};

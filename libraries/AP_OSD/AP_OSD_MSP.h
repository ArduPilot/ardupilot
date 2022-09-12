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

    // MSP OSD does not require the screen to be drawn by the OSD thread
    bool is_draw_required() const override {return false;};

private:
    void setup_defaults(void);
};

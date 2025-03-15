#include "AP_DAC_config.h"

#if AP_DAC_ENABLED
#include "AP_DAC_Backend.h"

class AP_DAC_MCP40D1x : public AP_DAC_Backend
{
public:
    using AP_DAC_Backend::AP_DAC_Backend;

    virtual ~AP_DAC_MCP40D1x() {}

    void init(void) override;

    void update(void) override;

    // set voltage for a channel
    bool set_voltage(uint8_t chan, float v) override;

private:
    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    bool register_read(uint8_t reg, uint8_t &val);
    bool register_write(uint8_t reg, uint8_t val);

    bool configured[4];
};

#endif // AP_DAC_ENABLED

#pragma once

#include "AP_HAL_Linux.h"
#include "AP_HAL/RCOutput.h"
#include <tuple>
#include <stdint.h>

namespace Linux {


class RCOutput_Multi : public AP_HAL::RCOutput {
public:
    RCOutput_Multi(const std::tuple<AP_HAL::RCOutput&,int> * output_groups, unsigned n_output_groups) : out_groups(output_groups), n_out_groups(n_output_groups)
    {
    }
    ~RCOutput_Multi();
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;

    void     cork() override;
    void     push() override;

    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;

    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    bool     force_safety_on() override;
    void     force_safety_off() override;
    bool     supports_gpio() override { return false; };

private:
    const std::tuple<AP_HAL::RCOutput&,int> * out_groups;
    const unsigned n_out_groups;
    bool resolve_channel(uint8_t ch, unsigned & gid, unsigned &cid);
};

}
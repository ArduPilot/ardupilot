#pragma once

#include "AP_HAL_Linux.h"
#include "RCOutput_Sysfs.h"
#include "RCOutput_Bebop.h"

namespace Linux {

class RCOutput_Disco : public AP_HAL::RCOutput {
public:
    RCOutput_Disco(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    ~RCOutput_Disco() {}

    static RCOutput_Bebop *from(AP_HAL::RCOutput *rcoutput)
    {
        // this is used by AP_BattMonitor_Bebop to get obs data. We
        // need to return the Bebop output, not ourselves
        RCOutput_Disco *d = static_cast<RCOutput_Disco *>(rcoutput);
        return static_cast<RCOutput_Bebop *>(&d->bebop_out);
    }

    void init() override;
    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void enable_ch(uint8_t ch) override;
    void disable_ch(uint8_t ch) override;
    void write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void read(uint16_t *period_us, uint8_t len) override;
    void cork() override;
    void push() override;
    void set_esc_scaling(uint16_t min_pwm, uint16_t max_pwm) override;

private:
    // Disco RC output combines methods from Sysfs and Bebop
    RCOutput_Bebop bebop_out;
    RCOutput_Sysfs sysfs_out{0, 1, 6};

    /*
      use a table to provide the mapping to channel numbers in each
      backend
     */
    typedef struct {
        RCOutput &output;
        uint8_t channel;
    } output_table_t;
    const output_table_t output_table[7] = {
        { sysfs_out, 3 },
        { sysfs_out, 2 },
        { bebop_out, 0 },
        { sysfs_out, 4 },
        { sysfs_out, 1 },
        { sysfs_out, 5 },
        { sysfs_out, 0 },
    };
};

}

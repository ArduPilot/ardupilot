#pragma once

#include "AP_HAL_Embox.h"

namespace Embox {
    class RCInput : public AP_HAL::RCInput {
    public:
        static RCInput* from(AP_HAL::RCInput* rcinput) {
            return static_cast<RCInput*>(rcinput);
        }
        RCInput();
        void init() override;
        bool new_input() override;
        uint8_t num_channels() override;
        uint16_t read(uint8_t ch) override;
        uint8_t read(uint16_t* periods, uint8_t len) override;
        virtual const char* protocol() const override {
            return "Embox";
        }

        // default empty _timer_tick, this is overridden by board
        // specific implementations
        virtual void _timer_tick() {
        }
    };
} // namespace Embox
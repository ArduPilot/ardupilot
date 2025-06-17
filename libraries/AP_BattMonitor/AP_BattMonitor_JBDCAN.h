    #pragma once

    #include <AP_BattMonitor/AP_BattMonitor_Backend.h>

    class AP_BattMonitor_JBDCAN : public AP_BattMonitor_Backend {
    public:
        // ✅ Khai báo constructor đúng với phần .cpp
        AP_BattMonitor_JBDCAN(AP_BattMonitor &mon,
                            AP_BattMonitor::BattMonitor_State &mon_state,
                            AP_BattMonitor_Params &params);

        void read() override;
        bool has_current() const override {
        return true;
    }

    private:
        static const struct AP_Param::GroupInfo var_info[];
    };

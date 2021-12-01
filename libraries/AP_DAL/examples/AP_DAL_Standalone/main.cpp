//
// Ensure that AP_NavEKF libraries can be compiled when not linked to
// anything except the DAL.
//

#include <AP_DAL/AP_DAL.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Logger/AP_Logger.h>

void AP_Param::setup_object_defaults(void const*, AP_Param::GroupInfo const*) {}

int AP_HAL::Util::vsnprintf(char*, size_t, char const*, va_list) { return -1; }

void *nologger = nullptr;
AP_Logger &AP::logger() {
    return *((AP_Logger*)nologger);  // this is not usually a good idea...
}
void AP_Logger::WriteBlock(void const*, unsigned short) {}

class AP_HAL_DAL_Standalone : public AP_HAL::HAL {
public:
    AP_HAL_DAL_Standalone() :
        AP_HAL::HAL(
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr
            ) {}
    void run(int argc, char* const argv[], Callbacks* callbacks) const override {}
    void setup() { }
    void loop() { }
};

AP_HAL_DAL_Standalone _hal;
const AP_HAL::HAL &hal = _hal;

NavEKF2 navekf2;
NavEKF3 navekf3;

int main(int argc, const char *argv[])
{
    navekf2.InitialiseFilter();
    navekf3.InitialiseFilter();
    navekf2.UpdateFilter();
    navekf3.UpdateFilter();
    return navekf2.healthy() && navekf3.healthy()?0:1;
}

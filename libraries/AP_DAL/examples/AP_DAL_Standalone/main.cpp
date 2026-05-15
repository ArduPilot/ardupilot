//
// Ensure that AP_NavEKF libraries can be compiled when not linked to
// anything except the DAL.
//
#include <AP_DAL/AP_DAL.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

/* 1. GCS STUBS 
   This part handles the "unknown type name 'GCS'" error. 
   If GCS is disabled in the build (--no-gcs), we define a tiny 
   fake class so the EKF doesn't crash the compiler.
*/
#if !HAL_GCS_ENABLED
class GCS {
public:
    void send_text(MAV_SEVERITY severity, const char *fmt, ...) {}
};
GCS &gcs() {
    static GCS _gcs_dummy;
    return _gcs_dummy;
}
#else
GCS &gcs() {
    static GCS *dummy = nullptr;
    return *dummy;
}
void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...) {}
#endif

/* 2. OTHER SYSTEM STUBS 
   Necessary ArduPilot boilerplate to make the EKF run in a sandbox.
*/
void AP_Param::setup_object_defaults(void const*, AP_Param::GroupInfo const*) {}
template<typename T, ap_var_type PT>
void AP_ParamTBase<T, PT>::set_and_default(const T &v) {}
template class AP_ParamTBase<int8_t, AP_PARAM_INT8>;

int AP_HAL::Util::vsnprintf(char*, size_t, char const*, va_list) { return -1; }

void *nologger = nullptr;
AP_Logger &AP::logger() {
    return *((AP_Logger*)nologger);
}
void AP_Logger::WriteBlock(void const*, unsigned short) {}

/* 3. MINIMAL HAL 
   The most basic hardware abstraction layer possible.
*/
class AP_HAL_DAL_Standalone : public AP_HAL::HAL {
public:
    AP_HAL_DAL_Standalone() :
        AP_HAL::HAL(nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
                    nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
                    nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,nullptr,
                    nullptr,nullptr) {}
    void run(int argc, char* const argv[], Callbacks* callbacks) const override {}
};

AP_HAL_DAL_Standalone _hal;
const AP_HAL::HAL &hal = _hal;

/* 4. EXECUTION 
*/
NavEKF2 navekf2;
NavEKF3 navekf3;

int main(int argc, const char *argv[])
{
    navekf2.InitialiseFilter();
    navekf3.InitialiseFilter();
    navekf2.UpdateFilter();
    navekf3.UpdateFilter();
    
    return navekf2.healthy() && navekf3.healthy() ? 0 : 1;
}
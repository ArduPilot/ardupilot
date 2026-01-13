//
// Unit tests for the AP_Common code
//

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_SerialManager/AP_SerialManager.h>

void setup();
void loop();
void test_print(void);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_SerialManager _serialmanager;
GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};


/*
 *  euler angle tests
 */
void setup(void)
{
    ::printf("DBGprint tests\n\n");
    gcs().init();
    gcs().setup_console();

    hal.scheduler->delay(1000);
}

void loop(void)
{
    if (AP_HAL::millis() <= 4000) {
        DBG_CONSOLE(500, 4, "console");
        DBG_PRINTF(500, 4, "printf");
        DBG_GCS(500, 4, "gcs %d\n", (int)AP_HAL::millis());

        static bool value = false;
        if ((AP_HAL::millis() % 1000) == 0) {
            value = !value;
        }
        DBG_WCONSOLE(value, 0, "watch, value: %d", value);
        // logger not instantiated
        DBG_LOG(200, "DBG", "Qf", "TimeUS,Value", AP_HAL::micros(), (float) value);
    }

    // Delay 1 mS for 1 KHz loop rate
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();

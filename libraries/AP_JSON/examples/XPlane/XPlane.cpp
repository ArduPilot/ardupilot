//
// tests for the AP_JSON parser
//

#include <AP_HAL/AP_HAL.h>
#include <AP_JSON/AP_JSON.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static uint32_t test_count;
static uint32_t label_count;

static bool test_xplane(void)
{
    const uint32_t m1 = hal.util->available_memory();
    auto *obj = AP_JSON::load_json("@ROMFS/models/xplane_plane.json");
    if (obj == nullptr) {
        ::printf("Failed to load or parse json\n");
        return false;
    }
    const uint32_t m2 = hal.util->available_memory();
    ::printf("Used %u bytes\n", unsigned(m1-m2));

    const AP_JSON::value::object& o = obj->get<AP_JSON::value::object>();
    for (AP_JSON::value::object::const_iterator i = o.begin();
         i != o.end();
         ++i) {
        const char *label = i->first.c_str();
        ::printf("Label: %s\n", label);
        label_count++;
    }
    delete obj;
    return label_count > 10;
}

/*
 *  euler angle tests
 */
void setup(void)
{
    hal.console->printf("AP_JSON tests\n");
}

void loop(void)
{
    ::printf("Memory: %u\n", (unsigned)hal.util->available_memory());
    if (test_xplane()) {
        test_count++;
    } else {
        ::printf("Test FAILED\n");
        exit(1);
    }
    if (test_count > 10) {
        ::printf("Test PASSED\n");
        exit(0);
    }
}

AP_HAL_MAIN();

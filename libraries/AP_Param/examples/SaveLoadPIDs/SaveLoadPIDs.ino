#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AC_PID.h>
#include <AC_HELI_PID.h>


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
class Parameters {
public:
    static const uint16_t        k_format_version = 130;

    enum {
        k_param_format_version = 0,
        
        k_param_pid1,
        k_param_pid2,
    };
    AP_Int16        format_version;

    AC_PID pid1;
    AC_PID pid2;
};
Parameters g;
#define GSCALAR(v, name, def) { g.v.vtype, name, Parameters::k_param_ ## v, &g.v, {def_value : def} }
#define ASCALAR(v, name, def) { aparm.v.vtype, name, Parameters::k_param_ ## v, &aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, &v, {group_info : class::var_info} }


// default PID values
#define TEST_P 1.0
#define TEST_I 0.01
#define TEST_D 0.2
#define TEST_IMAX 10

// setup (unfortunately must be done here as we cannot create a global AC_PID object)
AC_PID pid1(9, 9, 9, 9 * 100);
AC_PID pid2(3, 3, 3, 3 * 100);
AC_PID pids[2] = {pid1, pid2};

const AP_Param::Info var_info[] PROGMEM = {
  GSCALAR(format_version, "SYSID_SW_MREV",   0),
  
  GOBJECTN(pids[0], pid1, "PID1", AC_PID),
  GOBJECTN(pids[1], pid2, "PID2", AC_PID),
  
  AP_VAREND
};

// setup the var_info table
AP_Param param_loader(var_info);

    
// setup function
void setup() {
    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad var table\n");
    }
  
    hal.console->printf("PRESAVE P %f  I %f  D %f  imax %f\n", (float)pids[0].kP(), (float)pids[0].kI(), (float)pids[0].kD(), (float)pids[0].imax());
    pids[0].save_gains();
    pids[1].save_gains();
    hal.scheduler->delay(1000);
    
    pids[0].kP(TEST_P);
    pids[0].kI(TEST_I);
    pids[0].kD(TEST_D);
    pids[0].imax(TEST_IMAX); 

    hal.console->printf("PRELOAD P %f  I %f  D %f  imax %f\n", (float)pids[0].kP(), (float)pids[0].kI(), (float)pids[0].kD(), (float)pids[0].imax());
    pids[0].load_gains();
    
    if (!g.format_version.load() || 
        g.format_version != Parameters::k_format_version) {

        // erase all parameters
        hal.console->printf("Firmware change: erasing EEPROM...\n");
        AP_Param::erase_all();

        // save the current format version
        g.format_version.set_and_save(Parameters::k_format_version);
        hal.console->printf("done\n");
    } else {
        uint32_t before = hal.scheduler->micros();
        // Load all auto-loaded EEPROM variables
        AP_Param::load_all();
        hal.console->printf("load_all took %d us\n", hal.scheduler->micros() - before);
    }
    
    hal.console->printf("g.format_version :%d\n", g.format_version);
}

// main loop
void loop()
{
    // display PID gains
    hal.console->printf("P %f  I %f  D %f  imax %f\n", (float)pids[0].kP(), (float)pids[0].kI(), (float)pids[0].kD(), (float)pids[0].imax());
    hal.scheduler->delay(50);
}

AP_HAL_MAIN();


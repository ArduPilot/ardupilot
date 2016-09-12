/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <fenv.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_Declination/AP_Declination.h>
#include <Filter/Filter.h>
#include <AP_Buffer/AP_Buffer.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Notify/AP_Notify.h>
#include <DataFlash/DataFlash.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>
#include <AP_HAL/utility/getopt_cpp.h>
#include <AP_SerialManager/AP_SerialManager.h>

class ReplayVehicle {
public:
    void setup();
    void load_parameters(void);

    AP_InertialSensor ins;
    AP_Baro barometer;
    AP_GPS gps;
    Compass compass;
    AP_SerialManager serial_manager;
    RangeFinder rng {serial_manager};
    NavEKF EKF{&ahrs, barometer, rng};
    NavEKF2 EKF2{&ahrs, barometer, rng};
    AP_AHRS_NavEKF ahrs {ins, barometer, gps, rng, EKF, EKF2};
    AP_InertialNav_NavEKF inertial_nav{ahrs};
    AP_Vehicle::FixedWing aparm;
    AP_Airspeed airspeed;
    DataFlash_Class dataflash{"Replay v0.1"};

private:
    Parameters g;

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
};


class Replay : public AP_HAL::HAL::Callbacks {
public:
    Replay(ReplayVehicle &vehicle) :
        filename("log.bin"),
        _vehicle(vehicle) { }

    // HAL::Callbacks implementation.
    void setup() override;
    void loop() override;

    void flush_dataflash(void);

    bool check_solution = false;
    const char *log_filename = NULL;
    bool generate_fpe = true;

    /*
      information about a log from find_log_info
     */
    struct log_information {
        uint16_t update_rate;
        bool have_imu2:1;
        bool have_imt:1;
        bool have_imt2:1;
    } log_info {};

    // return true if a user parameter of name is set
    bool check_user_param(const char *name);
    
private:
    const char *filename;
    ReplayVehicle &_vehicle;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL sitl;
#endif

    LogReader logreader{_vehicle.ahrs, _vehicle.ins, _vehicle.barometer, _vehicle.compass, _vehicle.gps, _vehicle.airspeed, _vehicle.dataflash, nottypes};

    FILE *plotf;
    FILE *plotf2;
    FILE *ekf1f;
    FILE *ekf2f;
    FILE *ekf3f;
    FILE *ekf4f;

    bool done_parameters;
    bool done_baro_init;
    bool done_home_init;
    int32_t arm_time_ms = -1;
    bool ahrs_healthy;
    bool use_imt = true;
    bool check_generate = false;
    float tolerance_euler = 3;
    float tolerance_pos = 2;
    float tolerance_vel = 2;
    const char **nottypes = NULL;
    uint16_t downsample = 0;
    bool logmatch = false;
    uint32_t output_counter = 0;

    struct {
        float max_roll_error;
        float max_pitch_error;
        float max_yaw_error;
        float max_pos_error;
        float max_alt_error;
        float max_vel_error;
    } check_result {};

    void _parse_command_line(uint8_t argc, char * const argv[]);

    struct user_parameter {
        struct user_parameter *next;
        char name[17];
        float value;
    } *user_parameters;

    void set_ins_update_rate(uint16_t update_rate);
    void inhibit_gyro_cal();

    void usage(void);
    void set_user_parameters(void);
    void read_sensors(const char *type);
    void write_ekf_logs(void);
    void log_check_generate();
    void log_check_solution();
    bool show_error(const char *text, float max_error, float tolerance);
    void report_checks();
    bool find_log_info(struct log_information &info);
    const char **parse_list_from_string(const char *str);
    bool parse_param_line(char *line, char **vname, float &value);
    void load_param_file(const char *filename);
    void set_signal_handlers(void);
};

enum {
    LOG_CHEK_MSG=100
};

/*
  Replay specific log structures
 */
struct PACKED log_Chek {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll;
    int16_t pitch;
    uint16_t yaw;
    int32_t lat;
    int32_t lng;
    float alt;
    float vnorth;
    float veast;
    float vdown;
};


extern Replay replay;

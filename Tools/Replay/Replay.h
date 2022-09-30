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

#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Vehicle/AP_FixedWing.h>

#include "LogReader.h"

struct user_parameter {
    struct user_parameter *next;
    char name[17];
    float value;
};

extern user_parameter *user_parameters;
extern bool replay_force_ekf2;
extern bool replay_force_ekf3;

class ReplayVehicle : public AP_Vehicle {
public:
    friend class Replay;

    ReplayVehicle() { unused.set(-1); }
    // HAL::Callbacks implementation.
    void load_parameters(void) override;
    void get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                             uint8_t &task_count,
                             uint32_t &log_bit) override {
        tasks = nullptr;
        task_count = 0;
        log_bit = 0;
    };

    virtual bool set_mode(const uint8_t new_mode, const ModeReason reason) override { return true; }
    virtual uint8_t get_mode() const override { return 0; }

    AP_FixedWing aparm;

    AP_Int32 unused; // logging is magic for Replay; this is unused
    struct LogStructure log_structure[256] = {
    };
    AP_Logger logger{unused};

    NavEKF2 ekf2;
    NavEKF3 ekf3;

protected:

    void init_ardupilot() override;

private:
    Parameters g;

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
};

class Replay : public AP_HAL::HAL::Callbacks {

public:
    Replay(ReplayVehicle &vehicle) :
        _vehicle(vehicle) { }

    void setup() override;
    void loop() override;

    // return true if a user parameter of name is set
    bool check_user_param(const char *name);
    
private:
    const char *filename;
    ReplayVehicle &_vehicle;

    LogReader reader{_vehicle.log_structure, _vehicle.ekf2, _vehicle.ekf3};

    void _parse_command_line(uint8_t argc, char * const argv[]);

    void set_user_parameters(void);
    bool parse_param_line(char *line, char **vname, float &value);
    void load_param_file(const char *filename);
    void usage();
};

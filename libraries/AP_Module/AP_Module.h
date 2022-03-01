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

/*
  support for external modules

  ******************************************************************
  PLEASE NOTE: module hooks are called synchronously from
  ArduPilot. They must not block or make any IO calls. If anything
  could take more than a few 10s of microseconds then you must defer
  it to another thread. Modules are responsible for their own thread
  handling
  ******************************************************************

 */
#pragma once

#if AP_MODULE_SUPPORTED

#include <AP_AHRS/AP_AHRS.h>

#ifndef AP_MODULE_DEFAULT_DIRECTORY
#define AP_MODULE_DEFAULT_DIRECTORY "/usr/lib/ardupilot/modules"
#endif

class AP_Module {
public:

    // initialise AP_Module, looking for shared libraries in the given module path
    static void init(const char *module_path);
    
    // call any setup_start hooks
    static void call_hook_setup_start(void);

    // call any setup_complete hooks
    static void call_hook_setup_complete(void);
    
    // call any AHRS_update hooks
    static void call_hook_AHRS_update(const AP_AHRS &ahrs);

    // call any gyro_sample hooks
    static void call_hook_gyro_sample(uint8_t instance, float dt, const Vector3f &gyro);

    // call any accel_sample hooks
    static void call_hook_accel_sample(uint8_t instance, float dt, const Vector3f &accel, bool fsync_set);
    
    
private:

    enum ModuleHooks {
        HOOK_SETUP_START    = 0,
        HOOK_SETUP_COMPLETE,
        HOOK_AHRS_UPDATE,
        HOOK_GYRO_SAMPLE,
        HOOK_ACCEL_SAMPLE,
        NUM_HOOKS
    };

    // singly linked list per hook
    struct hook_list {
        struct hook_list *next;
        void *symbol; // from dlsym()
    };

    // currently installed hooks
    static struct hook_list *hooks[NUM_HOOKS];

    // table giving the name of the hooks in the external
    // modules. These are passed to dlsym(). The table order must
    // match the ModuleHooks enum
    static const char *hook_names[NUM_HOOKS];
    
    // scan a module for hooks
    static void module_scan(const char *path);
};

#endif // AP_MODULE_SUPPORTED

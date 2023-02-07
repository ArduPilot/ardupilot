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

#include <AP_HAL/AP_HAL.h>

#if AP_MODULE_SUPPORTED

/*
  support for external modules
 */

#include <stdio.h>
#if defined(HAVE_LIBDL)
#include <dirent.h>
#include <dlfcn.h>
#endif
#include <AP_Module/AP_Module.h>
#include <AP_Module/AP_Module_Structures.h>

struct AP_Module::hook_list *AP_Module::hooks[NUM_HOOKS];

const char *AP_Module::hook_names[AP_Module::NUM_HOOKS] = {
    "ap_hook_setup_start",
    "ap_hook_setup_complete",
    "ap_hook_AHRS_update",
    "ap_hook_gyro_sample",
    "ap_hook_accel_sample",
};

/*
  scan a module for hook symbols
 */
void AP_Module::module_scan(const char *path)
{
#if defined(HAVE_LIBDL)
    void *m = dlopen(path, RTLD_NOW);
    if (m == nullptr) {
        printf("dlopen(%s) -> %s\n", path, dlerror());
        return;
    }
    uint8_t found_hooks = 0;
    for (uint16_t i=0; i<NUM_HOOKS; i++) {
        void *s = dlsym(m, hook_names[i]);
        if (s != nullptr) {
            // found a hook in this module, add it to the list
            struct hook_list *h = new hook_list;
            if (h == nullptr) {
                AP_HAL::panic("Failed to allocate hook for %s", hook_names[i]);
            }
            h->next = hooks[i];
            h->symbol = s;
            hooks[i] = h;
            found_hooks++;
        }
    }
    if (found_hooks == 0) {
        // we don't need this module
        dlclose(m);
    } else {
        printf("AP_Module: Loaded %u hooks from %s\n", (unsigned)found_hooks, path);
    }
#endif
}

/*
  initialise AP_Module, looking for shared libraries in the given module path
*/
void AP_Module::init(const char *module_path)
{
#if AP_MODULE_SUPPORTED
    // scan through module directory looking for *.so
    DIR *d;
    struct dirent *de;
    d = opendir(module_path);
    if (d == nullptr) {
        return;
    }
    while ((de = readdir(d))) {
        const char *extension = strrchr(de->d_name, '.');
        if (extension == nullptr || strcmp(extension, ".so") != 0) {
            continue;
        }
        char *path = nullptr;
        if (asprintf(&path, "%s/%s", module_path, de->d_name) == -1) {
            continue;
        }
        module_scan(path);
        free(path);
    }
    closedir(d);
#endif
}


/*
  call any setup_start hooks
*/
void AP_Module::call_hook_setup_start(void)
{
#if AP_MODULE_SUPPORTED
    uint64_t now = AP_HAL::micros64();
    for (const struct hook_list *h=hooks[HOOK_SETUP_START]; h; h=h->next) {
        ap_hook_setup_start_fn_t fn = reinterpret_cast<ap_hook_setup_start_fn_t>(h->symbol);
        fn(now);
    }
#endif
}

/*
  call any setup_complete hooks
*/
void AP_Module::call_hook_setup_complete(void)
{
#if AP_MODULE_SUPPORTED
    uint64_t now = AP_HAL::micros64();
    for (const struct hook_list *h=hooks[HOOK_SETUP_COMPLETE]; h; h=h->next) {
        ap_hook_setup_complete_fn_t fn = reinterpret_cast<ap_hook_setup_complete_fn_t>(h->symbol);
        fn(now);
    }
#endif
}

/*
  call any AHRS_update hooks
*/
void AP_Module::call_hook_AHRS_update(const AP_AHRS &ahrs)
{
#if AP_MODULE_SUPPORTED
    if (hooks[HOOK_AHRS_UPDATE] == nullptr) {
        // avoid filling in AHRS_state
        return;
    }

    /*
      construct AHRS_state structure
     */
    struct AHRS_state state {};
    state.structure_version = AHRS_state_version;
    state.time_us = AP_HAL::micros64();

    if (!ahrs.initialised()) {
        state.status = AHRS_STATUS_INITIALISING;
    } else if (ahrs.healthy()) {
        state.status = AHRS_STATUS_HEALTHY;
    } else {
        state.status = AHRS_STATUS_UNHEALTHY;
    }

    Quaternion q;
    q.from_rotation_matrix(ahrs.get_rotation_body_to_ned());
    state.quat[0] = q[0];
    state.quat[1] = q[1];
    state.quat[2] = q[2];
    state.quat[3] = q[3];

    state.eulers[0] = ahrs.roll;
    state.eulers[1] = ahrs.pitch;
    state.eulers[2] = ahrs.yaw;

    Location loc;
    if (ahrs.get_origin(loc)) {
        state.origin.initialised = true;
        state.origin.latitude = loc.lat;
        state.origin.longitude = loc.lng;
        state.origin.altitude = loc.alt*0.01f;
    }

    if (ahrs.get_location(loc)) {
        state.position.available = true;
        state.position.latitude = loc.lat;
        state.position.longitude = loc.lng;
        state.position.altitude = loc.alt*0.01f;
    }
    
    Vector3f pos;
    if (ahrs.get_relative_position_NED_origin(pos)) {
        state.relative_position[0] = pos[0];
        state.relative_position[1] = pos[1];
        state.relative_position[2] = pos[2];
    }

    const Vector3f &gyro = ahrs.get_gyro();
    state.gyro_rate[0] = gyro[0];
    state.gyro_rate[1] = gyro[1];
    state.gyro_rate[2] = gyro[2];

    const Vector3f &accel_ef = ahrs.get_accel_ef();
    state.accel_ef[0] = accel_ef[0];
    state.accel_ef[1] = accel_ef[0];
    state.accel_ef[2] = accel_ef[0];

    state.primary_accel = ahrs.get_primary_accel_index();
    state.primary_gyro = ahrs.get_primary_gyro_index();

    const Vector3f &gyro_bias = ahrs.get_gyro_drift();
    state.gyro_bias[0] = gyro_bias[0];
    state.gyro_bias[1] = gyro_bias[1];
    state.gyro_bias[2] = gyro_bias[2];

    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        state.velocity_ned[0] = vel.x;
        state.velocity_ned[1] = vel.y;
        state.velocity_ned[2] = vel.z;
    }
    
    for (const struct hook_list *h=hooks[HOOK_AHRS_UPDATE]; h; h=h->next) {
        ap_hook_AHRS_update_fn_t fn = reinterpret_cast<ap_hook_AHRS_update_fn_t>(h->symbol);
        fn(&state);
    }
#endif
}


/*
  call any gyro_sample hooks
*/
void AP_Module::call_hook_gyro_sample(uint8_t instance, float dt, const Vector3f &gyro)
{
#if AP_MODULE_SUPPORTED
    if (hooks[HOOK_GYRO_SAMPLE] == nullptr) {
        // avoid filling in struct
        return;
    }

    /*
      construct gyro_sample structure
     */
    struct gyro_sample state {};
    state.structure_version = gyro_sample_version;
    state.time_us = AP_HAL::micros64();
    state.instance = instance;
    state.delta_time = dt;
    state.gyro[0] = gyro[0];
    state.gyro[1] = gyro[1];
    state.gyro[2] = gyro[2];

    for (const struct hook_list *h=hooks[HOOK_GYRO_SAMPLE]; h; h=h->next) {
        ap_hook_gyro_sample_fn_t fn = reinterpret_cast<ap_hook_gyro_sample_fn_t>(h->symbol);
        fn(&state);
    }
#endif
}

/*
  call any accel_sample hooks
*/
void AP_Module::call_hook_accel_sample(uint8_t instance, float dt, const Vector3f &accel, bool fsync_set)
{
#if AP_MODULE_SUPPORTED
    if (hooks[HOOK_ACCEL_SAMPLE] == nullptr) {
        // avoid filling in struct
        return;
    }

    /*
      construct accel_sample structure
     */
    struct accel_sample state {};
    state.structure_version = accel_sample_version;
    state.time_us = AP_HAL::micros64();
    state.instance = instance;
    state.delta_time = dt;
    state.accel[0] = accel[0];
    state.accel[1] = accel[1];
    state.accel[2] = accel[2];
    state.fsync_set = fsync_set;

    for (const struct hook_list *h=hooks[HOOK_ACCEL_SAMPLE]; h; h=h->next) {
        ap_hook_accel_sample_fn_t fn = reinterpret_cast<ap_hook_accel_sample_fn_t>(h->symbol);
        fn(&state);
    }
#endif
}

#endif // AP_MODULE_SUPPORTED

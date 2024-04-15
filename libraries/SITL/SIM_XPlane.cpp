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
  simulator connector for XPlane
*/

#include "SIM_XPlane.h"

#if HAL_SIM_XPLANE_ENABLED

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

// ignore cast errors in this case to keep complexity down
#pragma GCC diagnostic ignored "-Wcast-align"

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_Heli)
#define XPLANE_JSON "xplane_heli.json"
#else
#define XPLANE_JSON "xplane_plane.json"
#endif

// DATA@ frame types. Thanks to TauLabs xplanesimulator.h
// (which strangely enough acknowledges APM as a source!)
enum {
    FramRate            = 0,
    Times               = 1,
    SimStats            = 2,
    Speed               = 3,
    Gload               = 4,
    AtmosphereWeather   = 5,
    AtmosphereAircraft  = 6,
    SystemPressures     = 7,
    Joystick1           = 8,
    Joystick2           = 9,
    ArtStab             = 10,
    FlightCon           = 11,
    WingSweep           = 12,
    Trim                = 13,
    Brakes              = 14,
    AngularMoments      = 15,
    AngularVelocities   = 16,
    PitchRollHeading    = 17,
    AoA                 = 18,
    MagCompass          = 19,
    LatLonAlt           = 20,
    LocVelDistTraveled  = 21,
    ThrottleCommand     = 25,
    CarbHeat            = 30,
    EngineRPM           = 37,
    PropRPM             = 38,
    PropPitch           = 39,
    Generator           = 58,
    JoystickRaw         = 136,
};

enum RREF {
    RREF_VERSION = 1,
};

static const uint8_t required_data[] {
        Times, LatLonAlt, Speed, PitchRollHeading,
        LocVelDistTraveled, AngularVelocities, Gload,
        Trim,
        PropPitch, EngineRPM, PropRPM,
        JoystickRaw };

using namespace SITL;

XPlane::XPlane(const char *frame_str) :
    Aircraft(frame_str)
{
    use_time_sync = false;
    const char *colon = strchr(frame_str, ':');
    if (colon) {
        xplane_ip = colon+1;
    }

    socket_in.bind("0.0.0.0", bind_port);
    printf("Waiting for XPlane data on UDP port %u and sending to port %u\n",
           (unsigned)bind_port, (unsigned)xplane_port);

    // XPlane sensor data is not good enough for EKF. Use fake EKF by default
    AP_Param::set_default_by_name("AHRS_EKF_TYPE", 10);
    AP_Param::set_default_by_name("GPS1_TYPE", 100);
    AP_Param::set_default_by_name("INS_GYR_CAL", 0);

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // default flaps to channel 5
    AP_Param::set_default_by_name("SERVO5_FUNCTION", 3);
    AP_Param::set_default_by_name("SERVO5_MIN", 1000);
    AP_Param::set_default_by_name("SERVO5_MAX", 2000);
#endif

    if (!load_dref_map(XPLANE_JSON)) {
        AP_HAL::panic("%s failed to load\n", XPLANE_JSON);
    }
}

/*
  add one DRef to list
 */
void XPlane::add_dref(const char *name, DRefType type, const AP_JSON::value &dref)
{
    struct DRef *d = new struct DRef;
    if (d == nullptr) {
        AP_HAL::panic("out of memory for DRef %s", name);
    }
    d->name = strdup(name);
    d->type = type;
    if (d->name == nullptr) {
        AP_HAL::panic("out of memory for DRef %s", name);
    }
    if (d->type == DRefType::FIXED) {
        d->fixed_value = dref.get("value").get<double>();
    } else {
        d->range = dref.get("range").get<double>();
        d->channel = dref.get("channel").get<double>();
    }
    // add to linked list
    d->next = drefs;
    drefs = d;
}

/*
  add one joystick axis to list
 */
void XPlane::add_joyinput(const char *label, JoyType type, const AP_JSON::value &d)
{
    if (strncmp(label, "axis", 4) == 0) {
        struct JoyInput *j = new struct JoyInput;
        if (j == nullptr) {
            AP_HAL::panic("out of memory for JoyInput %s", label);
        }
        j->axis = atoi(label+4);
        j->type = JoyType::AXIS;
        j->channel = d.get("channel").get<double>();
        j->input_min = d.get("input_min").get<double>();
        j->input_max = d.get("input_max").get<double>();
        j->next = joyinputs;
        joyinputs = j;
    }
    if (strncmp(label, "button", 6) == 0) {
        struct JoyInput *j = new struct JoyInput;
        if (j == nullptr) {
            AP_HAL::panic("out of memory for JoyInput %s", label);
        }
        j->type = JoyType::BUTTON;
        j->channel = d.get("channel").get<double>();
        j->mask = d.get("mask").get<double>();
        j->next = joyinputs;
        joyinputs = j;
    }
}

/*
  handle a setting
 */
void XPlane::handle_setting(const AP_JSON::value &d)
{
    if (d.contains("debug")) {
        dref_debug = d.get("debug").get<double>();
    }
}


/*
  load mapping of channels to datarefs from a json file
 */
bool XPlane::load_dref_map(const char *map_json)
{
    char *fname = nullptr;
    if (AP::FS().stat(map_json, &map_st) == 0) {
        fname = strdup(map_json);
    } else {
        IGNORE_RETURN(asprintf(&fname, "@ROMFS/models/%s", map_json));
        if (AP::FS().stat(fname, &map_st) != 0) {
            return false;
        }
    }
    if (fname == nullptr) {
        return false;
    }
    AP_JSON::value *obj = AP_JSON::load_json(fname);
    if (obj == nullptr) {
        return false;
    }

    free(map_filename);
    map_filename = fname;

    // free old drefs
    while (drefs) {
        auto *d = drefs->next;
        free(drefs->name);
        delete drefs;
        drefs = d;
    }

    // free old joystick
    while (joyinputs) {
        auto *j = joyinputs->next;
        delete joyinputs;
        joyinputs = j;
    }
    
    uint32_t count = 0;
    // obtain a const reference to the map, and print the contents
    const AP_JSON::value::object& o = obj->get<AP_JSON::value::object>();
    for (AP_JSON::value::object::const_iterator i = o.begin();
         i != o.end();
         ++i) {
        const char *label = i->first.c_str();
        const auto &d = i->second;
        if (strchr(label, '/') != nullptr) {
            const char *type_s = d.get("type").to_str().c_str();
            if (strcmp(type_s, "angle") == 0) {
                add_dref(label, DRefType::ANGLE, d);
            } else if (strcmp(type_s, "range") == 0) {
                add_dref(label, DRefType::RANGE, d);
            } else if (strcmp(type_s, "fixed") == 0) {
                add_dref(label, DRefType::FIXED, d);
            } else {
                ::printf("Invalid dref type %s for %s in %s", type_s, label, map_filename);
            }
        } else if (strcmp(label, "settings") == 0) {
            handle_setting(d);
        } else if (strncmp(label, "axis", 4) == 0) {
            add_joyinput(label, JoyType::AXIS, d);
        } else if (strncmp(label, "button", 6) == 0) {
            add_joyinput(label, JoyType::BUTTON, d);
        } else {
            ::printf("Invalid json type %s in %s", label, map_json);
            continue;
        }
        count++;
    }
    delete obj;

    ::printf("Loaded %u DRefs from %s\n", unsigned(count), map_filename);
    return true;
}

/*
  load mapping of channels to datarefs from a json file
 */
void XPlane::check_reload_dref(void)
{
    if (!hal.util->get_soft_armed()) {
        struct stat st;
        if (AP::FS().stat(map_filename, &st) == 0 && st.st_mtime != map_st.st_mtime) {
            load_dref_map(map_filename);
        }
    }
}

int8_t XPlane::find_data_index(uint8_t code)
{
    for (uint8_t i = 0; i<ARRAY_SIZE(required_data); i++) {
        if (required_data[i] == code) {
            return i;
        }
    }
    return -1;
}

/*
 change what data is requested from XPlane. This saves the user from
 having to setup the data screen correctly
 */
void XPlane::select_data(void)
{
    const uint64_t all_mask = (1U<<ARRAY_SIZE(required_data))-1;
    if ((seen_mask & all_mask) == all_mask) {
        // got it all
        return;
    }
    struct PACKED {
        uint8_t  marker[5] { 'D', 'S', 'E', 'L', '0' };
        uint32_t data[8] {};
    } dsel;
    uint8_t count = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(required_data); i++) {
        if (seen_mask & (1U<<i)) {
            // got this one
            continue;
        }
        dsel.data[count++] = required_data[i];
    }
    if (count != 0) {
        socket_out.send(&dsel, sizeof(dsel));
        printf("Selecting %u data types\n", (unsigned)count);
    }
}

void XPlane::deselect_code(uint8_t code)
{
    struct PACKED {
        uint8_t  marker[5] { 'U', 'S', 'E', 'L', '0' };
        uint32_t data[8] {};
    } usel;
    usel.data[0] = code;
    socket_out.send(&usel, sizeof(usel));
    printf("De-selecting code %u\n", code);
}

/*
  receive data from X-Plane via UDP
  return true if we get a gyro frame
*/
bool XPlane::receive_data(void)
{
    uint8_t pkt[10000];
    uint8_t *p = &pkt[5];
    const uint8_t pkt_len = 36;
    Location loc {};
    Vector3d pos;
    uint32_t wait_time_ms = 1;
    uint32_t now = AP_HAL::millis();
    bool ret = false;

    // if we are about to get another frame from X-Plane then wait longer
    if (xplane_frame_time > wait_time_ms &&
        now+1 >= last_data_time_ms + xplane_frame_time) {
        wait_time_ms = 10;
    }
    ssize_t len = socket_in.recv(pkt, sizeof(pkt), wait_time_ms);
    
    if (len < 5) {
        // bad packet
        goto failed;
    }

    if (memcmp(pkt, "RREF", 4) == 0) {
        handle_rref(pkt, len);
        return false;
    }

    if (memcmp(pkt, "DATA", 4) != 0) {
        // not a data packet we understand
        ::printf("PACKET: %4.4s\n", (const char *)pkt);
        goto failed;
    }
    len -= 5;

    if (len < pkt_len) {
        // bad packet
        goto failed;
    }

    
    if (!connected) {
        // we now know the IP X-Plane is using
        uint16_t port;
        socket_in.last_recv_address(xplane_ip, port);
        socket_out.connect(xplane_ip, xplane_port);
        connected = true;
        printf("Connected to %s:%u\n", xplane_ip, (unsigned)xplane_port);
    }
    
    while (len >= pkt_len) {
        const float *data = (const float *)p;
        uint8_t code = p[0];
        int8_t idx = find_data_index(code);
        if (idx == -1) {
            deselect_code(code);
            len -= pkt_len;
            p += pkt_len;
            continue;
        }
        seen_mask |= (1U<<idx);

        switch (code) {
        case Times: {
            uint64_t tus = data[3] * 1.0e6f;
            if (tus + time_base_us <= time_now_us) {
                uint64_t tdiff = time_now_us - (tus + time_base_us);
                if (tdiff > 1e6f) {
                    printf("X-Plane time reset %lu\n", (unsigned long)tdiff);
                }
                time_base_us = time_now_us - tus;
            }
            uint64_t tnew = time_base_us + tus;
            //uint64_t dt = tnew - time_now_us;
            //printf("dt %u\n", (unsigned)dt);
            time_now_us = tnew;
            break;
        }
            
        case LatLonAlt: {
            loc.lat = data[1] * 1e7;
            loc.lng = data[2] * 1e7;
            loc.alt = data[3] * FEET_TO_METERS * 100.0f;
            const float altitude_above_ground = data[4] * FEET_TO_METERS;
            ground_level = loc.alt * 0.01f - altitude_above_ground;
            break;
        }

        case Speed:
            airspeed = data[2] * KNOTS_TO_METERS_PER_SECOND;
            airspeed_pitot = airspeed;
            break;

        case AoA:
            // ignored
            break;

        case PitchRollHeading: {
            float roll, pitch, yaw;
            pitch = radians(data[1]);
            roll = radians(data[2]);
            yaw = radians(data[3]);
            dcm.from_euler(roll, pitch, yaw);
            break;
        }

        case AtmosphereWeather:
            // ignored
            break;

        case LocVelDistTraveled:
            pos.y = data[1];
            pos.z = -data[2];
            pos.x = -data[3];
            velocity_ef.y = data[4];
            velocity_ef.z = -data[5];
            velocity_ef.x = -data[6];
            break;

        case AngularVelocities:
            if (is_xplane12()) {
                gyro.x = radians(data[1]);
                gyro.y = radians(data[2]);
                gyro.z = radians(data[3]);
            } else {
                // xplane 11
                gyro.x = data[2];
                gyro.y = data[1];
                gyro.z = data[3];
            }
            // we only count gyro data towards data counts
            ret = true;
            break;

        case Gload:
            accel_body.z = -data[5] * GRAVITY_MSS;
            accel_body.x = data[6] * GRAVITY_MSS;
            accel_body.y = data[7] * GRAVITY_MSS;
            break;

        case PropPitch: {
            break;
        }

        case EngineRPM:
            rpm[0] = data[1];
            motor_mask |= 1;
            break;

        case PropRPM:
            rpm[1] = data[1];
            motor_mask |= 2;
            break;
            
        case JoystickRaw: {
            for (auto *j = joyinputs; j; j=j->next) {
                switch (j->type) {
                case JoyType::AXIS: {
                    if (j->axis >= 1 && j->axis <= 6) {
                        float v = (data[j->axis] - j->input_min) / (j->input_max - j->input_min);
                        rcin[j->channel-1] = v;
                        rcin_chan_count = MAX(rcin_chan_count, j->channel);
                    }
                    break;
                }
                case JoyType::BUTTON: {
                    uint32_t m = uint32_t(data[7]) & j->mask;
                    float v = 0;
                    if (m == 0) {
                        v = 0;
                    } else if (1U<<(__builtin_ffs(j->mask)-1) != m) {
                        v = 0.5;
                    } else {
                        v = 1;
                    }
                    rcin[j->channel-1] = v;
                    rcin_chan_count = MAX(rcin_chan_count, j->channel);
                    break;
                }
                }
            }
        }
        }
        len -= pkt_len;
        p += pkt_len;
    }

    // update data selection
    select_data();

    position = pos + position_zero;
    position.xy() += origin.get_distance_NE_double(home);
    update_position();
    time_advance();

    accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;
    
    // the position may slowly deviate due to float accuracy and longitude scaling
    if (loc.get_distance(location) > 4 || abs(loc.alt - location.alt)*0.01f > 2.0f) {
        printf("X-Plane home reset dist=%f alt=%.1f/%.1f\n",
               loc.get_distance(location), loc.alt*0.01f, location.alt*0.01f);
        // reset home location
        position_zero = {-pos.x, -pos.y, -pos.z};
        home.lat = loc.lat;
        home.lng = loc.lng;
        home.alt = loc.alt;
        origin = home;
        position.x = 0;
        position.y = 0;
        position.z = 0;
        update_position();
        time_advance();
    }

    update_mag_field_bf();

    if (now > last_data_time_ms && now - last_data_time_ms < 100) {
        xplane_frame_time = now - last_data_time_ms;
    }
    last_data_time_ms = AP_HAL::millis();

    if (ret) {
        report.data_count++;
        report.frame_count++;
    }
    
    return ret;
        
failed:
    if (AP_HAL::millis() - last_data_time_ms > 200) {
        // don't extrapolate beyond 0.2s
        return false;
    }

    // advance time by 1ms
    frame_time_us = 1000;
    float delta_time = frame_time_us * 1e-6f;

    time_now_us += frame_time_us;

    extrapolate_sensors(delta_time);
    
    update_position();
    time_advance();
    update_mag_field_bf();
    report.frame_count++;
    return false;
}

/*
  receive RREF replies
*/
void XPlane::handle_rref(const uint8_t *pkt, uint32_t len)
{
    const uint8_t *p = &pkt[5];
    const struct PACKED RRefPacket {
        uint32_t code;
        union PACKED {
            float value_f;
            double value_d;
        };
    } *ref = (const struct RRefPacket *)p;
    switch (ref->code) {
    case RREF_VERSION:
        if (xplane_version == 0) {
            ::printf("XPlane version %.0f\n", ref->value_f);
        }
        xplane_version = uint32_t(ref->value_f);
        break;
    }
}


/*
  send DRef data to X-Plane via UDP
*/
void XPlane::send_drefs(const struct sitl_input &input)
{
    for (const auto *d = drefs; d; d=d->next) {
        switch (d->type) {

        case DRefType::ANGLE: {
            float v  = d->range * (input.servos[d->channel-1]-1500)/500.0;
            send_dref(d->name, v);
            break;
        }

        case DRefType::RANGE: {
            float v  = d->range * (input.servos[d->channel-1]-1000)/1000.0;
            send_dref(d->name, v);
            break;
        }

        case DRefType::FIXED: {
            send_dref(d->name, d->fixed_value);
            break;
        }
        }
    }
}


/*
  send DREF to X-Plane via UDP
*/
void XPlane::send_dref(const char *name, float value)
{
    struct PACKED {
        uint8_t  marker[5] { 'D', 'R', 'E', 'F', '0' };
        float value;
        char name[500];
    } d {};
    d.value = value;
    strcpy(d.name, name);
    socket_out.send(&d, sizeof(d));
    if (dref_debug > 0) {
        ::printf("-> %s : %.3f\n", name, value);
    }
}

/*
  request a dref
*/
void XPlane::request_dref(const char *name, uint8_t code, uint32_t rate)
{
    struct PACKED {
        uint8_t  marker[5] { 'R', 'R', 'E', 'F', '0' };
        uint32_t rate_hz;
        uint32_t code;
        char name[400];
    } d {};
    d.rate_hz = rate;
    d.code = code; // given back in responses
    strcpy(d.name, name);
    socket_in.sendto(&d, sizeof(d), xplane_ip, xplane_port);
}

void XPlane::request_drefs(void)
{
    request_dref("sim/version/xplane_internal_version", RREF_VERSION, 1);
}

/*
  update the XPlane simulation by one time step
 */
void XPlane::update(const struct sitl_input &input)
{
    if (receive_data()) {
        send_drefs(input);
    }

    uint32_t now = AP_HAL::millis();
    if (report.last_report_ms == 0) {
        report.last_report_ms = now;
        request_drefs();
    }
    if (now - report.last_report_ms > 5000) {
        float dt = (now - report.last_report_ms) * 1.0e-3f;
        printf("Data rate: %.1f FPS  Frame rate: %.1f FPS\n",
               report.data_count/dt, report.frame_count/dt);
        report.last_report_ms = now;
        report.data_count = 0;
        report.frame_count = 0;
        request_drefs();
    }
    check_reload_dref();
}

#endif  // HAL_SIM_XPLANE_ENABLED

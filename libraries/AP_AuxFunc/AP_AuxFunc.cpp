#include "AP_AuxFunc.h"

#include <GCS_MAVLink/GCS.h>

#include <AC_Avoidance/AC_Avoid.h>
#include <AC_Fence/AC_Fence.h>
#include <AC_Sprayer/AC_Sprayer.h>
#include <AP_ADSB/AP_ADSB.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Avoidance/AP_Avoidance.h>
#include <AP_Camera/AP_Camera.h>
#include <AP_Camera/AP_RunCam.h>
#include <AP_Generator/AP_Generator.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_LandingGear/AP_LandingGear.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_ServoRelayEvents/AP_ServoRelayEvents.h>
#include <AP_VideoTX/AP_VideoTX.h>
#include <AP_VisualOdom/AP_VisualOdom.h>

//
// support for auxillary switches:
//

// init_aux_switch_function - initialize aux functions
bool AP_AuxFunc::init_function(const Function function, const SwitchPos pos)
{
    // init channel options
    switch(function) {
    // the following functions do not need to be initialised:
    case Function::ARMDISARM:
    case Function::CAMERA_TRIGGER:
    case Function::CLEAR_WP:
    case Function::COMPASS_LEARN:
    case Function::DISARM:
    case Function::DO_NOTHING:
    case Function::LANDING_GEAR:
    case Function::LOST_VEHICLE_SOUND:
    case Function::RELAY:
    case Function::RELAY2:
    case Function::RELAY3:
    case Function::RELAY4:
    case Function::RELAY5:
    case Function::RELAY6:
    case Function::VISODOM_CALIBRATE:
    case Function::EKF_LANE_SWITCH:
    case Function::EKF_YAW_RESET:
    case Function::GENERATOR: // don't turn generator on or off initially
    case Function::EKF_POS_SOURCE:
    case Function::SCRIPTING_1:
    case Function::SCRIPTING_2:
    case Function::SCRIPTING_3:
    case Function::SCRIPTING_4:
    case Function::SCRIPTING_5:
    case Function::SCRIPTING_6:
    case Function::SCRIPTING_7:
    case Function::SCRIPTING_8:
    case Function::VTX_POWER:
        return true;
    case Function::AVOID_ADSB:
    case Function::AVOID_PROXIMITY:
    case Function::FENCE:
    case Function::GPS_DISABLE:
    case Function::GPS_DISABLE_YAW:
    case Function::GRIPPER:
    case Function::KILL_IMU1:
    case Function::KILL_IMU2:
    case Function::MISSION_RESET:
    case Function::MOTOR_ESTOP:
    case Function::RC_OVERRIDE_ENABLE:
    case Function::RUNCAM_CONTROL:
    case Function::RUNCAM_OSD_CONTROL:
    case Function::SPRAYER:
    case Function::DISABLE_AIRSPEED_USE:
#if HAL_MOUNT_ENABLED
    case Function::RETRACT_MOUNT:
#endif
        do_function(function, pos);
        return true;
    default:
        return false;
    }
    return false;
}

#if !HAL_MINIMIZE_FEATURES

const AP_AuxFunc::LookupTable AP_AuxFunc::lookuptable[] = {
    { Function::SAVE_WP, "SaveWaypoint"},
    { Function::CAMERA_TRIGGER, "CameraTrigger"},
    { Function::RANGEFINDER, "Rangefinder"},
    { Function::FENCE, "Fence"},
    { Function::SPRAYER, "Sprayer"},
    { Function::PARACHUTE_ENABLE, "ParachuteEnable"},
    { Function::PARACHUTE_RELEASE, "ParachuteRelease"},
    { Function::PARACHUTE_3POS, "Parachute3Position"},
    { Function::MISSION_RESET, "MissionReset"},
    { Function::RETRACT_MOUNT, "RetractMount"},
    { Function::RELAY, "Relay1"},
    { Function::LANDING_GEAR, "Landing"},
    { Function::MOTOR_ESTOP, "MotorEStop"},
    { Function::MOTOR_INTERLOCK, "MotorInterlock"},
    { Function::RELAY2, "Relay2"},
    { Function::RELAY3, "Relay3"},
    { Function::RELAY4, "Relay4"},
    { Function::PRECISION_LOITER, "PrecisionLoiter"},
    { Function::AVOID_PROXIMITY, "AvoidProximity"},
    { Function::WINCH_ENABLE, "WinchEnable"},
    { Function::WINCH_CONTROL, "WinchControl"},
    { Function::CLEAR_WP, "ClearWaypoint"},
    { Function::COMPASS_LEARN, "CompassLearn"},
    { Function::SAILBOAT_TACK, "SailboatTack"},
    { Function::GPS_DISABLE, "GPSDisable"},
    { Function::GPS_DISABLE_YAW, "GPSDisableYaw"},
    { Function::DISABLE_AIRSPEED_USE,"DisableAirspeedUse"},
    { Function::RELAY5, "Relay5"},
    { Function::RELAY6, "Relay6"},
    { Function::SAILBOAT_MOTOR_3POS, "SailboatMotor"},
    { Function::SURFACE_TRACKING, "SurfaceTracking"},
    { Function::RUNCAM_CONTROL, "RunCamControl"},
    { Function::RUNCAM_OSD_CONTROL, "RunCamOSDControl"},
    { Function::VISODOM_CALIBRATE, "VisodomCalibrate"},
    { Function::EKF_POS_SOURCE, "EKFPosSource"},
    { Function::CAM_MODE_TOGGLE, "CamModeToggle"},
    { Function::GENERATOR, "Generator"},
    { Function::ARSPD_CALIBRATE, "Calibrate Airspeed"},
};

/* lookup the announcement for switch change */
const char *AP_AuxFunc::string_for_function(Function function)
{
     for (const struct LookupTable entry : lookuptable) {
        if (entry.option == function) {
            return entry.announcement;
        }
     }
     return nullptr;
}

#endif // HAL_MINIMIZE_FEATURES

void AP_AuxFunc::do_function_armdisarm(const SwitchPos pos)
{
    // arm or disarm the vehicle
    switch (pos) {
    case SwitchPos::HIGH:
        AP::arming().arm(AP_Arming::Method::AUXSWITCH, true);
        break;
    case SwitchPos::MIDDLE:
        // nothing
        break;
    case SwitchPos::LOW:
        AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        break;
    }
}

void AP_AuxFunc::do_function_avoid_adsb(const SwitchPos pos)
{
#if HAL_ADSB_ENABLED
    AP_Avoidance *avoidance = AP::ap_avoidance();
    if (avoidance == nullptr) {
        return;
    }
    AP_ADSB *adsb = AP::ADSB();
    if (adsb == nullptr) {
        return;
    }
    if (pos == SwitchPos::HIGH) {
        // try to enable AP_Avoidance
        if (!adsb->enabled() || !adsb->healthy()) {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "ADSB not available");
            return;
        }
        avoidance->enable();
        AP::logger().Write_Event(LogEvent::AVOIDANCE_ADSB_ENABLE);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "ADSB Avoidance Enabled");
        return;
    }

    // disable AP_Avoidance
    avoidance->disable();
    AP::logger().Write_Event(LogEvent::AVOIDANCE_ADSB_DISABLE);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "ADSB Avoidance Disabled");
#endif
}

void AP_AuxFunc::do_function_avoid_proximity(const SwitchPos pos)
{
    AC_Avoid *avoid = AP::ac_avoid();
    if (avoid == nullptr) {
        return;
    }

    switch (pos) {
    case SwitchPos::HIGH:
        avoid->proximity_avoidance_enable(true);
        break;
    case SwitchPos::MIDDLE:
        // nothing
        break;
    case SwitchPos::LOW:
        avoid->proximity_avoidance_enable(false);
        break;
    }
}

void AP_AuxFunc::do_function_camera_trigger(const SwitchPos pos)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return;
    }
    if (pos == SwitchPos::HIGH) {
        camera->take_picture();
    }
}

void AP_AuxFunc::do_function_runcam_control(const SwitchPos pos)
{
#if HAL_RUNCAM_ENABLED
    AP_RunCam *runcam = AP::runcam();
    if (runcam == nullptr) {
        return;
    }

    switch (pos) {
        case SwitchPos::HIGH:
            runcam->start_recording();
            break;
        case SwitchPos::MIDDLE:
            runcam->osd_option();
            break;
        case SwitchPos::LOW:
            runcam->stop_recording();
            break;
    }
#endif
}

void AP_AuxFunc::do_function_runcam_osd_control(const SwitchPos pos)
{
#if HAL_RUNCAM_ENABLED
    AP_RunCam *runcam = AP::runcam();
    if (runcam == nullptr) {
        return;
    }

    switch (pos) {
        case SwitchPos::HIGH:
            runcam->enter_osd();
            break;
        case SwitchPos::MIDDLE:
        case SwitchPos::LOW:
            runcam->exit_osd();
            break;
    }
#endif
}

// enable or disable the fence
void AP_AuxFunc::do_function_fence(const SwitchPos pos)
{
    AC_Fence *fence = AP::fence();
    if (fence == nullptr) {
        return;
    }

    fence->enable(pos == SwitchPos::HIGH);
}

void AP_AuxFunc::do_function_clear_wp(const SwitchPos pos)
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    if (pos == SwitchPos::HIGH) {
        mission->clear();
    }
}

void AP_AuxFunc::do_function_relay(const uint8_t relay, bool val)
{
    AP_ServoRelayEvents *servorelayevents = AP::servorelayevents();
    if (servorelayevents == nullptr) {
        return;
    }
    servorelayevents->do_set_relay(relay, val);
}

#if GENERATOR_ENABLED
void AP_AuxFunc::do_function_generator(const SwitchPos pos)
{
    AP_Generator *generator = AP::generator();
    if (generator == nullptr) {
        return;
    }

    switch (pos) {
    case SwitchPos::LOW:
        generator->stop();
        break;
    case SwitchPos::MIDDLE:
        generator->idle();
        break;
    case SwitchPos::HIGH:
        generator->run();
        break;
    }
}
#endif

void AP_AuxFunc::do_function_sprayer(const SwitchPos pos)
{
#if HAL_SPRAYER_ENABLED
    AC_Sprayer *sprayer = AP::sprayer();
    if (sprayer == nullptr) {
        return;
    }

    sprayer->run(pos == SwitchPos::HIGH);
    // if we are disarmed the pilot must want to test the pump
    sprayer->test_pump((pos == SwitchPos::HIGH) && !hal.util->get_soft_armed());
#endif // HAL_SPRAYER_ENABLED
}

void AP_AuxFunc::do_function_gripper(const SwitchPos pos)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return;
    }

    switch(pos) {
    case SwitchPos::LOW:
        gripper->release();
        break;
    case SwitchPos::MIDDLE:
        // nothing
        break;
    case SwitchPos::HIGH:
        gripper->grab();
        break;
    }
}

void AP_AuxFunc::do_function_lost_vehicle_sound(const SwitchPos pos)
{
    switch (pos) {
    case SwitchPos::HIGH:
        AP_Notify::flags.vehicle_lost = true;
        break;
    case SwitchPos::MIDDLE:
        // nothing
        break;
    case SwitchPos::LOW:
        AP_Notify::flags.vehicle_lost = false;
        break;
    }
}

void AP_AuxFunc::do_function_rc_override_enable(const SwitchPos pos)
{
    switch (pos) {
    case SwitchPos::HIGH: {
        rc().set_gcs_overrides_enabled(true);
        break;
    }
    case SwitchPos::MIDDLE:
        // nothing
        break;
    case SwitchPos::LOW: {
        rc().set_gcs_overrides_enabled(false);
        break;
    }
    }
}

void AP_AuxFunc::do_function_mission_reset(const SwitchPos pos)
{
    if (pos != SwitchPos::HIGH) {
        return;
    }
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }
    mission->reset();
}

bool AP_AuxFunc::run_function(AP_AuxFunc::Function function,
                              AP_AuxFunc::SwitchPos pos,
                              AP_AuxFunc::TriggerSource source)
{
    const bool ret = do_function(function, pos);

    // @LoggerMessage: AUXF
    // @Description: Auixillary function invocation information
    // @Field: TimeUS: Time since system startup
    // @Field: function: ID of triggered function
    // @Field: pos: switch position when function triggered
    // @Field: source: source of auxillary function invocation
    // @Field: result: true if function was successful
    AP::logger().Write(
        "AUXF",
        "TimeUS,function,pos,source,result",
        "s----",
        "F----",
        "QHBBB",
        AP_HAL::micros64(),
        uint16_t(function),
        uint8_t(pos),
        uint8_t(source),
        uint8_t(ret)
        );
    return ret;
}

bool AP_AuxFunc::do_function(const AP_AuxFunc::Function function,
                             const AP_AuxFunc::SwitchPos pos)
{
    switch(function) {
    case Function::CAMERA_TRIGGER:
        do_function_camera_trigger(pos);
        break;

    case Function::FENCE:
        do_function_fence(pos);
        break;

    case Function::GRIPPER:
        do_function_gripper(pos);
        break;

    case Function::RC_OVERRIDE_ENABLE:
        // Allow or disallow RC_Override
        do_function_rc_override_enable(pos);
        break;

    case Function::AVOID_PROXIMITY:
        do_function_avoid_proximity(pos);
        break;

    case Function::RELAY:
        do_function_relay(0, pos == SwitchPos::HIGH);
        break;
    case Function::RELAY2:
        do_function_relay(1, pos == SwitchPos::HIGH);
        break;
    case Function::RELAY3:
        do_function_relay(2, pos == SwitchPos::HIGH);
        break;
    case Function::RELAY4:
        do_function_relay(3, pos == SwitchPos::HIGH);
        break;
    case Function::RELAY5:
        do_function_relay(4, pos == SwitchPos::HIGH);
        break;
    case Function::RELAY6:
        do_function_relay(5, pos == SwitchPos::HIGH);
        break;

    case Function::RUNCAM_CONTROL:
        do_function_runcam_control(pos);
        break;

    case Function::RUNCAM_OSD_CONTROL:
        do_function_runcam_osd_control(pos);
        break;

    case Function::CLEAR_WP:
        do_function_clear_wp(pos);
        break;
    case Function::MISSION_RESET:
        do_function_mission_reset(pos);
        break;

    case Function::AVOID_ADSB:
        do_function_avoid_adsb(pos);
        break;

#if GENERATOR_ENABLED
    case Function::GENERATOR:
        do_function_generator(pos);
        break;
#endif

    case Function::SPRAYER:
        do_function_sprayer(pos);
        break;

    case Function::LOST_VEHICLE_SOUND:
        do_function_lost_vehicle_sound(pos);
        break;

    case Function::ARMDISARM:
        do_function_armdisarm(pos);
        break;

    case Function::DISARM:
        if (pos == SwitchPos::HIGH) {
            AP::arming().disarm(AP_Arming::Method::AUXSWITCH);
        }
        break;

    case Function::COMPASS_LEARN:
        if (pos == SwitchPos::HIGH) {
            Compass &compass = AP::compass();
            compass.set_learn_type(Compass::LEARN_INFLIGHT, false);
        }
        break;

    case Function::LANDING_GEAR: {
        AP_LandingGear *lg = AP_LandingGear::get_singleton();
        if (lg == nullptr) {
            break;
        }
        switch (pos) {
        case SwitchPos::LOW:
            lg->set_position(AP_LandingGear::LandingGear_Deploy);
            break;
        case SwitchPos::MIDDLE:
            // nothing
            break;
        case SwitchPos::HIGH:
            lg->set_position(AP_LandingGear::LandingGear_Retract);
            break;
        }
        break;
    }

    case Function::GPS_DISABLE:
        AP::gps().force_disable(pos == SwitchPos::HIGH);
        break;

    case Function::GPS_DISABLE_YAW:
        AP::gps().set_force_disable_yaw(pos == SwitchPos::HIGH);
        break;

    case Function::DISABLE_AIRSPEED_USE: {
        AP_Airspeed *airspeed = AP::airspeed();
        if (airspeed == nullptr) {
            break;
        }
        switch (pos) {
        case SwitchPos::HIGH:
            airspeed->force_disable_use(true);
            break;
        case SwitchPos::MIDDLE:
            break;
        case SwitchPos::LOW:
            airspeed->force_disable_use(false);
            break;
        }
        break;
    }

    case Function::MOTOR_ESTOP:
        switch (pos) {
        case SwitchPos::HIGH: {
            SRV_Channels::set_emergency_stop(true);

            // log E-stop
            AP_Logger *logger = AP_Logger::get_singleton();
            if (logger && logger->logging_enabled()) {
                logger->Write_Event(LogEvent::MOTORS_EMERGENCY_STOPPED);
            }
            break;
        }
        case SwitchPos::MIDDLE:
            // nothing
            break;
        case SwitchPos::LOW: {
            SRV_Channels::set_emergency_stop(false);

            // log E-stop cleared
            AP_Logger *logger = AP_Logger::get_singleton();
            if (logger && logger->logging_enabled()) {
                logger->Write_Event(LogEvent::MOTORS_EMERGENCY_STOP_CLEARED);
            }
            break;
        }
        }
        break;

    case Function::VISODOM_CALIBRATE:
#if HAL_VISUALODOM_ENABLED
        if (pos == SwitchPos::HIGH) {
            AP_VisualOdom *visual_odom = AP::visualodom();
            if (visual_odom != nullptr) {
                visual_odom->align_sensor_to_vehicle();
            }
        }
#endif
        break;

    case Function::EKF_POS_SOURCE:
        switch (pos) {
        case SwitchPos::LOW:
            // low switches to primary source
            AP::ahrs().set_posvelyaw_source_set(0);
            break;
        case SwitchPos::MIDDLE:
            // middle switches to secondary source
            AP::ahrs().set_posvelyaw_source_set(1);
            break;
        case SwitchPos::HIGH:
            // high switches to tertiary source
            AP::ahrs().set_posvelyaw_source_set(2);
            break;
        }
        break;

#if !HAL_MINIMIZE_FEATURES
    case Function::KILL_IMU1:
        AP::ins().kill_imu(0, pos == SwitchPos::HIGH);
        break;

    case Function::KILL_IMU2:
        AP::ins().kill_imu(1, pos == SwitchPos::HIGH);
        break;
#endif // HAL_MINIMIZE_FEATURES

    case Function::CAM_MODE_TOGGLE: {
        // Momentary switch to for cycling camera modes
        AP_Camera *camera = AP_Camera::get_singleton();
        if (camera == nullptr) {
            break;
        }
        switch (pos) {
        case SwitchPos::LOW:
            // nothing
            break;
        case SwitchPos::MIDDLE:
            // nothing
            break;
        case SwitchPos::HIGH:
            camera->cam_mode_toggle();
            break;
        }
        break;
    }

    case Function::RETRACT_MOUNT: {
#if HAL_MOUNT_ENABLED
        AP_Mount *mount = AP::mount();
        if (mount == nullptr) {
            break;
        }
        switch (pos) {
            case SwitchPos::HIGH:
                mount->set_mode(MAV_MOUNT_MODE_RETRACT);
                break;
            case SwitchPos::MIDDLE:
                // nothing
                break;
            case SwitchPos::LOW:
                mount->set_mode_to_default();
                break;
        }
#endif
        break;
    }

    case Function::EKF_LANE_SWITCH:
        // used to test emergency lane switch
        AP::ahrs().check_lane_switch();
        break;

    case Function::EKF_YAW_RESET:
        // used to test emergency yaw reset
        AP::ahrs().request_yaw_reset();
        break;

    case Function::SCRIPTING_1:
    case Function::SCRIPTING_2:
    case Function::SCRIPTING_3:
    case Function::SCRIPTING_4:
    case Function::SCRIPTING_5:
    case Function::SCRIPTING_6:
    case Function::SCRIPTING_7:
    case Function::SCRIPTING_8:
        break;

    default:
        gcs().send_text(MAV_SEVERITY_INFO, "Invalid Auxiliary Function (%u)", (unsigned int)function);
        return false;
    }

    return true;
}

/*
 * Singleton-Related functionality
 */
AP_AuxFunc *AP_AuxFunc::_singleton = nullptr;

AP_AuxFunc *AP_AuxFunc::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_AuxFunc &auxfunc()
{
    return *AP_AuxFunc::get_singleton();
}

};

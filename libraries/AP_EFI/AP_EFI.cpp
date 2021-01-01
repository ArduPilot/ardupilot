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

#include "AP_EFI.h"

#if EFI_ENABLED

#include "AP_EFI_Serial_MS.h"
#include "AP_EFI_NWPMU.h"
#include <AP_Logger/AP_Logger.h>

#if HAL_MAX_CAN_PROTOCOL_DRIVERS
#include <AP_CANManager/AP_CANManager.h>
#endif

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_EFI::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: EFI communication type
    // @Description: What method of communication is used for EFI #1
    // @Values: 0:None,1:Serial-MS,2:NWPMU
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_EFI, type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _COEF1
    // @DisplayName: EFI Calibration Coefficient 1
    // @Description: Used to calibrate fuel flow for MS protocol (Slope)
    // @Range: 0 1
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_COEF1", 2, AP_EFI, coef1, 0),

    // @Param: _COEF2
    // @DisplayName: EFI Calibration Coefficient 2
    // @Description: Used to calibrate fuel flow for MS protocol (Offset)
    // @Range: 0 10
    // @User: Advanced
    // @RebootRequired: False
    AP_GROUPINFO("_COEF2", 3, AP_EFI, coef2, 0),

    AP_GROUPEND
};

AP_EFI *AP_EFI::singleton;

// Initialize parameters
AP_EFI::AP_EFI()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

// Initialize backends based on existing params
void AP_EFI::init(void)
{
    if (backend != nullptr) {
        // Init called twice, perhaps
        return;
    }
    // Check for MegaSquirt Serial EFI
    switch (type) {
        case EFI_COMMUNICATION_TYPE_NONE:
            break;
        case EFI_COMMUNICATION_TYPE_SERIAL_MS:
            backend = new AP_EFI_Serial_MS(*this);
            break;
        case EFI_COMMUNICATION_TYPE_NWPMU:
#if HAL_EFI_NWPWU_ENABLED
            backend = new AP_EFI_NWPMU(*this);
#endif
            break;
    }
}

// Ask all backends to update the frontend
void AP_EFI::update()
{
    if (backend) {
        backend->update();
        log_status();
    }
}

bool AP_EFI::is_healthy(void) const
{
    return (backend && (AP_HAL::millis() - state.last_updated_ms) < HEALTHY_LAST_RECEIVED_MS);
}

/*
  write status to log
 */
void AP_EFI::log_status(void)
{
// @LoggerMessage: EFI
// @Description: Electronic Fuel Injection system data
// @Field: TimeUS: Time since system startup
// @Field: LP: Reported engine load
// @Field: Rpm: Reported engine RPM
// @Field: SDT: Spark Dwell Time
// @Field: ATM: Atmospheric pressure
// @Field: IMP: Intake manifold pressure
// @Field: IMT: Intake manifold temperature
// @Field: ECT: Engine Coolant Temperature
// @Field: OilP: Oil Pressure
// @Field: OilT: Oil temperature
// @Field: FP: Fuel Pressure
// @Field: FCR: Fuel Consumption Rate
// @Field: CFV: Consumed fueld volume
// @Field: TPS: Throttle Position
// @Field: IDX: Index of the publishing ECU
    AP::logger().Write("EFI",
                       "TimeUS,LP,Rpm,SDT,ATM,IMP,IMT,ECT,OilP,OilT,FP,FCR,CFV,TPS,IDX",
                       "s%qsPPOOPOP--%-",
                       "F00C--00-0-0000",
                       "QBIffffffffffBB",
                       AP_HAL::micros64(),
                       uint8_t(state.engine_load_percent),
                       uint32_t(state.engine_speed_rpm),
                       float(state.spark_dwell_time_ms),
                       float(state.atmospheric_pressure_kpa),
                       float(state.intake_manifold_pressure_kpa),
                       float(state.intake_manifold_temperature),
                       float(state.coolant_temperature),
                       float(state.oil_pressure),
                       float(state.oil_temperature),
                       float(state.fuel_pressure),
                       float(state.fuel_consumption_rate_cm3pm),
                       float(state.estimated_consumed_fuel_volume_cm3),
                       uint8_t(state.throttle_position_percent),
                       uint8_t(state.ecu_index));

// @LoggerMessage: EFI2
// @Description: Electronic Fuel Injection system data - redux
// @Field: TimeUS: Time since system startup
// @Field: Healthy: True if EFI is healthy
// @Field: ES: Engine state
// @Field: GE: General error
// @Field: CSE: Crankshaft sensor status
// @Field: TS: Temperature status
// @Field: FPS: Fuel pressure status
// @Field: OPS: Oil pressure status
// @Field: DS: Detonation status
// @Field: MS: Misfire status
// @Field: DebS: Debris status
// @Field: SPU: Spark plug usage
// @Field: IDX: Index of the publishing ECU
    AP::logger().Write("EFI2",
                       "TimeUS,Healthy,ES,GE,CSE,TS,FPS,OPS,DS,MS,DebS,SPU,IDX",
                       "s------------",
                       "F------------",
                       "QBBBBBBBBBBBB",
                       AP_HAL::micros64(),
                       uint8_t(is_healthy()),
                       uint8_t(state.engine_state),
                       uint8_t(state.general_error),
                       uint8_t(state.crankshaft_sensor_status),
                       uint8_t(state.temperature_status),
                       uint8_t(state.fuel_pressure_status),
                       uint8_t(state.oil_pressure_status),
                       uint8_t(state.detonation_status),
                       uint8_t(state.misfire_status),
                       uint8_t(state.debris_status),
                       uint8_t(state.spark_plug_usage),
                       uint8_t(state.ecu_index));

    for (uint8_t i = 0; i < ENGINE_MAX_CYLINDERS; i++) {
// @LoggerMessage: ECYL
// @Description: EFI per-cylinder information
// @Field: TimeUS: Time since system startup
// @Field: Inst: Cylinder this data belongs to
// @Field: IgnT: Ignition timing
// @Field: InjT: Injection time
// @Field: CHT: Cylinder head temperature
// @Field: EGT: Exhaust gas temperature
// @Field: Lambda: Estimated lambda coefficient (dimensionless ratio)
// @Field: IDX: Index of the publishing ECU
        AP::logger().Write("ECYL",
                           "TimeUS,Inst,IgnT,InjT,CHT,EGT,Lambda,IDX",
                           "s#dsOO--",
                           "F-0C0000",
                           "QBfffffB",
                           AP_HAL::micros64(),
                           i,
                           state.cylinder_status[i].ignition_timing_deg,
                           state.cylinder_status[i].injection_time_ms,
                           state.cylinder_status[i].cylinder_head_temperature,
                           state.cylinder_status[i].exhaust_gas_temperature,
                           state.cylinder_status[i].lambda_coefficient,
                           state.ecu_index);
    }
}

/*
  send EFI_STATUS
 */
void AP_EFI::send_mavlink_status(mavlink_channel_t chan)
{
    if (!backend) {
        return;
    }
    mavlink_msg_efi_status_send(
        chan,
        AP_EFI::is_healthy(),
        state.ecu_index,
        state.engine_speed_rpm,
        state.estimated_consumed_fuel_volume_cm3,
        state.fuel_consumption_rate_cm3pm,
        state.engine_load_percent,
        state.throttle_position_percent,
        state.spark_dwell_time_ms,
        state.atmospheric_pressure_kpa,
        state.intake_manifold_pressure_kpa,
        (state.intake_manifold_temperature - 273.0f),
        (state.cylinder_status[0].cylinder_head_temperature - 273.0f),
        state.cylinder_status[0].ignition_timing_deg,
        state.cylinder_status[0].injection_time_ms,
        0, 0, 0);
}

namespace AP {
AP_EFI *EFI()
{
    return AP_EFI::get_singleton();
}
}

#endif // EFI_ENABLED


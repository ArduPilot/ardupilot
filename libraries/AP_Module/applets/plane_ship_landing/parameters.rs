// SPDX-License-Identifier: GPL-3.0-or-later

use ap_rust_sdk::{Context, Error, Parameter, Result};

ap_rust_sdk::param_table! {
    pub struct Table, enum P;
    key = 7, prefix = "SHIP_";
    size = 3;

    // @Param: SHIP_ENABLE
    // @DisplayName: Ship landing enable
    // @Description: Enable ship landing system
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    Enable = 1, "ENABLE", f32: 0.0;

    // @Param: SHIP_LAND_ANGLE
    // @DisplayName: Ship landing angle
    // @Description: Angle from the stern of the ship for landing approach. Use this to ensure that on a go-around that ship superstructure and cables are avoided. A value of zero means to approach from the rear of the ship. A value of 90 means the landing will approach from the port (left) side of the ship. A value of -90 will mean approaching from the starboard (right) side of the ship. A value of 180 will approach from the bow of the ship. This parameter is combined with the sign of the RTL_RADIUS parameter to determine the holdoff pattern. If RTL_RADIUS is positive then a clockwise loiter is performed, if RTL_RADIUS is negative then a counter-clockwise loiter is used.
    // @Range: -180 180
    // @Units: deg
    // @User: Standard
    LandAngle = 2, "LAND_ANGLE", f32: 0.0;

    // @Param: SHIP_AUTO_OFS
    // @DisplayName: Ship automatic offset trigger
    // @Description: Settings this parameter to one triggers an automatic follow offset calculation based on current position of the vehicle and the landing target. NOTE: This parameter will auto-reset to zero once the offset has been calculated.
    // @Values: 0:Disabled,1:Trigger
    // @User: Standard
    AutoOfs = 3, "AUTO_OFS", f32: 0.0;
}

pub fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
    let start = context.time_us();
    loop {
        match call() {
            Err(Error::Busy) if context.time_us().saturating_sub(start) < 100_000 => {
                context.sleep_ms(1)?;
            }
            result => return result,
        }
    }
}

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

pub fn read(parameter: &Parameter<'_, '_>) -> Result<f32> {
    let value = parameter.get()?.ok_or(Error::Invalid)?;
    value.is_finite().then_some(value).ok_or(Error::Invalid)
}

pub fn save(parameter: &Parameter<'_, '_>, value: f32) -> Result {
    if parameter.set_and_save(value)? {
        Ok(())
    } else {
        Err(Error::Host)
    }
}

pub struct Parameters<'ctx, 'host> {
    pub enable: Parameter<'ctx, 'host>,
    pub land_angle: Parameter<'ctx, 'host>,
    pub auto_offset: Parameter<'ctx, 'host>,
    pub rcmap_throttle: Parameter<'ctx, 'host>,
    pub rtl_altitude: Parameter<'ctx, 'host>,
    pub q_rtl_alt: Parameter<'ctx, 'host>,
    pub airspeed_cruise: Parameter<'ctx, 'host>,
    pub tecs_land_airspeed: Parameter<'ctx, 'host>,
    pub q_transition_decel: Parameter<'ctx, 'host>,
    pub wp_loiter_radius: Parameter<'ctx, 'host>,
    pub rtl_radius: Parameter<'ctx, 'host>,
    pub foll_ofs_x: Parameter<'ctx, 'host>,
    pub foll_ofs_y: Parameter<'ctx, 'host>,
    pub foll_ofs_z: Parameter<'ctx, 'host>,
    pub foll_enable: Parameter<'ctx, 'host>,
    pub foll_ofs_type: Parameter<'ctx, 'host>,
    pub foll_alt_type: Parameter<'ctx, 'host>,
}

impl<'ctx, 'host> Parameters<'ctx, 'host> {
    pub fn new(context: &'ctx Context<'host>) -> Result<Self> {
        Table::register_definitions(context)?;
        Ok(Self {
            enable: Table::bind(context, P::Enable)?,
            land_angle: Table::bind(context, P::LandAngle)?,
            auto_offset: Table::bind(context, P::AutoOfs)?,
            rcmap_throttle: bind(context, "RCMAP_THROTTLE")?,
            rtl_altitude: bind(context, "RTL_ALTITUDE")?,
            q_rtl_alt: bind(context, "Q_RTL_ALT")?,
            airspeed_cruise: bind(context, "AIRSPEED_CRUISE")?,
            tecs_land_airspeed: bind(context, "TECS_LAND_ARSPD")?,
            q_transition_decel: bind(context, "Q_TRANS_DECEL")?,
            wp_loiter_radius: bind(context, "WP_LOITER_RAD")?,
            rtl_radius: bind(context, "RTL_RADIUS")?,
            foll_ofs_x: bind(context, "FOLL_OFS_X")?,
            foll_ofs_y: bind(context, "FOLL_OFS_Y")?,
            foll_ofs_z: bind(context, "FOLL_OFS_Z")?,
            foll_enable: bind(context, "FOLL_ENABLE")?,
            foll_ofs_type: bind(context, "FOLL_OFS_TYPE")?,
            foll_alt_type: bind(context, "FOLL_ALT_TYPE")?,
        })
    }
}

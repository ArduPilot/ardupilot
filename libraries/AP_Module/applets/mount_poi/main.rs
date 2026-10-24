// SPDX-License-Identifier: GPL-3.0-or-later
//! Native port of AP_Scripting/applets/mount-poi.lua.
#![no_std]
#![no_main]

mod mavlink;

use ap_rust_sdk::{
    enums::{LocationAltFrame, MavSeverity, RcChannelAuxFunc},
    Context, Error, Parameter, Result,
};
use core::fmt::{self, Write};
use mavlink::{camera_feedback, CAMERA_FEEDBACK_ID};

ap_rust_sdk::module!(run);

const PARAM_TABLE_KEY: u8 = 78;
const AUX_POI: RcChannelAuxFunc = RcChannelAuxFunc(300);
const AUX_ROI: RcChannelAuxFunc = RcChannelAuxFunc(301);
const ALT_ABSOLUTE: LocationAltFrame = LocationAltFrame(0);
const RAD_TO_DEG: f32 = 57.295_78;

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self { bytes: [0; N], length: 0 };
        text.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(text)
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.length]).unwrap()
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length + value.len();
        if end > N {
            return Err(fmt::Error);
        }
        self.bytes[self.length..end].copy_from_slice(value.as_bytes());
        self.length = end;
        Ok(())
    }
}

fn retry<T>(context: &Context<'_>, mut call: impl FnMut() -> Result<T>) -> Result<T> {
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

fn text(context: &Context<'_>, severity: i32, args: fmt::Arguments<'_>) -> Result {
    let message = Text::<160>::new(args)?;
    retry(context, || context.gcs().send_text(MavSeverity(severity), message.as_str()))
}

fn bind<'a, 'host>(context: &'a Context<'host>, name: &str) -> Result<Parameter<'a, 'host>> {
    retry(context, || context.parameter(name))?.ok_or(Error::Invalid)
}

fn value(context: &Context<'_>, parameter: &Parameter<'_, '_>) -> Result<f32> {
    retry(context, || parameter.get())?
        .filter(|value| value.is_finite())
        .ok_or(Error::Invalid)
}

fn wrap_180(mut angle: f32) -> f32 {
    angle %= 360.0;
    if angle < 0.0 {
        angle += 360.0;
    }
    if angle > 180.0 {
        angle -= 360.0;
    }
    angle
}

fn interpolate(mut low: f32, mut high: f32, value: f32, mut value_low: f32, mut value_high: f32) -> f32 {
    if value_low > value_high {
        core::mem::swap(&mut value_low, &mut value_high);
        core::mem::swap(&mut low, &mut high);
    }
    if value <= value_low {
        low
    } else if value > value_high {
        high
    } else {
        low + (value - value_low) / (value_high - value_low) * (high - low)
    }
}

struct Applet<'ctx, 'host> {
    distance_max: Parameter<'ctx, 'host>,
    terrain_spacing: Parameter<'ctx, 'host>,
    last_poi: Option<u8>,
    last_roi: Option<u8>,
    success_count: u16,
}

impl Applet<'_, '_> {
    fn send_feedback(&self, context: &Context<'_>, lat: i32, lon: i32, altitude: i32) -> Result {
        let payload = camera_feedback(
            context.time_us(),
            lat,
            lon,
            altitude as f32,
            altitude as f32,
            0.0,
            0.0,
            0.0,
            0.0,
            1,
            0,
            0,
            0,
            self.success_count,
        );
        let _ = retry(context, || {
            context.mavlink().send_chan(0, CAMERA_FEEDBACK_ID, &payload, None)
        })?;
        let _ = retry(context, || {
            context.mavlink().send_chan(1, CAMERA_FEEDBACK_ID, &payload, None)
        })?;
        Ok(())
    }

    fn update(&mut self, context: &Context<'_>) -> Result {
        let poi = retry(context, || context.rc().get_aux_cached(AUX_POI))?;
        let roi = retry(context, || context.rc().get_aux_cached(AUX_ROI))?;
        let poi_requested = poi == Some(2) && poi != self.last_poi;
        let roi_requested = roi == Some(2) && roi != self.last_roi;
        self.last_poi = poi;
        self.last_roi = roi;
        if !poi_requested && !roi_requested {
            return Ok(());
        }

        let Some(mut location) = retry(context, || context.ahrs().get_location())? else {
            return text(context, 3, format_args!("POI: vehicle pos unavailable"));
        };
        let _ = retry(context, || location.change_alt_frame(context, ALT_ABSOLUTE))?;
        let Some((_, pitch, yaw_body)) = retry(context, || context.mount().get_attitude_euler(0))? else {
            return text(context, 3, format_args!("POI: gimbal attitude unavailable"));
        };

        let mut test_location = location;
        let mut previous_location = test_location;
        let Some(mut terrain_altitude) = retry(context, || {
            context.terrain().height_amsl(&mut test_location, true)
        })? else {
            return text(context, 3, format_args!("POI: failed to get terrain alt"));
        };
        let mut previous_terrain_altitude = terrain_altitude;
        let yaw = wrap_180(yaw_body + retry(context, || context.ahrs().get_yaw_rad())? * RAD_TO_DEG);
        let increment = value(context, &self.terrain_spacing)?;
        let maximum = value(context, &self.distance_max)?;
        let mut distance = 0.0;

        while distance < maximum && test_location.alt() as f32 * 0.01 > terrain_altitude {
            distance += increment;
            previous_location = test_location;
            previous_terrain_altitude = terrain_altitude;
            retry(context, || {
                test_location.offset_bearing_and_pitch(context, yaw, pitch, increment)
            })?;
            let Some(altitude) = retry(context, || {
                context.terrain().height_amsl(&mut test_location, true)
            })? else {
                return text(context, 3, format_args!("POI: failed to get terrain alt"));
            };
            terrain_altitude = altitude;
        }

        if distance >= maximum {
            return text(
                context,
                3,
                format_args!("POI: unable to find terrain within {maximum} m"),
            );
        }
        let interpolation = interpolate(
            0.0,
            increment,
            0.0,
            previous_location.alt() as f32 * 0.01 - previous_terrain_altitude,
            test_location.alt() as f32 * 0.01 - terrain_altitude,
        );
        let mut poi_location = previous_location;
        retry(context, || {
            poi_location.offset_bearing_and_pitch(context, yaw, pitch, interpolation)
        })?;
        text(
            context,
            6,
            format_args!(
                "POI {:.7}, {:.7}, {:.2} (asml)",
                f64::from(poi_location.lat()) / 10_000_000.0,
                f64::from(poi_location.lng()) / 10_000_000.0,
                f64::from(poi_location.alt()) * 0.01,
            ),
        )?;
        if roi_requested {
            retry(context, || context.mount().set_roi_target(0, &mut poi_location))?;
        }
        self.success_count = self.success_count.checked_add(1).ok_or(Error::Invalid)?;
        self.send_feedback(context, poi_location.lat(), poi_location.lng(), poi_location.alt())
    }
}

fn run(context: &Context<'_>) -> Result {
    if !retry(context, || context.param().add_table(PARAM_TABLE_KEY, "POI_", 1))?
        || !retry(context, || {
            context.param().add_param(PARAM_TABLE_KEY, 1, "DIST_MAX", 10_000.0)
        })?
    {
        return Err(Error::Host);
    }
    let mut applet = Applet {
        distance_max: bind(context, "POI_DIST_MAX")?,
        terrain_spacing: bind(context, "TERRAIN_SPACING")?,
        last_poi: Some(0),
        last_roi: Some(0),
        success_count: 0,
    };
    text(context, 6, format_args!("Mount-poi script started"))?;
    loop {
        match applet.update(context) {
            Ok(()) => {}
            Err(Error::Stopped) => return Ok(()),
            Err(error) => {
                let _ = text(context, 3, format_args!("Mount-poi error: {error:?}"));
            }
        }
        match context.sleep_ms(100) {
            Ok(()) => {}
            Err(Error::Stopped) => return Ok(()),
            Err(error) => return Err(error),
        }
    }
}

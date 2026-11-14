// SPDX-License-Identifier: GPL-3.0-or-later
//! DroneCAN CircuitStatus battery-monitor driver.
#![no_std]
#![no_main]

use ap_rust_sdk::{BattMonitorScriptState, Context, DroneCanSubscription, Error, Result};

use core::fmt::{self, Write};

ap_rust_sdk::module!(run);

ap_rust_sdk::param_table! {
    pub struct Table, enum P;
    key = 50, prefix = "DCS";
    size = 19;

    // @Param: DCS_NUM_CIRCUITS
    // @DisplayName: Number of CircuitStatus monitors
    // @Description: Number of DroneCAN CircuitStatus battery monitor instances. The per-instance DCSx_ parameters are created based on this value.
    // @Range: 0 9
    // @RebootRequired: True
    // @User: Standard
    NumCircuits = 1, "_NUM_CIRCUITS", f32: 0.0;

    // @Param: DCS1_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit1CircuitId = 2, "1_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS1_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit1BattIdx = 3, "1_BATT_IDX", f32: 0.0;

    // @Param: DCS2_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit2CircuitId = 4, "2_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS2_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit2BattIdx = 5, "2_BATT_IDX", f32: 0.0;

    // @Param: DCS3_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit3CircuitId = 6, "3_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS3_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit3BattIdx = 7, "3_BATT_IDX", f32: 0.0;

    // @Param: DCS4_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit4CircuitId = 8, "4_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS4_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit4BattIdx = 9, "4_BATT_IDX", f32: 0.0;

    // @Param: DCS5_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit5CircuitId = 10, "5_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS5_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit5BattIdx = 11, "5_BATT_IDX", f32: 0.0;

    // @Param: DCS6_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit6CircuitId = 12, "6_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS6_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit6BattIdx = 13, "6_BATT_IDX", f32: 0.0;

    // @Param: DCS7_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit7CircuitId = 14, "7_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS7_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit7BattIdx = 15, "7_BATT_IDX", f32: 0.0;

    // @Param: DCS8_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit8CircuitId = 16, "8_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS8_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit8BattIdx = 17, "8_BATT_IDX", f32: 0.0;

    // @Param: DCS9_CIRCUIT_ID
    // @DisplayName: CircuitStatus circuit ID
    // @Description: circuit_id of the CircuitStatus message to use for this instance. Set to -1 to disable.
    // @Range: -1 65535
    // @User: Standard
    Circuit9CircuitId = 18, "9_CIRCUIT_ID", f32: -1.0;

    // @Param: DCS9_BATT_IDX
    // @DisplayName: CircuitStatus battery index
    // @Description: Battery monitor instance to feed with this circuit. Set to 1 for BATT, 2 for BATT2 etc. The corresponding BATTn_MONITOR must be 29 (scripting). Set to 0 to disable.
    // @Range: 0 9
    // @User: Standard
    Circuit9BattIdx = 19, "9_BATT_IDX", f32: 0.0;
}

const CIRCUIT_STATUS_ID: u16 = 1091;
const CIRCUIT_STATUS_SIGNATURE: u64 = 0x8313_d33d_0ddd_a115;
const MAX_CIRCUITS: usize = 9;

struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Text<N> {
    fn new(args: fmt::Arguments<'_>) -> Result<Self> {
        let mut text = Self {
            bytes: [0; N],
            length: 0,
        };
        text.write_fmt(args).map_err(|_| Error::Invalid)?;
        Ok(text)
    }

    fn as_str(&self) -> Result<&str> {
        core::str::from_utf8(&self.bytes[..self.length]).map_err(|_| Error::Host)
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length.checked_add(value.len()).ok_or(fmt::Error)?;
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

fn half_to_float(value: u16) -> f32 {
    let sign = u32::from(value & 0x8000) << 16;
    let exponent = (value >> 10) & 0x1f;
    let mut fraction = u32::from(value & 0x03ff);
    let bits = if exponent == 0 {
        if fraction == 0 {
            sign
        } else {
            let mut exponent = 113u32;
            while fraction & 0x0400 == 0 {
                fraction <<= 1;
                exponent -= 1;
            }
            sign | (exponent << 23) | ((fraction & 0x03ff) << 13)
        }
    } else if exponent == 0x1f {
        sign | 0x7f80_0000 | (fraction << 13)
    } else {
        sign | ((u32::from(exponent) + 112) << 23) | (fraction << 13)
    };
    f32::from_bits(bits)
}

fn parameter_name(circuit: usize, suffix: &str) -> Result<Text<17>> {
    Text::new(format_args!("DCS{circuit}_{suffix}"))
}

fn handle_message(
    context: &Context<'_>,
    payload: &[u8],
    circuits: usize,
    last_error: &mut [u8; MAX_CIRCUITS],
    last_index_warning_ms: &mut u32,
) -> Result {
    let bytes: [u8; 7] = match payload.try_into() {
        Ok(bytes) => bytes,
        Err(_) => return Ok(()),
    };
    let circuit_id = u16::from_le_bytes([bytes[0], bytes[1]]);
    let voltage = half_to_float(u16::from_le_bytes([bytes[2], bytes[3]]));
    let current = half_to_float(u16::from_le_bytes([bytes[4], bytes[5]]));
    let error_flags = bytes[6];
    if !voltage.is_finite() || voltage < 0.0 {
        return Ok(());
    }
    for circuit in 1..=circuits {
        let circuit_name = parameter_name(circuit, "CIRCUIT_ID")?;
        if retry(context, || context.param().get(circuit_name.as_str()?))? != Some(circuit_id as f32) {
            continue;
        }
        let battery_name = parameter_name(circuit, "BATT_IDX")?;
        let battery_index = retry(context, || context.param().get(battery_name.as_str()?))?
            .unwrap_or(0.0) as i32;
        if !(1..=9).contains(&battery_index) {
            continue;
        }
        let mut state = BattMonitorScriptState::default();
        state.healthy = true;
        state.voltage = voltage;
        state.current_amps = current.is_finite().then_some(current);
        if !retry(context, || {
            context.battery().handle_scripting((battery_index - 1) as u8, &state)
        })? {
            let now_ms = (context.time_us() / 1000) as u32;
            if now_ms.wrapping_sub(*last_index_warning_ms) > 10_000 {
                *last_index_warning_ms = now_ms;
                let name = if battery_index == 1 {
                    Text::<8>::new(format_args!("BATT"))?
                } else {
                    Text::<8>::new(format_args!("BATT{battery_index}"))?
                };
                let warning = Text::<80>::new(format_args!(
                    "CircuitStatus: {}_MONITOR must be 29",
                    name.as_str()?
                ))?;
                retry(context, || context.text(4, warning.as_str()?))?;
            }
        }
        if last_error[circuit - 1] != error_flags {
            last_error[circuit - 1] = error_flags;
            if error_flags != 0 {
                let warning = Text::<80>::new(format_args!(
                    "CircuitStatus: circuit {circuit_id} error 0x{error_flags:02x}"
                ))?;
                retry(context, || context.text(4, warning.as_str()?))?;
            }
        }
    }
    Ok(())
}

fn run(context: &Context<'_>) -> Result {
    Table::register_table(context)?;
    Table::definition(P::NumCircuits).register(context, Table::KEY)?;
    let circuits = retry(context, || context.param().get("DCS_NUM_CIRCUITS"))?
        .unwrap_or(0.0)
        .clamp(0.0, MAX_CIRCUITS as f32) as usize;
    if circuits == 0 {
        return Ok(());
    }
    for d in &Table::definitions()[1..1 + 2 * circuits] {
        d.register(context, Table::KEY)?;
    }

    let mut handles: [Option<DroneCanSubscription>; 2] = [None, None];
    let mut handle_count = 0;
    for bus in 0..2 {
        if let Some(handle) = retry(context, || {
            context.dronecan().subscribe(bus, CIRCUIT_STATUS_SIGNATURE, CIRCUIT_STATUS_ID)
        })? {
            handles[handle_count] = Some(handle);
            handle_count += 1;
        }
    }
    if handle_count == 0 {
        retry(context, || context.text(3, "CircuitStatus: no DroneCAN driver"))?;
        return Ok(());
    }
    let loaded = Text::<64>::new(format_args!(
        "CircuitStatus: loaded {circuits} circuits"
    ))?;
    retry(context, || context.text(6, loaded.as_str()?))?;

    let mut last_error = [0; MAX_CIRCUITS];
    let mut last_index_warning_ms = 0;
    loop {
        for handle in handles[..handle_count].iter().flatten() {
            while let Some(message) = retry(context, || handle.check_message(context))? {
                handle_message(
                    context,
                    message.payload(),
                    circuits,
                    &mut last_error,
                    &mut last_index_warning_ms,
                )?;
            }
        }
        match context.sleep_ms(100) {
            Ok(()) => {}
            Err(Error::Stopped) => return Ok(()),
            Err(error) => return Err(error),
        }
    }
}

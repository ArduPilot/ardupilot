// SPDX-License-Identifier: GPL-3.0-or-later
#![no_std]

use core::alloc::{GlobalAlloc, Layout};
use core::ffi::c_void;
use core::marker::PhantomData;
use core::panic::PanicInfo;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};

macro_rules! generated_module {
    ($name:ident) => {
        mod $name {
            include!(concat!(env!("AP_MODULE_SDK_GENERATED"), "/", stringify!($name), ".rs"));
        }
    };
}

macro_rules! generated_public_module {
    ($name:ident) => {
        pub mod $name {
            include!(concat!(env!("AP_MODULE_SDK_GENERATED"), "/", stringify!($name), ".rs"));
        }
    };
}

generated_public_module!(api);
generated_module!(analog_pwm);
generated_module!(airspeed);
generated_module!(applet);
generated_module!(arming);
generated_public_module!(enums);
generated_public_module!(bindings);
generated_module!(battery);
generated_module!(borrowed);
generated_module!(can);
mod compatibility;
generated_module!(camera);
generated_module!(command_int);
generated_module!(crsf);
generated_module!(custom_mode);
generated_module!(dronecan);
generated_module!(efi);
generated_module!(example_constants);
generated_module!(examples);
generated_module!(file);
generated_module!(filesystem);
generated_module!(format);
mod gcs;
generated_module!(log);
generated_module!(mavlink);
generated_module!(imports);
generated_module!(i2c);
generated_module!(libm);
generated_module!(network);
generated_module!(strtof);
generated_module!(math);
mod memory;
mod printf;
generated_module!(motor_config);
generated_module!(onvif);
generated_module!(old_parameter);
generated_module!(parameter);
generated_module!(periph);
generated_module!(rangefinder_proximity);
generated_module!(serial);
generated_module!(serialled);
generated_module!(shared_state);
pub use shared_state::SharedState;
generated_module!(telemetry);
generated_module!(values);
pub use applet::{
    VideoStreamInformation, GPS_INJECT_DATA_MAX, VIDEO_STREAM_NAME_SIZE, VIDEO_STREAM_URI_SIZE,
};
pub use analog_pwm::{AnalogSource, PwmSource};
pub use airspeed::AirspeedBackend;
pub use math::{Quaternion, Vector3f};
pub use arming::AUX_AUTH_FAILURE_MESSAGE_MAX;
pub use borrowed::RcChannel;
pub use battery::BattMonitorScriptState;
pub use can::{
    CanBuffer, CanFrame, CAN_BUFFER_LENGTH_MAX, CAN_FLAG_EFF, CAN_FLAG_ERR, CAN_FLAG_RTR,
    CAN_FRAME_DATA_MAX, CAN_MASK_EXT_ID, CAN_MASK_STD_ID, CAN_WRITE_TIMEOUT_MAX_US,
};
pub use camera::{CameraInformation, CameraState, CAMERA_DEFINITION_URI_SIZE,
                 CAMERA_MODEL_NAME_SIZE, CAMERA_VENDOR_NAME_SIZE};
pub use command_int::CommandIntParams;
pub use crsf::{
    CrsfMenu, CrsfMenuEvent, CrsfParameter, CrsfParameterData, CRSF_EVENT_PAYLOAD_MAX,
    CRSF_MENU_NAME_MAX, CRSF_PARAMETER_DATA_MAX,
};
pub use custom_mode::{CustomMode, CUSTOM_MODE_FULL_NAME_MAX, CUSTOM_MODE_SHORT_NAME_MAX};
pub use dronecan::{DroneCanHandle, DroneCanMessage, DroneCanSubscription, DRONECAN_PAYLOAD_MAX};
pub use efi::{
    CrankshaftSensorStatus, DebrisStatus, DetonationStatus, EfiBackend, EfiCylinderState, EfiState,
    EngineState, FuelPressureStatus, MisfireStatus, OilPressureStatus, SparkPlugUsage, TemperatureStatus,
};
pub use example_constants::{
    GPS_NO_FIX, GPS_NO_GPS, GPS_OK_FIX_2D, GPS_OK_FIX_3D, GPS_OK_FIX_3D_DGPS,
    GPS_OK_FIX_3D_RTK_FIXED, GPS_OK_FIX_3D_RTK_FLOAT, MISSION_COMPLETE, MISSION_RUNNING,
    MISSION_STOPPED, TERRAIN_STATUS_DISABLED, TERRAIN_STATUS_OK, TERRAIN_STATUS_UNHEALTHY,
};
pub use examples::{DroneCanFlexDebug, MissionCommand};
pub use i2c::{I2cDevice, I2C_TRANSFER_MAX};
pub use filesystem::{DirEntry, FileStat, FILESYSTEM_READ_MAX, FILESYSTEM_WRITE_MAX};
pub use mavlink::{MavlinkMessage, MAVLINK_MESSAGE_SIZE, MAVLINK_PAYLOAD_MAX};
pub use motor_config::{MotorFactorTable, MOTOR_FACTOR_COUNT, MOTOR_FRAME_STRING_MAX};
pub use network::{
    Ipv4Address, Ipv4Endpoint, Socket, SocketReceive, SOCKET_POLL_TIMEOUT_MAX_MS, SOCKET_TRANSFER_MAX,
};
pub use onvif::ONVIF_STRING_MAX;
pub use old_parameter::OldParameterType;
pub use parameter::Parameter;
pub use rangefinder_proximity::{
    ProximityBackend, RangeFinderBackend, RangeFinderState, RangeFinderStatus,
};
pub use serial::{SerialFlowControl, SerialParity, SerialPort, SerialStopBits, SERIAL_TRANSFER_MAX};
pub use telemetry::{EscTelemetryUpdate, ServoTelemetryData, ServoTelemetryUpdate};
pub use log::LogValue;
pub use values::{Location, MissionItem, Vector2f};
pub use printf::{PrintfArg, PrintfText};

/// A bounded copy of a static firmware version string.
///
/// The host rejects null pointers, strings longer than 64 bytes and invalid
/// UTF-8 rather than truncating or exposing firmware memory to the module.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FirmwareString {
    bytes: [u8; 64],
    length: u8,
}

impl FirmwareString {
    pub(crate) fn decode(wire: &[u8]) -> Result<Self> {
        if wire.len() != 65 {
            return Err(Error::Host);
        }
        let length = wire[0] as usize;
        if length > 64 || wire[1 + length..].iter().any(|byte| *byte != 0) {
            return Err(Error::Host);
        }
        let mut bytes = [0; 64];
        bytes.copy_from_slice(&wire[1..]);
        core::str::from_utf8(&bytes[..length]).map_err(|_| Error::Host)?;
        Ok(Self { bytes, length: length as u8 })
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.length as usize]
    }

    pub fn as_str(&self) -> &str {
        // Construction validates UTF-8 and fields are private.
        unsafe { core::str::from_utf8_unchecked(self.as_bytes()) }
    }

    pub fn len(&self) -> usize {
        self.length as usize
    }

    pub fn is_empty(&self) -> bool {
        self.length == 0
    }
}

/// Patched by the loader before entry; read-only for the module's lifetime.
#[repr(transparent)]
struct Import(core::cell::UnsafeCell<u32>);

// Only the loader writes, before any Rust code runs. No module mutation API.
unsafe impl Sync for Import {}

impl Import {
    const fn new() -> Self {
        Self(core::cell::UnsafeCell::new(u32::MAX))
    }

    fn slot(&self) -> u32 {
        // The compiler must not replace the externally patched slot with its
        // initial value, including under fat LTO.
        unsafe { self.0.get().read_volatile() }
    }
}

#[cfg(test)]
mod binding_tests;
#[cfg(test)]
mod gcs_tests;
#[cfg(test)]
mod printf_tests;
#[cfg(test)]
mod panic_tests;
#[cfg(test)]
mod analog_pwm_tests;
#[cfg(test)]
mod camera_tests;
#[cfg(test)]
mod custom_mode_tests;
#[cfg(test)]
mod efi_tests;
#[cfg(test)]
mod borrowed_tests;
#[cfg(test)]
mod value_tests;
#[cfg(test)]
mod file_tests;
#[cfg(test)]
mod fwversion_tests;
#[cfg(test)]
mod log_tests;
#[cfg(test)]
mod math_tests;
#[cfg(test)]
mod libm_tests;
#[cfg(test)]
mod strtof_tests;
#[cfg(test)]
mod parameter_tests;
#[cfg(test)]
mod singleton_tests;
#[cfg(test)]
mod serial_tests;

static ENTERED: AtomicBool = AtomicBool::new(false);
static PANICKING: AtomicBool = AtomicBool::new(false);
static HOST: AtomicPtr<api::HostApi> = AtomicPtr::new(ptr::null_mut());
static CONTEXT: AtomicPtr<c_void> = AtomicPtr::new(ptr::null_mut());

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Stopped,
    Invalid,
    NoMemory,
    Unavailable,
    /// Transient binding-lock contention. Sleep before retrying; propagating
    /// this out of start reports a module error rather than retrying for you.
    Busy,
    Host,
}

pub type Result<T = ()> = core::result::Result<T, Error>;

impl Error {
    fn status(self) -> i32 {
        match self {
            Self::Stopped => api::STATUS_STOPPED,
            Self::Invalid => api::STATUS_INVALID,
            Self::NoMemory => api::STATUS_NO_MEMORY,
            Self::Unavailable => api::STATUS_UNAVAILABLE,
            Self::Busy => api::STATUS_BUSY,
            Self::Host => api::STATUS_HOST_ERROR,
        }
    }
}

fn result(status: i32) -> Result {
    match status {
        api::STATUS_OK => Ok(()),
        api::STATUS_STOPPED => Err(Error::Stopped),
        api::STATUS_INVALID => Err(Error::Invalid),
        api::STATUS_NO_MEMORY => Err(Error::NoMemory),
        api::STATUS_UNAVAILABLE => Err(Error::Unavailable),
        api::STATUS_BUSY => Err(Error::Busy),
        _ => Err(Error::Host),
    }
}

/// One native thread owns a module context. It cannot be sent or shared with
/// other threads, and safe module code cannot manufacture a firmware context.
pub struct Context<'a> {
    api: &'a api::HostApi,
    raw: *mut c_void,
    _not_send_sync: PhantomData<*mut ()>,
}

impl Context<'_> {
    pub fn time_us(&self) -> u64 {
        // Entry validates all function pointers before creating the Context.
        unsafe { self.api.time_us.unwrap()(self.raw) }
    }

    pub fn stop_requested(&self) -> bool {
        unsafe { self.api.stop_requested.unwrap()(self.raw) != 0 }
    }

    /// A stop request interrupts sleep. Propagate the error with `?` so module
    /// locals are dropped before returning through the firmware trampoline.
    pub fn sleep_ms(&self, milliseconds: u32) -> Result {
        result(unsafe { self.api.sleep_ms.unwrap()(self.raw, milliseconds) })
    }

    /// MAVLink severity 0 (emergency) through 7 (debug). The host copies the
    /// message during this call; it must not retain a pointer into the module.
    pub fn text(&self, severity: u8, message: &str) -> Result {
        if severity > 7 || message.len() > u32::MAX as usize {
            return Err(Error::Invalid);
        }
        result(unsafe {
            self.api.text.unwrap()(self.raw, severity, message.as_ptr(), message.len() as u32)
        })
    }
}

/// Firmware-only entry adapter. Each separately loaded image may enter once.
///
/// # Safety
/// `api` must point to a readable version/size prefix (two u32 fields). If its
/// size advertises a complete HostApi, that whole object must be readable,
/// immutable and alive until the module thread has been joined. `context` must be the live
/// firmware context expected by every callback. Callbacks must obey the host
/// contract, never unwind, and never invoke module code or retain its pointers.
#[doc(hidden)]
pub unsafe fn enter(
    api: *const api::HostApi,
    context: *mut c_void,
    run: fn(&Context<'_>) -> Result,
) -> i32 {
    if api.is_null()
        || context.is_null()
        || (api as usize) % core::mem::align_of::<api::HostApi>() != 0
    {
        return api::STATUS_INVALID;
    }
    // Read only the prefix before forming a reference to the complete table.
    let prefix = api.cast::<u32>();
    if prefix.read() != api::ABI_VERSION
        || (prefix.add(1).read() as usize) < core::mem::size_of::<api::HostApi>()
    {
        return api::STATUS_INVALID;
    }
    if !(*api).is_valid() {
        return api::STATUS_INVALID;
    }
    if ENTERED.swap(true, Ordering::AcqRel) {
        return api::STATUS_INVALID;
    }
    CONTEXT.store(context, Ordering::Relaxed);
    HOST.store(api.cast_mut(), Ordering::Release);
    let module = Context {
        api: &*api,
        raw: context,
        _not_send_sync: PhantomData,
    };
    let status = match run(&module) {
        Ok(()) => api::STATUS_OK,
        Err(error) => error.status(),
    };
    HOST.store(ptr::null_mut(), Ordering::Release);
    status
}

/// Only `module!` should install this allocator. Firmware reclaims outstanding
/// allocations after joining, including deliberately forgotten Rust objects.
#[doc(hidden)]
pub struct Allocator;

unsafe impl GlobalAlloc for Allocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let host = HOST.load(Ordering::Acquire);
        if host.is_null() || layout.size() > u32::MAX as usize || layout.align() > u32::MAX as usize
        {
            return ptr::null_mut();
        }
        // Never panic or unwind from allocator callbacks.
        match (*host).allocate {
            Some(allocate) => allocate(
                CONTEXT.load(Ordering::Relaxed),
                layout.size() as u32,
                layout.align() as u32,
            ),
            None => ptr::null_mut(),
        }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, _: Layout) {
        let host = HOST.load(Ordering::Acquire);
        if !host.is_null() {
            if let Some(deallocate) = (*host).deallocate {
                deallocate(CONTEXT.load(Ordering::Relaxed), pointer);
            }
        }
    }
}

fn panic_prefix(text: &str, maximum: usize) -> &str {
    let mut end = text.len().min(maximum);
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    // end is in bounds and was rounded down to a UTF-8 boundary above. Avoid
    // str's indexing panic machinery (including Unicode debug formatting) here.
    unsafe { text.get_unchecked(..end) }
}

fn panic_text(
    context: &Context<'_>,
    message: &str,
    location: Option<(&str, u32, u32)>,
) -> Result<PrintfText> {
    if let Some((file, line, column)) = location {
        // Keep the filename end, where the useful component lives. Together
        // these bounds fit both the string pool and worst-case 255-byte output.
        let mut start = file.len().saturating_sub(128);
        while !file.is_char_boundary(start) {
            start += 1;
        }
        // start is in bounds and was rounded up to a UTF-8 boundary above.
        let file = unsafe { file.get_unchecked(start..) };
        context.string().format_printf(
            "%s at %s:%u:%u",
            &[
                panic_prefix(message, 96).into(),
                file.into(),
                line.into(),
                column.into(),
            ],
        )
    } else {
        context
            .string()
            .format_printf("%s", &[panic_prefix(message, 192).into()])
    }
}

/// Report the failure without allocating or unwinding. The host panic callback
/// marks this module failed and parks the thread indefinitely at low priority.
/// Its heap, image and stack must remain owned until reboot.
#[doc(hidden)]
pub fn panic(info: &PanicInfo<'_>) -> ! {
    let host = HOST.load(Ordering::Acquire);
    if !host.is_null() {
        // Formatting a user-supplied Display value can allocate or panic
        // recursively. Pass only a literal message and primitive location
        // fields to firmware printf instead of Rust's formatting machinery.
        let recursive = PANICKING.swap(true, Ordering::Relaxed);
        let fallback = if recursive {
            "Rust panic while reporting panic"
        } else {
            panic_prefix(
                info.message()
                    .as_str()
                    .unwrap_or("Rust panic (formatted message omitted)"),
                192,
            )
        };
        unsafe {
            if let Some(report) = (*host).panic {
                let raw = CONTEXT.load(Ordering::Relaxed);
                let context = Context {
                    api: &*host,
                    raw,
                    _not_send_sync: PhantomData,
                };
                let formatted = if recursive {
                    Err(Error::Host)
                } else {
                    panic_text(
                        &context,
                        fallback,
                        info.location()
                            .map(|location| (location.file(), location.line(), location.column())),
                    )
                };
                // Formatting can be rejected after stop, or fail for malformed
                // input. Still mark the module panicked using bounded raw text.
                let message = formatted
                    .as_ref()
                    .map(PrintfText::as_str)
                    .unwrap_or(fallback);
                report(raw, message.as_ptr(), message.len() as u32);
            }
        }
    }
    // Only reachable before entry, or if firmware violates the non-returning
    // panic callback contract. Never unwind across native firmware frames.
    loop {
        core::hint::spin_loop();
    }
}

/// Export the single native start symbol and install the no_std allocator and
/// panic handler. The supplied function takes `&Context` and returns `Result`.
#[macro_export]
macro_rules! module {
    ($start:path) => {
        #[global_allocator]
        static AP_MODULE_ALLOCATOR: $crate::Allocator = $crate::Allocator;

        #[panic_handler]
        fn ap_rust_panic(info: &core::panic::PanicInfo<'_>) -> ! {
            $crate::panic(info)
        }

        #[no_mangle]
        pub unsafe extern "C" fn start(
            api: *const $crate::api::HostApi,
            context: *mut core::ffi::c_void,
        ) -> i32 {
            $crate::enter(api, context, $start)
        }
    };
}

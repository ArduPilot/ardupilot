# ELRS receiver applet

`main.rs` is the production native-module entry point. `protocol.rs`,
`transport.rs`, `tracker.rs`, `receiver_control.rs` and `stream.rs` implement the
ELRS receiver, hardware recovery policy, binding, LED patterns and MAVLink framing
and flow control. `driver_sx1280_module.cpp` executes opaque module actions on a
worker at SPI priority + 1; SX1280 and VirtualUART drivers remain C++.

`parameters.rs` owns dynamic parameters, binding requests, learned UID persistence
and reboot. `module_rc.cpp` publishes the module's generic RC frame/status mailbox
over CAN; `AP_Module::RCInputSource` keeps the publication-expiry watchdog alive
after module exit. Timeout/repeat values come from the applet. SX1280 C++ code
interprets hardware actions only.

## Supported features

- Automatic RF-rate detection and frequency hopping.
- LoRa: 50, 150, 250 and 500 Hz; 100 and 333 Hz Full Resolution.
- FLRC: F500, F1000, D250 and D500.
- Hybrid/Wide: 12 channels. Full Resolution: up to 16 channels.
- Over-air binding, model matching and RSSI/SNR/LQ telemetry.
- Telemetry Off and ratios from 1:128 to 1:2.
- Bidirectional MAVLink transport through a DroneCAN serial tunnel.

## Setup

Enable `Periph_Module_RC`, `Module_SX1280` and its native-module dependency on an
AP_Periph board with SX1280 and VirtualUART support. The applet owns RC input;
there is no serial RC fallback. The `VM-ELRS-RX-NANO` hwdef includes this configuration.
Persist `MOD_ENABLE=1` to load the module at startup.

To bind:

1. Set `ELRS_BIND=1` and reboot, or hold the board's binding button during
   power-on if fitted. Release it when the binding indication starts.
2. Trigger Bind on the transmitter.
3. The receiver saves the learned identity and reboots automatically.

Binding uses DroneCAN maintenance mode and suspends RC output. Reboot to cancel
binding without changing the saved identity. Reconnection does not require rebinding.

## Parameters

The applet reserves dynamic table key 199 with prefix `ELRS_`. UID halves use
hidden Int32 storage, MODEL uses Int16, and BIND/VER use Int8. Parameters appear
only after the module starts. A prefix/key collision or invalid parameter stops
startup. UID parameters remain hidden from normal enumeration.

| Parameter | Default | Meaning |
| --- | --- | --- |
| ELRS_UID1 / UID2 | 0 | Binding UID halves, integer 0–16777215 |
| ELRS_MODEL | 255 | Model ID 0–255; 255 disables matching |
| ELRS_BIND | 0 | Set 1 and reboot to bind; consumed after version validation |
| ELRS_VER | 4 | Only compatibility version 4 is supported |

Parameter changes require a full reboot. Firmware updates retain the parameter
region. No parameter writes occur in the RF callback.

## Receiver behaviour

`RC_MSGRATE` limits DroneCAN RC output to 0–127 Hz (default 50); zero disables
output. RF rate and channel update rate differ from the CAN output rate.
Full16 output requires fresh CH1–8 and CH9–16. Stale channel sets are not
published. After previously valid input is unavailable for four seconds, the
receiver reports FAILSAFE with no channels. Flight-controller failsafe behaviour
requires separate validation.

Initialization and terminal radio faults report DroneCAN health ERROR. Binding,
searching and RF link loss do not themselves indicate a hardware fault.
Gemini, dual-radio diversity, sub-GHz radios and external PA/LNA switching are
not supported. Long-term stability and in-vehicle behaviour remain unvalidated.

## Code structure

| Layer | Source | Responsibility |
| --- | --- | --- |
| Module entry | `main.rs` | Configure parameters, register callback, process persistence requests and debug output on the module worker |
| Configuration | `parameters.rs` | Dynamic table ownership, binding UID persistence and reboot |
| Receiver policy | `receiver_control.rs` | Radio actions, binding, recovery, TX watchdog and LED policy |
| RF tracking | `tracker.rs` | Rate selection, synchronization, hopping, channel validity and telemetry scheduling |
| Wire protocol | `protocol.rs` | Packet encoding/decoding and protocol primitives |
| Transport | `transport.rs`, `stream.rs`, `mavlink_crc.rs` | ELRS fragmentation, MAVLink framing/validation and flow control |
| Module boundary | `libraries/AP_Module/BindingPeriph.h`, `generator/periph_binding.py`, `generator/bindings.py` | Validated SDK calls, opaque configuration, owned report snapshots and callback registration |
| Callback lifetime | `libraries/AP_Module/Runtime.cpp` | Revoke configuration/callback ownership before module memory is released |
| Hardware executor | `libraries/AP_Radio/driver_sx1280_module.*` | Serialized RF callbacks, hardware actions and VirtualUART mailbox movement |
| Hardware driver | `libraries/AP_Radio/driver_sx1280*` excluding the executor | SX1280 SPI/GPIO/IRQ operations |
| UART abstraction | `libraries/AP_HAL/utility/VirtualUARTDriver.*` | Buffered virtual serial device |
| CAN integration | `Tools/AP_Periph/module_rc.cpp`, `rc_in.cpp`, `can.cpp` | RC publication pacing, generic status and DroneCAN encoding |
| Publication protection | `libraries/AP_Module/RCInputSource.h` | Frame validation and host watchdog surviving module exit |
| Target configuration | `libraries/AP_HAL_ChibiOS/hwdef/VM-ELRS-RX-NANO/` | Pin mapping, module enablement, dynamic parameters, SPI mode and ROMFS embedding |

Paths beginning with `libraries/` or `Tools/` above are repository-relative.

The module worker may call SDK parameter/storage/reboot services. The RF callback
must remain bounded and must not call SDK services, allocate or panic. The
executor owns hardware access; the AP_Periph main loop consumes reports and
publishes CAN messages. The publication watchdog remains host-owned so a module
exit cannot leave an indefinitely valid RC frame.

## Build and deployment

The board hwdef owns the complete module configuration; no `--extra-hwdef` is
needed. Waf embeds the existing `.apmod` file and rejects a missing module or one
older than the applet, SDK or generator sources. Always rebuild the applet before
building firmware, including after any Rust source or SDK generator change.

From the repository root, with the Rust, ARM and Python toolchains configured:

```sh
python Tools/modules/build.py libraries/AP_Module/applets/elrs/main.rs \
  build/modules/l431/elrs_full.apmod --target thumbv7em-none-eabihf \
  --cpu cortex-m4 --stack-size 1536 --heap-size 256 --opt-level 3
./waf configure --board VM-ELRS-RX-NANO --out build/elrs-rust-full \
  --debug-symbols
./waf AP_Periph
```

Debug UART output is disabled by the compile-time `DEBUG` constant in `main.rs`.
Set it to `true` to include startup diagnostics and callback statistics once per
second, then rebuild and flash the firmware. With the default `false`, the
optimized build removes the diagnostic code and strings. No runtime parameter
or additional build option is required. CAN fault and recovery messages remain
enabled independently. The `--debug` build option controls debug symbols and
does not enable UART output.

The `.apmod` is embedded at `modules/elrs_full.apmod` in ROMFS. Persist
`MOD_ENABLE=1` to enable loading. Upgrade by rebuilding and flashing the whole
firmware.
Module hot reload is not
validated on L431; use a full reboot after firmware or parameter changes.

## MAVLink telemetry

Route a virtual UART to the SX1280 module executor using `AP_RADIO_SX1280_UART`. On `VM-ELRS-RX-NANO`, `VUART1` is `SERIAL2`: configure the flight
controller's DroneCAN serial tunnel with the receiver node ID, port index `2`
and MAVLink2, then select MAVLink link mode on the transmitter.

The receiver forwards byte streams in both directions using ELRS fragmentation
and ACKs. Uplink data is reassembled and passed through the virtual UART to the
existing DroneCAN tunnel; MAVLink messages are interpreted by the endpoints. The
official transmitter converts supported MAVLink messages to radio telemetry.
For its default relative-altitude sensor, send both `GLOBAL_POSITION_INT` and
`GPS_RAW_INT`. Match the flight controller's stream rate to the RF capacity;
excess data can overflow the UART queue. Pending ELRS transfers and mailbox data
are discarded on disconnects. Uplink forwarding retains partial UART writes and
stops accepting new transfers while its completed-transfer buffer is full.

MAVLink forwarding remains experimental and requires further validation before
use on a vehicle.
## MAVLink dialect

`mavlink_crc.rs` contains CRC extras from the repository's generated ardupilotmega
dialect. Regenerate it with `generate_mavlink_crc.py PATH/ardupilotmega.h` when
updating the MAVLink dialect. The parser preserves MAVLink 1/2 wire bytes and
signatures. It checks CRC but does not authenticate signatures. Frames with unknown message IDs are discarded.

## L431 timing

The L431 hwdef enables `HAL_SPI_USE_POLLED` for the short SX1280
transactions. Driver settling waits of at most 100 us use bounded polling;
longer waits yield.
These choices reduce DMA/scheduler jitter while keeping the 500 us F1000
service limit intact.

## Upstream reference

Based on [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) (GPLv3):

- Development baseline: `9ff3fa4cdb184c92eea95e9db5cfe231da3714da`.
- Release comparison: `4.1.0`, commit `a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`.

These are fixed references, not a guarantee of compatibility with future
ExpressLRS releases. Upstream SX1280 support also includes Semtech Revised BSD
sources; original copyright and license notices must be retained.

Development was AI-assisted.

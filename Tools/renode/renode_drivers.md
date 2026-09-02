# Renode I2C and serial driver coverage

This is the working plan and progress ledger for making ArduPilot physical I2C
and serial peripherals available to Renode simulations. Update it whenever a
driver changes coverage level, a protocol limitation is found, or a development
stage is completed.

## Coverage levels

1. **Inventoried**: production backend and selection mechanism are known.
2. **Modelled**: Renode implements the required electrical transactions.
3. **Detected**: the production ChibiOS ArduPilot backend initialises it.
4. **Dynamic**: reported values follow Renode physics or explicit model input.
5. **Fault tested**: timeout, corrupt data and detach behavior are verified.
6. **Runtime recovered**: reattachment works without reboot where ArduPilot has
   a reprobe or backend-reinitialisation path.

Only devices at level 4 or above should be presented as fully supported. A
detection-only model must be labelled as such in the launcher.

## Scope rules

- The inventory covers physical I2C and UART device backends. CAN, SPI, analog,
  PWM and timer-pin devices are tracked separately.
- MAVLink, PPP, SLCAN, scripting serial and similar link services are serial
  protocol classifications, not selectable physical sensor models.
- Fixed onboard devices come from `hwdef.dat` and are emulated automatically.
  The Config tab is primarily for external devices which the selected firmware
  can probe or instantiate.
- Electrical hotplug and driver rediscovery are separate. Renode can always
  remove a device electrically, but some ArduPilot drivers only probe at boot.
  Large changes to production drivers solely to add reprobe are out of scope.
- Runtime artifacts must work in the precompiled Windows bundle without an
  ArduPilot checkout. Prefer C# Renode models; compiled sidecars are acceptable
  only for protocols which cannot reasonably live in-process.
- Existing SITL models are protocol references and golden-vector oracles. Do
  not link the installer to the full SITL library dependency graph.

## Inventory baseline

Baseline recorded on 2026-08-27 with:

```sh
Tools/renode/driver_inventory.py --check
```

The source scanner currently reports:

- 51 `AP_SerialManager` protocol names, including aliases and internal/link
  protocols.
- 80 production source candidates which directly acquire a UART.
- 49 production source candidates which directly acquire an I2C device.
- 13 barometer, 14 compass and 2 IMU I2C probe families used by ChibiOS
  hwdefs.
- 15 concrete Renode I2C transport models and 5 concrete UART transport
  models, including models which inherit their transport implementation.
- Nine launcher catalog entries: five I2C, three UART and one CAN.
- Current I2C coverage: five dynamic selectable models.
- Current UART coverage: three dynamic selectable models.

Source-candidate counts are deliberately not called driver counts. Frontends,
shared helpers and multi-variant backends need classification in the reviewed
catalog before the exact supported-driver total is meaningful.

## Architecture decisions

- `driver_catalog.py` is now the source of truth for attachable models.
  `gen_board.py` and the launcher continue to expose `ATTACHABLE_DEVICES` for
  compatibility.
- Every catalog entry records its production backend, feature define, current
  coverage, expected hotplug mechanism and either a parameter recipe or an
  auto-probe selection rule.
- `driver_inventory.py` scans the production tree and validates that catalog
  models implement the declared Renode transport and reference real backend
  sources. It also rejects stale feature defines, malformed recipes and recipe
  placeholders which do not match the selected bus.
- The catalog supports shared physics sources with bounded per-device channel
  indices. Later schema revisions still need device variants, configurable
  addresses and declared observables. Probe profiles and initial multi-instance
  airspeed naming are implemented; other libraries still need explicit
  treatment because they do not share one convention (`GPS1_`, `RNGFND1_`,
  `ARSPD_`/`ARSPD2_`).
- The same catalog will generate the Config-tab choices, documentation and CI
  test matrix. These must not become separate hand-maintained lists.

## Driver groups

### I2C

- Barometers: BMP085/280/388/581, DPS280/310/SPL06, MS56xx, ICP101xx/201xx,
  ICM20789, LPS2xH, FBM320, Keller and AUAV.
- Compasses: AK8963/AK09916, BMM150/350, HMC5843, IIS2MDC/LIS2MDL/LIS3MDL,
  IST8308/8310, LSM303/LSM9, MAG3110, MMC families, QMC5883L/P and RM3100.
- Airspeed: MS4525, DLVR, MS5525, SDP3X, ASP5033 and AUAV.
- Rangefinder: MaxSonar, PulsedLight/LidarLite, LightWare, TeraRanger,
  Benewake, VL53L0X/L1X and TOFSense-F.
- Power: INA2xx/239/3221, LTC2946, AD7091R5, SMBus battery variants and
  BQ76952.
- Temperature: MCP9600, MLX90614, SHT3x, TMP119 and TSYS01/03.
- Other: PX4Flow, IRLock, LED/display controllers, DACs and I2C IMUs.

### Serial

- GNSS: UBX, NMEA variants, SIRF, SBP/SBP2, SBF, GSOF, ERB and NOVA.
- Rangefinder and proximity lidar families.
- NMEA airspeed, wind, AIS and beacon devices.
- External AHRS: VectorNav, MicroStrain, InertialLabs, SBG, Aeron and
  SensAItion.
- EFI, generators, fuel cells and ADS-B transponders.
- Mounts, gimbals, servos, ESCs, Torqeedo, cameras and video transmitters.
- RC and telemetry: FrSky, CRSF/ELRS, GHST, FPort, SRXL2, SBUS, HoTT, Devo,
  LTM, MSP and iBus.
- Output-only protocols require a receiver/analyser model rather than a sensor
  model.

## Per-driver acceptance test

Each device must eventually pass this sequence against a real ChibiOS vehicle
firmware backend:

1. Attach before boot and apply its parameter recipe.
2. Verify identification and initialisation.
3. Verify configuration writes or commands sent by ArduPilot.
4. Feed a known stable value and verify the frontend output.
5. Step the input and verify units, sign, orientation, latency and update rate.
6. Inject silence, NACK, truncation or checksum failure as applicable.
7. Verify unhealthy and recovery behavior.
8. Detach and verify timeout behavior.
9. Reattach at runtime when the production backend supports rediscovery.

Tests retain firmware identity, parameters, Renode output, bus traces, MAVLink
telemetry and DataFlash logs.

## Implementation stages

| Stage | Work | Status |
| --- | --- | --- |
| 1 | Central catalog, source inventory, validation and progress ledger | Complete |
| 2 | Reusable I2C/UART model primitives and driver-test harness | Complete |
| 3 | Expose and fully validate all existing Renode sensor models | In progress |
| 4 | Environmental/navigation I2C drivers | Not started |
| 5 | Remaining I2C power, output and high-rate devices | Not started |
| 6 | Serial navigation, ranging and proximity devices | Not started |
| 7 | Complex bidirectional and output-only serial devices | Not started |
| 8 | Full fault, hotplug, Windows bundle and nightly CI matrix | Not started |

## Progress log

### 2026-08-27

- Moved the existing attachable-device definitions into
  `Tools/renode/driver_catalog.py` without changing their launcher IDs or
  generated Renode models.
- Added honest metadata for backend source, compile-time feature, coverage and
  expected hotplug path. AUAV and ASP5033 started as detection-only; DroneCAN
  airspeed remains detection-only.
- Added `Tools/renode/driver_inventory.py` to discover serial protocols,
  direct production bus users, I2C hwdef probe families and Renode transport
  models.
- Added catalog validation and live-tree inventory tests.
- Classified all 51 real `AP_SerialManager` protocol entries as devices,
  outputs, generic links, internal links, aliases or disabled. Comments and the
  enum sentinel are deliberately excluded from the inventory.
- Classified every I2C barometer, compass and IMU family currently referenced
  by a ChibiOS hwdef as modelled or planned. CI now detects new and stale
  classifications.
- Classified all 49 sources which directly acquire an I2C device as a concrete
  device, frontend, reusable base or bus-access service. This avoids treating
  scanner candidates as 49 independently selectable drivers.
- Added initial parameter recipes for the seven non-CAN catalog devices and a
  resolver for concrete `SERIALn` and `I2C_ORDER` indices. Recipes are metadata
  only at this stage; the launcher does not silently change firmware parameters.
- Added catalog checks for real compile-time feature defines, recipe structure,
  duplicate parameters and bus-specific placeholders. The complete recipe and
  model metadata is now available in JSON inventory output.
- Added the runtime catalog to the source-independent bundle file list.
- Completed the Stage 1 foundation. This is inventory infrastructure, not a
  claim that the listed production drivers have passed the acceptance test.
- Added `Tools/renode/tests/test_driver_probe.py`, the first real-firmware
  driver probe harness. It boots MatekH743 ArduPlane with catalog attachments
  and requires production-backend evidence rather than generic sensor health.
- Ran the harness against the locally built MatekH743 ArduPlane ELF. ArduPilot
  announced `GPS 1: detected u-blox`, reported a 3D fix, and exposed the
  IST8310 as an I2C device at address `0x0E` with compass device type `0x0A`.
  The artifacts are under `build/renode-test/driver-probe-stage2/`.
- Added a controlled in-process physics provider to the probe harness. A
  real-firmware run verified baseline GPS and compass values, then stepped GPS
  from `-35.3632610,149.1652300,584m` to
  `-35.3622610,149.1642300,614m` and compass input from
  `(201,0,450)` to `(350,-120,80)` mG. The frontend reported
  `(351,-120,81)` mG after quantisation. Artifacts are under
  `build/renode-test/driver-probe-dynamic-stage2/`.
- Made that scenario the `matekh743-navigation` catalog probe profile. Profile
  validation checks its board, firmware defaults, attachments, assertions and
  parameter recipes. Rebuilt firmware with forced u-blox type and 230400 baud;
  the catalog-driven run passed under
  `build/renode-test/driver-probe-catalog-stage2/`.
- Extended `physics_stub.serve_connection()` with an injectable truth provider
  so driver tests can step inputs synchronously without a full vehicle model.
- Extracted the common 8-bit I2C register-pointer behavior into
  `peripherals/common/AP_I2CRegisterDevice.cs` and migrated the dynamic IST8310
  plus existing register barometers to it. The post-refactor production-driver
  probe passed under `build/renode-test/driver-probe-i2c-base-stage2/`.
- Fixed inventory discovery to follow C# transport inheritance, recognise
  buffered UART implementations and exclude abstract base classes.
- Added transaction counters, last-register observables, ignored-write faults
  and read corruption to the common I2C register model. The first assertion
  incorrectly expected corrupt samples in `RAW_IMU`; the production compass
  correctly rejected them and retained its last valid field. The final test
  instead verifies the intended behavior: compass health clears during
  out-of-range corruption, recovers when the fault is removed, and correct
  samples resume. The passing artifacts are under
  `build/renode-test/driver-probe-i2c-monitor-path-stage2/`.
- Added `AP_UARTFrameDevice`, which paces bytes at the declared baud rate and
  exposes frame/byte counts, firmware writes, output suppression and byte
  corruption. Migrated the Benewake and LightWare serial models to it.
- Added the catalog-driven `matekh743-rangefinders` profile with independent
  parameter recipes on `SERIAL4` and `SERIAL5`. A rebuilt ArduPlane firmware
  reported 5.00 m from both production backends as `DISTANCE_SENSOR` instances
  0 and 1. Artifacts are under
  `build/renode-test/driver-probe-rangefinders-stage2/`.
- Completed Stage 2: both transport primitives compile in all supported MCU
  scripts, catalog profiles drive the real-firmware harness, and stable,
  stepped and corrupt/recovery assertions are available.
- Added the `matekh743-airspeed` Stage 3 profile for the production MS4525
  backend on I2C bus 1. The rebuilt firmware reported device type `0x02` at
  address `0x28`, a healthy 0.00 m/s baseline, and 20.80 to 21.54 m/s after a
  20 m/s physics input step (the expected pressure/density conversion is
  21.49 m/s). Invalid status bits made the frontend unhealthy; clearing the
  fault restored both health and the stepped value. Passing artifacts are
  under `build/renode-test/driver-probe-ms4525-stage3-final/`.
- The isolated airspeed defaults set `ARSPD_OPTIONS` to disable voltage
  correction because this protocol probe intentionally has no emulated 5 V
  rail measurement. This keeps the protocol conversion test independent of
  board power-monitor emulation.
- Extended profile validation to check I2C parameter recipes against the
  checked-in firmware defaults as well as UART recipes.
- Made the AUAV and ASP5033 models physics-driven and added protocol-specific
  read corruption plus transaction observables. The
  `matekh743-airspeeds` profile attaches both devices to I2C bus 1, exercising
  the `ARSPD` and `ARSPD2` production backends simultaneously. The rebuilt
  firmware reported AUAV type `0x0B` at `0x26` and ASP5033 type `0x0A` at
  `0x6C`; both reported 0.00 hPa at rest and 2.31 hPa for the 20 m/s physics
  input. Corrupting either device independently cleared aggregate airspeed
  health, and both recovered without reboot. Passing artifacts are under
  `build/renode-test/driver-probe-auav-asp5033-stage3-values/`.
- Added instance-aware `{airspeed_prefix}` parameter recipes so one profile can
  validate multiple airspeed devices on a single I2C bus.
- Connected the Benewake and LightWare UART models to selectable physics
  rangefinder channels while retaining a 5 m fallback before a physics source
  connects. Suppressing each UART model independently caused its
  `DISTANCE_SENSOR` stream to time out; restoring bytes recovered the same
  backend without reboot. Initial passing artifacts are under
  `build/renode-test/driver-probe-rangefinder-dynamic-stage3-fixed/`.
- Migrated the u-blox model to `AP_UARTFrameDevice` and added both silence and
  byte-corruption recovery assertions. After four seconds without valid UBX
  frames, the production GPS frontend deleted its backend and changed status
  to `NO_GPS`. Restoring valid frames caused a fresh u-blox probe, a healthy
  3D fix and recovery of the stepped location without reboot. The same path
  passed after every output byte was XOR-corrupted. Passing artifacts are
  under `build/renode-test/driver-probe-ublox-fault-stage3-health/`.
- All seven non-CAN launcher devices are now physics-driven and exercised
  against their production ChibiOS backends. The remaining launcher entry is
  the detection-only DroneCAN airspeed sidecar.
- Asked Claude to review the Stage 1 through Stage 3 work. Claude found that
  `RangefinderIndex` existed on both UART models but was not assigned by the
  generated platform, so both models silently consumed physics channel 0. The
  probe's repeated `(5.0,) * 10` and `(8.0,) * 10` fixtures masked the defect.
- Fixed that finding with catalog-declared physics source/property/count
  metadata. Boot-time generation and live hotplug now allocate and validate a
  unique physics channel and set the model property. The production probe now
  deliberately supplies 5/8 m on channel 0 and 6/9 m on channel 1; Benewake
  reported 500 to 800 cm while LightWare reported 600 to 900 cm, including
  independent suppression/recovery. Passing artifacts are under
  `build/renode-test/driver-probe-rangefinder-claude-fix/`.
- Claude also raised a lower-confidence concern that ASP5033 might combine
  pressure and temperature from different physics snapshots. The production
  backend reads both in one five-byte I2C transaction, causing one model
  `Read()` and one snapshot update, so no change was needed.
- Classified all 80 production sources which directly acquire or accept a
  UART. Each has a role (`device`, `frontend`, `base`, `output`, `link`,
  `service` or `internal`) and a device/protocol family. Validation now fails
  for new, stale or malformed direct-UART classifications. This converts the
  raw source list into a grouped implementation backlog without counting
  framework helpers as separate peripherals.
- Added the BMP280 as the first Stage 4 catalog device on a selectable external
  I2C bus. Migrated its physics-driven model onto `AP_I2CRegisterDevice`, while
  retaining its datasheet calibration and inverse compensation model. The
  MatekH743 production backend was identified as `BARO3` on I2C bus 1 at
  `0x76`, then reported 1013.25 hPa/20 C and a stepped 900.00 hPa/30 C.
- A first fault attempt XOR-corrupted BMP280 bytes, but the transformed samples
  remained changing and finite, so the production health logic correctly kept
  the sensor healthy. The final test freezes the sample registers, exercises
  ArduPilot's two-second stuck-sensor detection, then resumes samples and
  verifies health and values recover without reboot. Passing artifacts are
  under `build/renode-test/driver-probe-bmp280-stage4-final/`.

The current fast probe can be repeated without rebuilding firmware:

```sh
Tools/renode/tests/test_driver_probe.py \
    --skip-build \
    --renode build/renode/renode
```

Omit `--skip-build` to configure and build the MatekH743 firmware with the
checked-in test defaults first.

Next work:

1. Add catalog coverage and production probes for the remaining existing I2C
   barometer models before implementing the planned compass families.
2. Extend the manifest with explicit protocol variants and per-frontend
   multi-instance parameter naming where a family shares one model.
3. Make the DroneCAN airspeed sidecar physics-driven and fault-testable as part
   of the separate CAN coverage track.

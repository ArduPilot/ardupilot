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
- 87 concrete Renode I2C transport classes and 48 concrete UART transport
  classes, including address-specific classes and models which inherit their
  transport implementation.
- 131 launcher catalog entries: 82 I2C, 48 UART and one CAN.
- Current I2C coverage: 82 dynamic selectable attachments.
- Current UART coverage: 48 dynamic selectable models.

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
- Every source which directly acquires an I2C device and is classified as a
  concrete device must either back a selectable catalog attachment or have an
  explicit application boundary. This prevents platform-only code from
  appearing as an unexplained coverage gap.
- The catalog supports shared physics sources with bounded per-device channel
  indices. Later schema revisions still need device variants, configurable
  addresses and declared observables. Probe profiles and initial multi-instance
  airspeed naming are implemented; other libraries still need explicit
  treatment because they do not share one convention (`GPS1_`, `RNGFND1_`,
  `ARSPD_`/`ARSPD2_`).
- A logical I2C attachment may declare multiple model/address endpoints. The
  generator and runtime launcher reserve, attach and detach the full endpoint
  set together, while conflict checks reject an overlap with any member of the
  set. This is used for hardware which is sold and configured as one unit but
  exposes several independently addressed controllers.
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
| 3 | Expose and fully validate all existing Renode sensor models | Complete |
| 4 | Environmental/navigation I2C drivers | Complete |
| 5 | Remaining I2C power, output and high-rate devices | Complete |
| 6 | Serial navigation, ranging and proximity devices | Complete |
| 7 | Complex bidirectional and output-only serial devices | In progress |
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
- Added the externally selectable DPS280 variant using the existing shared
  DPS280/DPS310 register model. ArduPilot's `BARO_PROBE_EXT` path instantiates
  `AP_Baro_DPS280`; explicit hwdef entries may instead select
  `AP_Baro_DPS310`, so the Config entry does not mislabel the production
  backend. The model now uses the common I2C register base and exposes its
  data-ready failure mode. The external backend was identified as `BARO3` on
  bus 1, reported 1013.25 hPa/19.99 C and 900.00 hPa/29.99 C, became unhealthy
  when readiness was suppressed, performed its normal reset attempts, and
  recovered. Passing artifacts are under
  `build/renode-test/driver-probe-dps280-stage4-final/`.
- Made the MS5611 temperature conversion follow physics as well as its existing
  pressure conversion, by inverting the production driver's first-order
  compensation equation. Added a missing-ADC conversion fault and exposed the
  external MS5611 through the catalog. The production backend reported the
  same two pressure/temperature states, became unhealthy when ADC responses
  were zero, and recovered without reboot. Passing artifacts are under
  `build/renode-test/driver-probe-ms5611-stage4-final/`.
- The first MS5611 probe incorrectly matched MatekH743's onboard `BARO1`, since
  it shares the external sensor's address and device type. Barometer identity
  checks now require the selected I2C bus as well. The corrected run matched
  the attached `BARO3` on bus 1; this protects every future multi-instance
  barometer profile from the same false-positive value check.
- Added the BMP581 on its selectable external I2C bus. The old minimal model
  advertised status `0x60`, which cannot pass the production backend's
  power-on-ready check; status `0x02` is the required ready state. Its sample
  registers now follow physics, and one sample is held across the driver's two
  consecutive block reads because the backend rejects non-identical reads.
  The production backend found `BARO3` on bus 1 at `0x46`, reported
  1013.25 hPa/20 C and 900.00 hPa/30 C, became unhealthy when invalid sample
  bytes were injected, and recovered without reboot. Passing artifacts are
  under `build/renode-test/driver-probe-bmp581-stage4-final/`.
- Migrated BMP388 to the common I2C register model and made its deliberately
  simple calibration invert physics truth. The first production run detected
  the backend but exposed a migration error: calibration words had changed
  from sensor-native little-endian to the base class's big-endian helper. With
  the correct helper, `BARO3` on bus 1 at `0x76` reported both truth points,
  became unhealthy when data-ready was suppressed, and recovered. Artifacts
  are under `build/renode-test/driver-probe-bmp388-stage4-final/`.
- Made the LPS22H variant of the shared LPS2xH model physics-driven and
  selectable at `0x5D`. Its direct pressure and temperature scaling reported
  both truth points through the production LPS2xH backend; status suppression
  caused runtime loss and recovery. Artifacts are under
  `build/renode-test/driver-probe-lps2xh-stage4-final/`.
- Replaced BMP085's fixed datasheet conversion with valid simplified
  calibration and a numerical inverse of ArduPilot's integer compensation
  routine. This exercises its command-driven temperature/pressure state
  machine. The backend reported both truth points, rejected zero ADC data and
  recovered; artifacts are under
  `build/renode-test/driver-probe-bmp085-stage4/`.
- Corrected the shared DPS/SPL model for SPL06. The old model lacked the two
  startup-ready bits required by SPL06 and generated raw values only for the
  DPS280 16x oversampling scale. It now selects the production scale from the
  pressure configuration register while retaining DPS280 behavior. SPL06
  reported both truth points through `BARO3` on bus 1 at `0x76`. Its backend
  checks ready bits only during initialization, so the initial readiness fault
  was replaced with zero sample injection for meaningful runtime failure and
  recovery. Artifacts are under
  `build/renode-test/driver-probe-spl06-stage4-final/`.
- Made the absolute-pressure half of the AUAV model physics-driven and exposed
  it separately from the differential-pressure airspeed device. The
  production AUAV barometer reported 1013.25 hPa/19.99 C and 900.00 hPa/29.99
  C on I2C bus 1 at `0x27`; corrupt status bits made it unhealthy and clearing
  the fault recovered it. Artifacts are under
  `build/renode-test/driver-probe-auav-baro-stage3/`.
- Replaced the ICP201xx model's fixed FIFO packet with signed 20-bit pressure
  and temperature samples derived from physics. ICP201xx has no external probe
  bit, so its catalog entry is explicitly `hwdef-probe` and its probe profile
  supplies a test-only hwdef declaration. The production backend reported both
  truth points, became unhealthy when FIFO data was suppressed and recovered.
  Artifacts are under
  `build/renode-test/driver-probe-icp201xx-stage3/`.
- Exposed the existing register-only Invensense I2C model as an MPU6000 with
  detection-only coverage. A first MatekH743 attempt was invalid because its
  three native IMUs already consume all normal frontend instances. The final
  KakuteF4 test added the I2C backend beside its single native IMU and verified
  both production accelerometer type `0x13` and gyroscope type `0x21` device
  IDs on bus 0 at `0x68`. The model still has an empty FIFO; physics-driven
  high-rate samples and fault recovery remain Stage 5 work. Passing artifacts
  are under
  `build/renode-test/driver-probe-invensense-kakutef4-stage3/`.
- Completed Stage 3. Every pre-existing selectable I2C and UART sensor model is
  now in the catalog and has been exercised against its production ChibiOS
  backend at its declared coverage level. Detection-only entries remain
  visibly distinct from fully supported dynamic models.
- Began the Stage 4 compass expansion with physics-driven IIS2MDC/LIS2MDL,
  IST8308, LIS3MDL, QMC5883L, QMC5883P and RM3100 models. Each model presents
  the identification, status and sample register behaviour used by its
  production backend, including the backend-specific sensor-frame transform.
  The six MatekH743 probes verified device IDs, two magnetic-field truth
  points, loss of frontend health when data-ready was suppressed, and runtime
  recovery. Artifacts are under the corresponding
  `build/renode-test/driver-probe-*-stage4*` directories.
- Added the second Stage 4 compass tranche: standalone AK09916 and AK8963,
  HMC5843/HMC5883 and MMC3416. The HMC model supplies valid startup
  self-calibration samples before switching to physics data, while MMC3416
  implements the production driver's refill, SET/RESET and repeated
  measurement sequence. All four production backends reported both magnetic
  truth points, became unhealthy under suppressed data and recovered. AK09916,
  HMC5843 and MMC3416 use external auto-probing; AK8963 uses its supported
  direct-I2C hwdef probe form. Artifacts are under the corresponding
  `build/renode-test/driver-probe-*-stage4*` directories.
- Completed the Bosch compass tranche with BMM150 and BMM350. BMM150 exposes
  a neutral, internally consistent factory-trim set and inverts the production
  integer compensation functions for physics samples. BMM350 implements its
  two-dummy-byte I2C reads, 32-word OTP sequence and PMU command status before
  applying neutral OTP compensation to dynamic samples. Both production
  backends reported the two magnetic truth points. BMM150 also passed
  data-ready loss and recovery; BMM350 has no runtime validity flag or
  zero-sample rejection, so its transport-loss behavior is deferred to the
  detach/hotplug fault stage. Artifacts are under
  `build/renode-test/driver-probe-bmm*-stage4*/`.
- Completed the remaining direct-I2C compass tranche with LSM303D, LSM9DS1,
  MMC5983 and MAG3110. The LSM models mask their SPI command bits on I2C in
  the same way as the production device layer. LSM9DS1 shares both address
  and identity byte with LIS3MDL, so its explicit test firmware disables the
  earlier LIS3MDL external auto-probe before using its hwdef probe. MMC5983
  implements the production SET, RESET and trigger sequence.
- The MAG3110 model follows the production backend's 1000 milligauss/LSB
  conversion, although the NXP data sheet specifies 0.1 microtesla/LSB, or 1
  milligauss/LSB. Its probe therefore uses representable 1-gauss-step magnetic
  truth values. All four production probes reported both magnetic truth
  points, became unhealthy when samples were suppressed and recovered without
  reboot.
  Artifacts are under the corresponding
  `build/renode-test/driver-probe-*-stage4*` directories.
- Added physics-driven MCP9600, MLX90614 and TMP119 temperature models. A
  MatekH743 test build enables the non-dummy production temperature frontend
  and routes its three instances to separate battery-temperature telemetry
  channels. This proves each configured backend independently rather than
  accepting a register-level response. All three reported 20 C, followed the
  physics step to 30 C, and remained within their protocol quantisation.
  Temperature-backend health is not exposed in MAVLink, and externally
  injected battery temperature intentionally retains its last value, so
  timeout/recovery will be checked through `TEMP` logs in the later detach and
  hotplug stage. Passing artifacts are under
  `build/renode-test/driver-probe-temperature-basic-stage4-fixed/`.
- Added the command-based SHT3x, TSYS01 and TSYS03 temperature models. SHT3x
  implements serial-number and measurement commands plus both CRC-protected
  words. TSYS01 supplies a valid PROM calibration and inverts the production
  polynomial to encode physics temperature in its 24-bit ADC. TSYS03 encodes
  its direct 16-bit conversion and matching CRC. The three production
  backends independently reported 20 C and followed the step to 30 C through
  separate telemetry channels. Passing artifacts are under
  `build/renode-test/driver-probe-temperature-command-stage4/`.
- Completed the direct-I2C airspeed set with DLVR, MS5525 and SDP3X. DLVR uses
  the existing four-byte pressure frame with its five-inch transfer function.
  MS5525 supplies a CRC-valid coefficient PROM and inverts the production
  compensation equations for its alternating 24-bit conversions. SDP3X
  implements its continuous-measurement command and CRC-protected words, and
  inversely applies the production pitot correction so the frontend receives
  the shared ideal physics pressure. All three production backends were found
  on I2C bus 1, reported zero and the 20 m/s pressure step, became unhealthy
  under protocol-specific invalid data, and recovered without reboot.
  Artifacts are under `build/renode-test/driver-probe-{dlvr,ms5525,sdp3x}-stage4*/`.
- Updated the probe's expected airspeed pressure to use the explicitly injected
  pressure and temperature. The older ISA-altitude calculation happened to
  agree with earlier fixtures but became incorrect once environmental probes
  began deliberately stepping those fields independently.
- Claude's stage review found that the SDP3X inverse correction used the
  standard 287.05 J/(kg K) dry-air constant while the production backend uses
  ArduPilot's 287.26 `ISA_GAS_CONSTANT`. The model now matches the backend
  exactly; no other airspeed-stage issues were found.
- Added physics-driven I2C models for MaxSonar I2CXL, Benewake TFMini Plus,
  Benewake TFS20-L, TeraRanger Evo, Nooploop TOFSense F and LightWare GRF.
  The first five non-conflicting addresses plus GRF share one production probe;
  TFMini Plus has a separate profile because it and TFS20-L both use address
  `0x10`. Every backend reported its independently indexed baseline distance
  and the 3 m step. TeraRanger CRC corruption, TOFSense status invalidation and
  GRF zero-distance rejection each removed their `DISTANCE_SENSOR` stream and
  recovered at runtime. Passing artifacts are under
  `build/renode-test/driver-probe-{i2c-rangefinders,tfmini-plus-i2c}-stage4/`.
- Claude reviewed all six command layouts, endian conversions, checksums,
  addresses, type recipes and instance routing and found no actionable issues.
- RangeFinder explicitly documents that it does not rediscover backends after
  `init()`, so these catalog entries accurately retain `boot-probe` hotplug
  metadata. Runtime detach/reattach can test loss behavior, but true production
  rediscovery requires an upstream frontend change and is not claimed here.
- Completed the selectable I2C rangefinder set with legacy LightWare,
  LidarLite V3, VL53L0X and short-range VL53L1X models. These exercise the
  production drivers' fallback protocol detection, register auto-increment,
  paged register state, SPAD setup and full VL53L1X startup sequence rather
  than bypassing initialization. The VL53 devices provide controllable
  data-ready suppression; both stopped publishing and recovered at runtime.
- The rangefinder physics fixture now uses 2.0--2.9 m at baseline and
  3.0--3.9 m after the step so every instance remains inside the VL53L1X
  short-range limit. A one-centimetre comparison tolerance accounts for the
  conversion of decimal metre inputs through single-precision physics values.
  The stateful, isolated VL53L1X, TFMini Plus I2C and serial rangefinder
  production probes all passed with the shared fixture. Artifacts are under
  `build/renode-test/driver-probe-{i2c-rangefinders-stateful-pass,vl53l1x-stage4,tfmini-plus-i2c-stage4-short-fixture,rangefinder-short-fixture-regression}/`.
- Claude reviewed the fallback, register paging, auto-increment, gain inversion,
  catalog wiring and test tolerance against all four production backends and
  found no actionable issues.
- Completed the environmental I2C stage with the Keller 4LD--9LD backend.
  Keller is a subsea absolute-pressure barometer, rather than an airspeed
  differential-pressure sensor, so its production profile uses ArduSub. The
  model supplies command-specific metadata and 0--2 bar calibration replies,
  then converts physics pressure and temperature into measurement frames. The
  production backend was identified as `BARO3` on I2C bus 1 at `0x40`, reported
  1013.24 hPa/20 C and the 900.02 hPa/30 C step, became unhealthy under a
  checksum-error status, and recovered. Artifacts are under
  `build/renode-test/driver-probe-keller-stage4-run3/`.
- Claude reviewed the calibration float layout, PAA offset semantics, inverse
  conversions, status mask, device identity, catalog recipe and ArduSub probe
  wiring and found no actionable issues.
- Began Stage 5 with all five devices selected by the shared INA2xx battery
  backend: INA226, INA228, INA238, INA231 and INA260. Each catalog choice has
  its own identification/register-width model while sharing the production
  `BATT_MONITOR=21` recipe. The recipe resolver now handles `BATT`, `BATT2`,
  and later battery-instance prefixes without hard-coding the first instance.
- All five production detection paths reported 12 V/3 A and followed the
  physics step to 24 V/12 A; the INA228 and INA238 paths also validated their
  die-temperature scaling. Their backend deliberately retries detection while
  absent and after transfer failures, so electrical detach and runtime
  re-detection will be exercised in the common Stage 8 transport test rather
  than approximated with valid-but-bogus sample bytes. Artifacts are under
  `build/renode-test/driver-probe-ina{226,228,238,231,260}-stage5/`.
- Claude found that INA238 temperature values should have their reserved low
  four register bits cleared. The model now rounds temperature to the device's
  0.125 C effective resolution and shifts it into bits 15:4. The suggested
  direct multiplication was not used because it would double-apply the scale;
  the corrected quantisation was checked against the production mask.
- Added the three-channel INA3221 as one physical I2C model rather than three
  conflicting devices at address `0x40`. A test-only hwdef enables its backend,
  which is normally SITL-only, without changing production feature defaults.
  Three battery instances share the backend's address driver and select
  channels 1, 2 and 3; all independently reported 12 V/3 A and the 24 V/12 A
  step through `BATTERY_STATUS`. The probe-profile schema can now declare
  multiple battery instances served by one physical model. INA3221 probes only
  during battery initialization, so transport loss remains a Stage 8 detach
  test and runtime rediscovery is not claimed. Artifacts are under
  `build/renode-test/driver-probe-ina3221-stage5/`.
- Claude's stage review found that the three profile instances exercised the
  catalog recipe only indirectly. The recipe now derives `BATT_CHANNEL` from
  the battery instance, unit tests cover instances 1 through 3, and profile
  validation checks every declared battery instance against its defaults.
- Added a physics-driven LTC2946 model at its board-defined address `0x6a`.
  Its identity byte, control writes and big-endian left-aligned 12-bit voltage
  and current ADC registers follow the production backend's fixed 500 microohm
  shunt scaling. A test-only hwdef supplies the compile-time bus and address
  macros required to include the backend without changing normal board builds.
  The real MatekH743 backend followed the physics step from 12.002 V/3.00 A to
  24.005 V/12.00 A; artifacts are under
  `build/renode-test/driver-probe-ltc2946-stage5/`. Claude's stage review found
  no actionable issues and confirmed that omitting bus/address parameters from
  the catalog recipe accurately represents this compile-time-only backend.
- Added the AD7091R5 four-channel ADC at its production-fixed bus 0/address
  `0x2f`. The model emits the backend's eight-byte channel scan. The initial
  profile maps channel 0 to battery voltage with a 10:1 divider and channel 1
  to current at 10 A/V; unused channels remain valid zero samples. Claude's
  stage review identified that a single-instance probe did not exercise the
  backend's static first-owner design. The profile now configures three battery
  frontend instances against the same physical channel pair, covering both the
  owning instance and the two shared-buffer consumers. Independent physics for
  alternative channel mappings remains future expansion, not claimed coverage.
  All three instances reported 11.996 V/2.99 A and then 24.000 V/11.99 A;
  artifacts are under
  `build/renode-test/driver-probe-ad7091r5-multi-stage5/`.
- Added a generic smart-battery model with SMBus PEC, little-endian word
  registers, four cell voltages, signed discharge current, temperature,
  capacities, serial number and cycle count. The profile uses the standard
  external bus/address parameters and checks pack voltage by summing cells.
  The real backend reported 12.000 V/3.00 A and 24.000 V/12.00 A with the
  expected temperatures; artifacts are under
  `build/renode-test/driver-probe-smbus-generic-stage5/`. Claude's stage review
  found no actionable issues. Corrupt-PEC, detach and runtime recovery testing
  remains explicitly deferred to Stage 8.
- Added Maxell and Rotoye variants of the generic smart battery. Maxell returns
  the exact manufacturer string which selects the production backend's no-PEC
  exception, while Rotoye supplies both its internal and external temperature
  registers over the normal PEC path. Claude's review found that the initial
  assertions did not distinguish the capacity and dual-temperature overrides.
  The Maxell profile now requires the backend's 2x scaler to rewrite the
  advertised 5,000 mAh capacity to 10,000 mAh. Rotoye reports its internal
  sensor 2 C below its external sensor so matching physics truth proves the
  backend selected the warmer value. Passing artifacts are under
  `build/renode-test/driver-probe-smbus-maxell-reviewed-stage5/` and
  `build/renode-test/driver-probe-smbus-rotoye-reviewed-stage5/`.
- Added a dedicated NeoDesign smart-battery variant. It requires PEC from the
  first transaction, reports a four-cell count at `0x5c`, serves consecutive
  cell registers from `0x30`, and lets the production backend derive pack
  voltage from their sum. The real backend reported 12.000 V/3.00 A and
  24.000 V/12.00 A; artifacts are under
  `build/renode-test/driver-probe-smbus-neodesign-stage5/`. Claude found no
  correctness issue, but identified the over-10-cell clamp and forced
  cell-read fallback as untested negative paths. Both are recorded for the
  Stage 8 SMBus fault-injection matrix rather than changing the normal model.
- Added separate SUI3 and SUI6 models. Both implement the length-prefixed
  four-cell voltage block and bare signed 32-bit current register. SUI6 also
  supplies pack voltage so the production backend reconstructs its remaining
  two cells; SUI3 uses only the first three values from the shared block.
  Claude found that uniform SUI6 cells did not discriminate the reconstruction
  arithmetic. Its direct cells now carry 10%, 12%, 14% and 16% of pack voltage,
  and the probe requires the backend to derive two 24% cells independently.
  Passing artifacts are under
  `build/renode-test/driver-probe-smbus-sui3-stage5/` and
  `build/renode-test/driver-probe-smbus-sui6-reviewed-stage5/`.
- Added the Solo smart battery with PEC-protected legacy and extended cell
  blocks, block-prefixed signed 32-bit current and manufacturer/button data. A
  test-only hwdef enables its normally board-selected backend. The legacy
  block deliberately totals half the pack voltage, while the extended block
  exposes six cells totalling the full value so telemetry proves the backend
  switched formats. The real backend reported 12.000 V/3.00 A and 24.000
  V/12.00 A with all six cells checked; artifacts are under
  `build/renode-test/driver-probe-smbus-solo-stage5/`. Claude found no
  actionable issues.
- Added a six-cell TI BQ76952 BMS model at its backend-fixed bus 0/address
  `0x08`. With configuration updates disabled, it implements the indirect
  device-number handshake and direct alarm/status, stack/pack voltage, cell,
  signed-current and internal-temperature commands. A test-only hwdef enables
  the normally AP_Periph-selected backend without changing production builds.
  The real backend reported 12.000 V/3.00 A and 24.000 V/12.00 A with all six
  cells checked; artifacts are under
  `build/renode-test/driver-probe-bq76952-stage5/`. Claude's review found no
  actionable issues. Configuration-update register programming remains outside
  this initial measurement-path coverage.
- Upgraded the I2C MPU6000 from detection-only to physics-driven FIFO coverage.
  The model now preserves configuration-register readback, handles FIFO and
  device resets, and supplies big-endian acceleration, temperature and gyro
  records with the production backend's sensor-frame ordering and scaling. On
  KakuteF4 the production driver registered the attached bus-0 device as IMU2;
  it reported the two nonuniform truth vectors within quantisation tolerance,
  including 20 C and 30 C temperature points. Artifacts are under
  `build/renode-test/driver-probe-invensense-i2c-stage5/`. Claude's review
  found no actionable issues.
- Added observable models for the IS31FL3195, LP5562, NCP5623, PCA9685 and
  TCA62724 I2C RGB LED controllers. Each exposes its initialized state and
  decoded colour channels so the driver probe can send two MAVLink override
  colours and prove the production notification backend emitted the correct
  device-specific register sequence and quantisation. All five production
  backends passed both colour points; artifacts are under the corresponding
  `build/renode-test/driver-probe-*-led-stage5/` directories. Claude's review
  found no actionable issues.
- Added a physics-driven PX4Flow integral-frame model. It combines body-frame
  translational image motion with the physics angular rate, then emits the
  production backend's 100 ms little-endian integral packet with quality,
  ground distance and temperature metadata. The production backend reported
  `(0.600, 0.800)` and `(-0.650, -0.750)` rad/s at the two motion points;
  artifacts are under `build/renode-test/driver-probe-px4flow-stage5/`.
  Claude's review found no actionable issues and confirmed the motion formula
  matches ArduPilot's SITL optical-flow backend. Sensor-position offset motion
  remains future physics-protocol work.
- Added an IR-LOCK I2C byte-stream model with controllable target position,
  target size, frame suppression and checksum corruption. The production
  Copter backend was tested on MatekH743 with a LidarLite sharing I2C1 so its
  pixel measurements could be converted into logged target vectors at 2 m and
  3 m. The DataFlash check also proves corrupt frames leave `LastMeasMS`
  unchanged for more than one second and that measurements resume after the
  checksum is restored; bus health alone would not prove this because the
  production driver records successful transfers before validating checksums.
  Artifacts are under
  `build/renode-test/driver-probe-irlock-stage5-final/`.
  Claude independently reviewed the byte framing, checksum behavior, geometry,
  catalog recipe and log assertions and found no actionable issues.
- Added SSD1306 and SH1106 monochrome display models which interpret each
  controller's command/addressing protocol and reconstruct its display RAM.
  The production AP_Notify backends initialized both displays, drew 628 lit
  pixels while disarmed, and changed to a stable 692-pixel framebuffer after a
  force-arm command changed the screen page. Passing artifacts are under
  `build/renode-test/driver-probe-ssd1306-stage5-arm/` and
  `build/renode-test/driver-probe-sh1106-stage5/`.
  Claude's review found that the SSD1306 parser consumed the `0x20` memory-mode
  argument only by coincidence when its value was zero. The opcode is now
  explicitly classified as a one-argument command.
- Added a physics-driven Bosch BMI160 I2C IMU model. It implements the
  production backend's read-flag convention, reset/configuration register
  behavior and little-endian gyro-then-accelerometer FIFO records at the
  configured 2000 dps and 16 g scales. On KakuteF4 the production backend
  registered the bus-0/address-`0x68` device as IMU2 and followed two
  nonuniform motion vectors: `(127, -254, -764)` mg and
  `(120, -230, 339)` mrad/s at baseline, then `(-309, 438, -661)` mg and
  `(-309, 169, -88)` mrad/s after the physics step. Passing artifacts are
  under `build/renode-test/driver-probe-bmi160-i2c-stage5/`. Claude's review
  found no actionable issues and independently confirmed the I2C read-flag,
  FIFO ordering, byte order, scales, rotation and device-ID assertions against
  the production backend.
- Added a physics-driven Bosch BMI270 I2C IMU model. It handles the production
  backend's config-blob upload handshake and its bus-independent
  `transfer_fullduplex()` reads, including the two response bytes discarded by
  that backend. It emits header-mode 13-byte combined gyro/accelerometer FIFO
  frames plus the separate temperature register. On KakuteF4 the real backend
  registered IMU2 at bus 0/address `0x68`, followed the baseline
  `(127, -254, -764)` mg / `(120, -230, 339)` mrad/s truth, then the stepped
  `(-305, 432, -662)` mg / `(-310, 170, -89)` mrad/s truth, and reported
  20.00 C then 30.00 C. Passing artifacts are under
  `build/renode-test/driver-probe-bmi270-i2c-stage5/`. Claude's review found
  no actionable issues and independently traced the HAL I2C full-duplex
  fallback, config handshake, FIFO frame, scales, temperature and instance
  assertions.
- Added a banked, physics-driven Invensensev2 ICM20948 I2C model. It preserves
  four register banks so the production bank-select callback and checked
  register machinery run unchanged, and supplies its big-endian 14-byte FIFO
  records with the backend's swapped/negated sensor-axis word ordering. On
  KakuteF4 the real backend registered IMU2 at bus 0/address `0x68`, followed
  `(127, -254, -764)` mg / `(120, -229, 339)` mrad/s and then
  `(-306, 433, -662)` mg / `(-316, 173, -92)` mrad/s, while temperature moved
  from 19.97 C to 30.07 C. The large temperature step also exercises the
  backend's FIFO-corruption guard and direct temperature-register reread.
  Passing artifacts are under
  `build/renode-test/driver-probe-icm20948-i2c-stage5/`. Claude's review found
  no actionable issues and confirmed the bank isolation, checked-register
  behavior, FIFO transform, scales and direct temperature-reread coverage.
- Added compound I2C attachments and used them for the Solo OreoLED set, whose
  four controllers occupy addresses `0x68` through `0x6b` on logical bus 0.
  Each address-specific model validates the production XOR checksum (including
  the controller address), accepts the boot, macro, simple RGB and extended RGB
  command forms, and exposes decoded state independently. The real MatekH743
  ArduPlane backend initialized all four endpoints and applied MAVLink override
  colours `(31,15,7)` and `(8,16,24)` without a checksum error. Passing
  artifacts are under
  `build/renode-test/driver-probe-matekh743-oreoled-stage5/`.
- Claude found that a monitor error partway through a compound runtime change
  could leave earlier endpoints applied but untracked. Compound attach and
  detach now carry one compensating command per endpoint: completed commands
  are rolled back in reverse order, and a failed rollback keeps the logical
  attachment visible in an error state so its addresses cannot be reused.
- Audited the four direct-I2C device sources which remain outside the launcher
  catalog. `AP_ADC_ADS1115` is instantiated only as a Linux HAL analog backend;
  MCP40D1x and TIx3204 DACs are instantiated and updated only by AP_Periph; and
  the L3G4200D/ADXL345 backend is Linux-guarded example code with no current
  frontend probe call. These are now machine-readable boundaries rather than
  implied missing ChibiOS vehicle models. Inventory validation fails if a new
  direct-I2C device source is neither selectable nor explicitly bounded.
- Added the command-based ICP101xx barometer. The model supplies CRC-protected
  OTP constants from the vendor conversion example, then algebraically inverts
  the production backend's nonlinear calibration formula for each physics
  pressure/temperature sample. Its nine-byte measurement layout includes all
  three per-word CRC positions even though the current backend ignores sample
  CRCs. The real MatekH743 backend registered it as BARO3 at bus 1/address
  `0x63`, reported 1013.25 hPa/20 C and the 900.00 hPa/30 C step, became
  unhealthy while samples were suppressed, and recovered without reboot.
  Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-icp101xx-stage5/`.
- Completed the ChibiOS I2C hwdef families with the compound ICM20789 package.
  One logical attachment reserves its barometer at `0x63` and its Invensense
  IMU at `0x68`. The barometer shares the ICP101xx OTP and nonlinear pressure
  conversion, but implements the ICM pressure-first `0x5059` sample layout;
  the IMU supplies the production driver's identity, temperature scaling and
  physics FIFO records. The real KakuteF4 backends registered BARO2 and IMU2,
  followed 1013.25 hPa/20 C and 900.00 hPa/30 C together with both nonuniform
  motion vectors, became unhealthy when barometer samples were suppressed and
  recovered without reboot. The H7 I2C controller also exposed split reads of
  the nine-byte sample, so the command model now retains a response cursor for
  the full transaction. The ICP101xx production probe passed again after that
  shared-model correction. ICM20789 artifacts are under
  `build/renode-test/driver-probe-kakutef4-icm20789-stage5-fixed/`.
- Completed Stage 5. Every barometer, compass and IMU family currently named by
  a ChibiOS I2C hwdef is classified as modelled, and every in-scope direct-I2C
  production source is either represented in the launcher catalog or has a
  machine-checked application boundary.
- Began Stage 6 with a standard NMEA GPS. The timed UART model emits
  checksum-protected RMC and GGA sentences with physics-driven latitude,
  longitude, altitude, horizontal speed and course. The production MatekH743
  NMEA backend detected it at 230400 baud and reported both non-axis-aligned
  motion points: 4.98 m/s at 323.13 degrees and 9.99 m/s at 126.87 degrees.
  Suppressing output and XOR-corrupting every byte each made GPS unhealthy;
  restoring valid frames reprobed NMEA and recovered the full stepped state
  without reboot. The shared GPS probe now checks velocity and course as well
  as position, and the existing u-blox profile passed the strengthened test.
  Artifacts are under `build/renode-test/driver-probe-matekh743-nmea-stage6/`
  and `build/renode-test/driver-probe-matekh743-ublox-stage6-regression/`.
- Added SiRF binary GPS message 41 with the production 91-byte payload, big-
  endian packed fields, 15-bit additive checksum and framing. The first real
  backend run exposed three long-standing decoder errors: fix fields were not
  byte-swapped, a 16-bit speed field was passed to `be32toh`, and course was
  interpreted as signed then truncated before wrapping. The separate
  `AP_GPS:` fix now follows the protocol's unsigned 16-bit speed/course fields.
  The corrected MatekH743 backend reported both physics points exactly,
  including 5.00 m/s at 323.13 degrees and 10.00 m/s at 126.87 degrees, and
  recovered after both silence and byte corruption. Passing artifacts are
  under
  `build/renode-test/driver-probe-matekh743-sirf-stage6-reviewed-fix/`.
- Added Emlid Reach Binary GPS. Each physics epoch emits checksum-protected
  status, position and velocity messages with one shared GPS time-of-week, so
  the production backend's stale position/velocity pairing guard is exercised.
  The position packet uses the protocol's little-endian doubles and accuracy
  fields; the velocity packet carries all three NED components plus independent
  ground speed and course. The MatekH743 ERB backend reported 5.00 m/s at
  323.13 degrees and 10.00 m/s at 126.86 degrees at the two position/altitude
  points, then recovered after both output suppression and byte corruption.
  Artifacts are under
  `build/renode-test/driver-probe-matekh743-erb-stage6/`.
- Added NovAtel NOVA GPS by mirroring ArduPilot's SITL golden implementation.
  The model emits 28-byte little-endian headers and CRC32-protected PSRDOP,
  BESTVEL and BESTPOS messages at the receiver's 19200 baud, with velocity and
  position sharing one GPS epoch. The specified MatekH743 NOVA backend reported
  both position/altitude and speed/course truth points, became unhealthy under
  silence and corrupted CRCs, and recovered in place both times. Artifacts are
  under `build/renode-test/driver-probe-matekh743-nova-stage6/`.
- Added Swift Navigation SBP and SBP2 GPS variants through a shared timed UART
  model. Both use the protocol's little-endian frame header and CRC-16/CCITT,
  while each concrete attachment emits its own message identifiers, heartbeat
  major version and validity flags. The production MatekH743 backends
  autodetected SBP and SBP2 independently at 115200 baud, reported both
  non-axis-aligned position, altitude, speed and course points, and recovered
  in place after silence and CRC corruption. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-sbp-stage6/` and
  `build/renode-test/driver-probe-matekh743-sbp2-stage6/`.
- Added Septentrio SBF GPS including the receiver command-mode handshake used by
  the hardware backend. The model answers the port-enable prompt and each
  configuration command, and starts SBF output only once the command sequence
  settles. The production probe disables SBAS so it exercises and explicitly
  checks the terminal SGA command. This exposed and fixes a production backend
  condition which discarded a successfully formatted SGA command. Scheduled
  UART responses and navigation frames are serialized on one wire.
  PVTGeodetic revision-2 blocks provide radians,
  vertical-up velocity and accuracy fields in the current backend layout, and
  DOP blocks share the same GPS epoch. The MatekH743 backend reported both
  physics truth points and recovered after silence and CRC corruption. Passing
  artifacts are under
  `build/renode-test/driver-probe-matekh743-sbf-stage6/`.
- Added Trimble GSOF GPS with bidirectional Data Collector framing. The model
  checksum-validates and acknowledges the backend's baud request and all five
  output-record requests, and does not emit navigation until position time,
  LLH, velocity, DOP and position-sigma records have all been requested. One
  big-endian GSOF report then carries the five physics-driven records with a
  shared epoch. The specified MatekH743 backend reported both position,
  altitude, speed and course truth points and recovered in place after silence
  and checksum corruption. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-gsof-stage6/`.
- Completed physical UART GNSS coverage and moved Stage 6 into serial ranging.
  Added MaxSonar Serial LV with its 9600-baud `Rnnn` inch reports and the
  checksum-protected NMEA DPT rangefinder variant. A catalog-driven MatekH743
  profile runs both production backends concurrently, maps them to independent
  physics channels, and verifies 2.00/2.10 m baseline and 3.00/3.10 m stepped
  values. Both backends stopped reporting under silence and invalid byte
  streams and recovered in place when valid output resumed. The generalized
  serial rangefinder probe now derives MAVLink sensor IDs from attachment
  instances and applies the same corruption coverage to the existing Benewake
  and LightWare models. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-ascii-rangefinders-stage6-fixed/`
  and
  `build/renode-test/driver-probe-matekh743-rangefinders-stage6-corruption/`.
- Added the one-way fixed-frame GY-US42v2, Lanbao and TeraRanger Serial
  rangefinders. Their packet layouts and checksum implementations mirror the
  existing SITL golden generators: additive checksum at 9600 baud, little-
  endian Modbus CRC, and polynomial-0x07 CRC-8 respectively. The serial models
  now share one abstract physics adapter while retaining per-device baud and
  framing. All three production backends ran concurrently on a MatekH743,
  reported 2.00/2.10/2.20 m and then 3.00/3.10/3.20 m, and independently
  stopped and recovered under both silence and checksum corruption. The prior
  MaxSonar/NMEA profile also passed after the shared-base refactor. Artifacts
  are under
  `build/renode-test/driver-probe-matekh743-binary-rangefinders-stage6/`
  and
  `build/renode-test/driver-probe-matekh743-ascii-rangefinders-stage6-base-regression/`.
- Expanded the shared Benewake serial model into explicit TF02, TFMini and TF03
  launcher choices. These production backends deliberately share the same
  `0x59 0x59` nine-byte parser but enforce different signal and maximum-range
  policies, so one wire model remains the correct implementation while each
  catalog entry applies its own `RNGFNDn_TYPE`. All three backends ran
  concurrently, tracked 2.00/2.10/2.20 m and 3.00/3.10/3.20 m physics inputs,
  and independently recovered after silence and checksum corruption. Artifacts
  are under
  `build/renode-test/driver-probe-matekh743-benewake-rangefinders-stage6/`.
- Added the remaining one-way serial radar/rangefinder frames: Aerotenna USD1
  v1, NoopLoop TOFSense-P/F, JRE, Ainstein LR-D1 v19 and Benewake RDS02UF.
  Their encoders mirror the checked-in SITL golden models, including USD1
  version discovery, NoopLoop's packed 24-bit distance, reflected CCITT CRC,
  LR-D1 validity/SNR fields and the RDS02UF device-specific CRC table. Five
  production backends ran concurrently and reported 2.00 through 2.40 m,
  recovered independently after both silence and invalid checksums, then
  reported 3.00 through 3.40 m. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-radar-rangefinders-stage6-fixed/`.
- Added the command-driven DTS6012M and Blue Robotics Ping1D rangefinders. The
  DTS model validates the backend's Modbus-CRC start-stream request before
  sending its 23-byte measurement frame at 921600 baud. Ping1D validates both
  the ping-interval and continuous-DISTANCE_SIMPLE commands before emitting
  checksum-protected replies. Both production backends completed their
  handshakes, reported 2.00/2.10 m and then 3.00/3.10 m, and recovered after
  independent silence and checksum corruption. Ping corruption initially
  exposed a production parser bug which reused the previous valid message for
  every later byte; the separately reviewed `AP_RangeFinder:` fix makes
  completion edge-triggered and adds a unit regression test. Passing Renode
  artifacts are under
  `build/renode-test/driver-probe-matekh743-command-rangefinders-stage6-fixed/`.
- Added the configured Attollo Wasp and LightWare GRF serial rangefinders. The
  Wasp model acknowledges the backend's baud, byte-order, format, frequency,
  ranging and filter commands and withholds ASCII measurements until the full
  default command set completes. The GRF model resynchronises after the
  serial-mode wakeup bytes, validates XMODEM CRCs, responds with a GRF product
  identity, echoes the update-rate and distance-selection writes, and starts
  its decimetre-resolution distance stream only after the requested stream ID
  is enabled. Both production backends reported 2.00/2.10 m and 3.00/3.10 m
  and recovered independently after silence and invalid output. Passing
  artifacts are under
  `build/renode-test/driver-probe-matekh743-configured-rangefinders-stage6/`.
- Added LeddarOne and LeddarVu8 as genuinely polled Modbus RTU devices. Each
  model validates the production backend's address, function, register range,
  count and little-endian Modbus CRC before replying. LeddarOne returns the
  requested ten-register detection block with a millimetre distance, while
  Vu8 returns eight big-endian segment distances in centimetres and lets the
  production backend select the shortest nonzero return. Both backends issued
  valid requests, reported 2.00/2.10 m and 3.00/3.10 m, and continued polling
  until they recovered independently from silence and bad CRCs. Passing
  artifacts are under
  `build/renode-test/driver-probe-matekh743-polled-rangefinders-stage6/`.
- Began physical serial proximity coverage with TeraRanger Tower and LD06. The
  Tower model emits its eight big-endian distances and polynomial-0x07 CRC at
  921600 baud; LD06 emits a rotating set of 12 confidence-qualified samples in
  the production 47-byte polynomial-0x4D frame at 230400 baud. The first live
  run also established that proximity probes must use Copter or Rover: Plane
  links common proximity telemetry but does not initialise the frontend. Both
  production Copter backends populated all eight MAVLink proximity directions
  at 2.00 m and then 3.00 m, stopped under silence and bad CRCs, and recovered
  without reboot. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-teraranger-proximity-stage6e/` and
  `build/renode-test/driver-probe-matekh743-ld06-proximity-stage6/`.
- Added TeraRanger Tower Evo with its complete production initialization
  exchange. The model validates the binary-printout, tower-sequence, 100 Hz
  refresh and stream-start commands in order, returns the four-byte
  acknowledgement expected by the backend after each command, and withholds
  its 20-byte eight-sector stream until configuration completes. The real
  MatekH743 Copter backend completed the four delayed requests, populated all
  directions at 2.00 m and 3.00 m, and recovered in place after both silence
  and CRC corruption. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-teraranger-evo-proximity-stage6/`.
- Added the command-started Cygbot D1. The model verifies the backend's exact
  eight-byte 2-D stream request and sends 161 big-endian samples spanning the
  sensor's real -60 to +60 degree field at 0.75-degree spacing. The probe now
  declares the three forward MAVLink faces expected from this partial scan
  instead of incorrectly requiring rear coverage. Its startup scheduler puts
  one complete scan into the H7's 576-byte UART buffer before the backend's
  one-second initialization holdoff expires, then starts continuous scans only
  after that first packet can be drained. The real MatekH743 Copter backend
  populated IDs 17, 10 and 11 at 2.00 m and 3.00 m and recovered after both
  silence and checksum corruption. This exposed a production checksum bug:
  the parser XORed an unwritten `payload[0]` byte, making valid hardware
  packets depend on the previous heap contents. The separate `AP_Proximity:`
  fix initializes the parser state and checksums only the received one-based
  payload. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-cygbot-proximity-stage6e/`.
- Added SLAMTEC RPLidar A2 with the complete standard-scan command path. The
  model responds to device-info, health, reset, stop and scan requests with
  production descriptors, identifies as the 16 m A2, and emits the packed
  five-byte records with complementary revolution bits, Q6 angles and Q2
  distances. A 200-sample/s emulator cadence and five-degree step cover all
  eight sectors without imposing the full sensor's host CPU cost. The real
  MatekH743 Copter backend discovered the A2, requested scan mode, populated
  all directions at 2.00 m and 3.00 m, and resynchronised after silence and
  deliberately invalid scan records. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-rplidar-a2-proximity-stage6/`.
- Added the LightWare SF40/C and SF45/B proximity scanners on a shared
  CRC16-XMODEM command parser, and moved the existing GRF rangefinder onto the
  same framing base. SF40/C implements the motor-state and token reads, output-
  rate and stream writes, and a 72-point full-circle distance packet. SF45/B
  implements its update-rate, output-field and stream writes and sweeps its
  individual distance records between -170 and +170 degrees. Both production
  Copter backends populated every proximity direction at 2.00 m and 3.00 m,
  stopped under silence and corrupt frames, and recovered without reboot. The
  existing GRF production backend also passed its complete regression probe
  after the parser refactor. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-lightware-sf40c-proximity-stage6b/`,
  `build/renode-test/driver-probe-matekh743-lightware-sf45b-proximity-stage6/`
  and
  `build/renode-test/driver-probe-matekh743-configured-rangefinders-lightware-base-regression/`.
- Completed Stage 6: every physical serial GNSS, rangefinder and proximity
  backend now has a dynamic catalog model and a production-driver probe.
- Began Stage 7 with the NMEA VHW/MTW water-speed sensor and MWV wind sensor.
  The shared NMEA encoder emits checksummed 4800-baud sentences. The water-
  speed model follows physics airspeed and temperature; its real MatekH743
  Rover backend reported 0/20 m/s and 20/30 C, became unhealthy under silence
  and checksum corruption, and recovered in place. The wind model exposes
  runtime speed and direction controls; its production backend tracked four
  value steps and retained its previous reading while output was suppressed or
  corrupted before accepting valid data again. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-nmea-airspeed-stage7c/` and
  `build/renode-test/driver-probe-matekh743-nmea-wind-stage7/`.
- Added the two physical UART optical-flow sensors: Cheerson CX-OF and UPixel
  UPFLOW. A shared physics adapter converts NED velocity, attitude, height and
  angular rate into each sensor's integrated-motion representation. CX-OF
  exercises its coarse 1.76-milliradian count and remapped 64--78 quality
  range; UPFLOW exercises its microsecond integration interval, sensor-axis
  negation and XOR-protected payload. Both real MatekH743 ArduPlane backends
  reported the baseline `(0.6,0.8)` rad/s and stepped `(-0.65,-0.75)` rad/s
  motion within protocol quantisation, became unhealthy under silence and
  corrupt byte streams, and recovered without reboot. UPFLOW also rejected
  frames with an independently corrupted checksum while retaining valid
  framing. The existing PX4Flow production probe passed after the shared
  assertion refactor. Artifacts are
  under `build/renode-test/driver-probe-matekh743-{cxof,upflow}-stage7/` and
  `build/renode-test/driver-probe-matekh743-px4flow-stage7-regression/`.
- Added a physics-driven NMEA AIVDM AIS receiver. It emits Class A position
  reports plus fragmented static/voyage data for one vessel, including MMSI,
  callsign, name, type and dimensions. The real MatekH743 Rover backend tracked
  baseline and stepped position, speed, course and heading values, rejected
  checksum-corrupt, suppressed and byte-corrupt input, and recovered after
  each fault without reboot. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-nmea-ais-stage8/`.
- Added the Pozyx UWB beacon as the first production serial-beacon backend.
  Its model emits the four anchor configurations, geometric ranges and a
  physics-driven vehicle position. A feature-forced MatekH743 Copter build
  recorded both position steps and all four ranges in `BCN` DataFlash records,
  lost health under checksum corruption, silence and arbitrary byte corruption,
  and recovered after each fault. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-pozyx-beacon-stage7b/`.
- Added the Marvelmind UWB beacon with high-resolution anchor positions,
  physics-driven vehicle positions and geometric anchor ranges. Its model
  preserves the production protocol's ENU wire coordinates and Modbus CRC,
  while the real MatekH743 Copter backend converts the data back to NED. The
  DataFlash oracle verified both position steps and all four ranges, plus
  health loss and recovery through checksum corruption, silence and arbitrary
  byte corruption. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-marvelmind-beacon-stage7/`.
- Added the bidirectional Nooploop UWB beacon. Its model streams signed 24-bit
  ENU node frames, recognises the production backend's 128-byte settings
  request and responds with four anchor positions before the backend accepts
  navigation data. The real MatekH743 Copter probe verified that handshake,
  both NED position/range sets, and three health-loss/recovery cycles through
  checksum corruption, silence and arbitrary byte corruption. Passing
  artifacts are under
  `build/renode-test/driver-probe-matekh743-nooploop-beacon-stage7d/`.
- Added an analyser for the output-only NMEA navigation protocol. A real
  MatekH743 ArduPlane build consumed physics-driven NMEA GPS on one UART and
  emitted checksummed GPGGA, GPRMC and PASHR sentences on another. The probe
  verified fix state, latitude, longitude, altitude, speed and the production
  output's course convention at baseline and stepped motion points, with no
  invalid checksums. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-nmea-output-stage7b/`.
- Added an analyser for output-only LightTelemetry (LTM). It validates the
  fixed GPS, attitude and sensor frame lengths and payload XOR checksums, and
  decodes their little-endian fields. A feature-forced MatekH743 ArduPlane
  build emitted all three frame types; the probe verified the GPS fix and
  satellite packing, baseline position, speed and relative altitude, then the
  stepped position and speed. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-ltm-output-stage7b/`.
- Added an analyser for the output-only Devo telemetry packet. A feature-forced
  MatekH743 ArduPlane build produced valid 20-byte additive-checksum frames;
  the probe verified DMS longitude conversion, baseline relative altitude and
  both ground-speed points. It also captured a production limitation:
  `gpsDdToDmsFormat()` returns an unsigned value, so the ChibiOS conversion
  clamps southern latitudes (and equivalently western longitudes) to zero.
  The oracle records that behavior without masking it in the analyser. Passing
  artifacts are under
  `build/renode-test/driver-probe-matekh743-devo-output-stage7b/`.
- Added an analyser for output-only FrSky D telemetry. It decodes the
  start-delimited, byte-stuffed little-endian sensor records emitted by the
  production backend. A feature-forced MatekH743 ArduPlane probe verified GPS
  status, southern/eastern position, speed and absolute altitude at baseline
  and stepped states. The 605 m baseline deliberately places `0x5D` in the
  altitude payload, proving the firmware's escaping and the analyser's
  unstuffing on the real UART stream. Passing artifacts are under
  `build/renode-test/driver-probe-matekh743-frsky-d-output-stage7b/`.

The current fast probe can be repeated without rebuilding firmware:

```sh
Tools/renode/tests/test_driver_probe.py \
    --skip-build \
    --renode build/renode/renode
```

Omit `--skip-build` to configure and build the MatekH743 firmware with the
checked-in test defaults first.

Next work:

1. Continue Stage 7 through the remaining bidirectional and output-only serial
   protocol families, grouping closely related protocols into reviewable
   tranches.
2. Extend the manifest with explicit protocol variants and per-frontend
   multi-instance parameter naming where a family shares one model.
3. Make the DroneCAN airspeed sidecar physics-driven and fault-testable as part
   of the separate CAN coverage track.

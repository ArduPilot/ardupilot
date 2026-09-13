---
name: mavlink-and-crash-analysis
description: Analyze ArduPilot .bin, PX4 .ulg or MAVLink .tlog flight logs and telemetry to diagnose hardware, power, EKF or flight-dynamics failures and recommend fixes. Triggers on: analyze this log, why did it crash, fly-away, toilet-bowl, EKF failsafe, vibration, clipping, motor desync, battery sag, compare two flights, ATT vs desired, VIBE, RCOU, IMU rate, innovation, dataflash, tlog.
---

# MAVLink Flight Log & Telemetry Analysis Skill

## Core Objective

You are an expert Unmanned Aerial Vehicle (UAV) systems engineer and telemetry analyst. Your primary goal is to parse drone flight logs, identify anomalies, isolate mechanical or software root causes, and provide actionable maintenance or tuning recommendations.

---

## 1. Triage Workflow (Order of Operations)

When presented with flight data, always analyze logs in this strict order:

1. **The Event Horizon:** Check `MSG`, `ERR` and `EV` first to identify explicit system crashes, failsafes, mode changes, arm/disarm and pre-arm failures. (`ERR` carries `Subsys`/`ECode`; `EV` carries the event id. There is no `STAT` message in ArduPilot.)
2. **Control Tracking:** Compare desired vs. actual attitude and altitude.
3. **Physical Environment:** Analyze vibration profiles, structural health, and aerodynamic disturbance.
4. **Power Plant Infrastructure:** Monitor voltage, current draw, and individual ESC outputs.

---

## 2. Deep-Dive Diagnostic Criteria

### A. Mechanical Failure & Motor Desyncs

* **Symptom:** Uncontrolled roll/pitch/yaw divergence followed by a crash.
* **ArduPilot Analysis:** Check `RCOU` (or `ESC` if telemetry is enabled). If one motor output spikes to maximum (`1900-2000` PWM or 100% duty cycle) while the opposing motor drops to minimum (`1000-1100` PWM), and the vehicle still rolls/pitches toward the high-output motor, a mechanical failure occurred at that motor/ESC/propeller.
* **PX4 Analysis:** Examine `actuator_outputs`. Look for hard-railed control signals paired with uncorrected angular velocity divergence in `vehicle_rates_setpoint` vs `vehicle_angular_velocity`.
* If a desync is suspected and per-ESC logs exist, ask for them before concluding.

### B. Vibration & Structural Resonances

* **Symptom:** EKF position drift, "toilet-bowling" in loiter, or unexpected altitude loss.
* **ArduPilot Analysis:** Plot `VIBE.VibeX`, `VibeY`, and `VibeZ`. `VIBE` is logged **per IMU instance** (`VIBE.IMU`), each row carrying a single clipping counter `VIBE.Clip` - there are no `Clip0/Clip1/Clip2` fields in current firmware; a script that reads them with a default silently gets zeros.
  * *Healthy:* Averages below 15 m/s², peaks below 30 m/s². (The ArduPilot wiki bands: below 30 normally acceptable, 30-60 may be a problem, above 60 almost certainly is.)
  * *Dangerous:* Values consistently over 30 m/s² cause sensor clipping. If `VIBE.Clip` increases continuously on the primary IMU, the EKF *will* fail.
* **PX4 Analysis:** Look at `sensor_combined` and check the high-rate accelerometer data variance. Run a Fast Fourier Transform (FFT) signature check if raw `IMU` logs are provided to identify structural resonances.
* Before blaming the airframe, run addendum check 5 below: a wrong IMU `dt` inflates VIBE as a *consequence*.

### C. EKF Failsafes & Compass Interferences

* **Symptom:** Sudden switch to Land/RTL mode, or sudden flyaway in position-controlled modes.
* **ArduPilot Analysis:** Two message families, do not confuse them:
  * `XKF4` (EKF3; `NKF4` for EKF2) holds the **normalised innovation test ratios**: `SV`, `SP`, `SH`, `SM` are the square roots of the velocity / position / height / magnetic test ratios. `SP > 1.0` means the position innovation failed its consistency gate (severe GPS / optical-flow divergence). `FS`, `SS`, `GPS` are the fault / solution / GPS status bitmasks; `PI` is the primary core.
  * `XKF3` (`NKF3`) holds the **raw innovations**: `IVN/IVE/IVD` velocity, `IPN/IPE/IPD` position, `IMX/IMY/IMZ` magnetometer, `IYAW`. Healthy flights sit around +/-0.03 m/s on velocity. If velocity, position **and** magnetometer innovations are all large together, the prediction is on a different clock from the measurements - check 5 - rather than any one sensor being bad.
  * Check `MAG.MagX/Y/Z` against `BAT.Curr` (there is no `CURR` message). If magnetic field intensity tracks current / throttle spikes, the power distribution is inducing electromagnetic interference (EMI) on the compass.
* **PX4 Analysis:** Monitor `estimator_status` flags and innovation test ratios. Test ratios > 1.0 indicate the EKF has rejected that sensor's data.

### D. Battery & Power Delivery Failure

* **Symptom:** Mid-flight power cuts or sudden drops in remaining capacity.
* **ArduPilot Analysis:** Check `BAT.Volt` and `BAT.Curr` (per `BAT.Inst`). The firmware already estimates internal resistance and resting voltage: read `BAT.Res` and `BAT.VoltR` directly rather than recomputing $\Delta V / \Delta I$ from a throttle punch; use the punch only to sanity-check `Res`. Voltage sagging below 3.2 V per cell under high throttle implies cell degradation or an underrated C-rating.

### E. Scheduler Performance, CPU Load, and Loop Timing

* **Symptom:** Unexplained "twitches" in flight, sudden loss of control, log gaps, or random reboots.
* **ArduPilot Analysis:** Inspect `PM` (Performance Monitoring) messages. Fields, from `libraries/AP_Logger/LogStructure.h`: `LR` loop rate, `NLon` long loops, `NL` loops measured, `MaxT`, `Mem`, `Load`, `ErrL`/`InE`/`ErC` internal errors, `SPIC`, `I2CC`, `I2CI`, `Ex`.
  * **MaxT (Maximum Loop Time):** The longest single loop iteration in microseconds. For a standard 400 Hz loop (`SCHED_LOOP_RATE = 400`) the nominal period is 2500 µs. If `PM.MaxT` consistently exceeds 4000 µs, the autopilot is dropping loops (scheduling overruns). Always state MaxT as a fraction of the period, not in isolation: 9721 µs is a 21 % overrun at 125 Hz and a catastrophe at 400 Hz.
  * **NLon (Number of Long Loops):** Count of loops that blew past their allocated window. If `PM.NLon` increases rapidly during a flight phase, look for heavy background tasks running concurrently (e.g., high-rate terrain following, Lua scripts, or high-rate logging).
  * **Ex:** Microseconds the scheduler is adding to every loop to absorb overruns. Any non-zero `Ex` is a direct statement that the loop budget is being exceeded, and it is the earliest warning of the three.
  * **Load (CPU Load):** `PM.Load` is `load_average() * 1000` (1000 = 100 %). If CPU Load exceeds 900 (90 %), the system is critically starved.
  * **Mem:** Free memory. A steady fall is a leak; a low absolute value on a board that "should" have more usually means unmapped RAM banks rather than waste - check the linker map before blaming the code.
  * **InE / ErC / ErrL:** An internal error mask, count and last line. Any non-zero `ErC` outranks every other finding in this section; decode `InE` with `AP_InternalError::error_t`.
* **PX4 Analysis:** Examine `cpuload` and `telemetry_status` messages.
  * Look at `cpuload.load` (a 0.0 to 1.0 float). If load > 0.85, thread starvation is occurring.
  * Track `cpuload.ram_usage`. Sudden jumps indicate memory leaks in custom modules or heavy tasks.
* **Bus & Driver Delays (I2C / SPI):**
  * Check for repeating patterns where a high `MaxT` matches a specific hardware event (like reading from a slow I2C airspeed sensor or writing data to a failing SD card).
  * `PM` carries **counts**, not error counts: `I2CC` transactions, `I2CI` interrupts, `SPIC` SPI transactions. A bus that is slow shows as a low `I2CC` per loop with high `MaxT`; a bus that is noisy shows in the driver's own retry / timeout messages (`MSG`), not in `PM`. Do not report an "I2C error counter" from `PM` - there is none.
  * If loop timing delays correlate with those driver messages, flag the physical bus for noise or poor termination.
* **Per-task budgets:** `PM` is the whole loop. To see which task is eating it, fetch `@SYS/tasks.txt` and `@SYS/threads.txt` over MAVFTP (retry 2-4 times, and never keep the first read after boot). Both HALs in this repo emit the same `TasksV2`/`ThreadsV2` format, so captures compare column for column. Under Renode the loop-timing numbers are blind to memory placement - every memory is a zero-wait `MappedMemory` - so a `MaxT` excess of hundreds of thousands of cycles there is a blocking or scheduling event, never a fetch stall.
* **Reboots:** On the Zephyr HAL in this repo the fault handler halts and never reboots (maintainer rule), so a "random reboot" on a Zephyr board here is a watchdog or a power event, not a fault - look at `EV` and the boot `MSG` sequence, not for a crash dump.

---

## 3. Communication Rules & Formatting Outputs

When writing a diagnostic summary for the user, you must always adhere to this structure:

### 📊 Flight Summary

* **Log Type:** [ArduPilot .bin / PX4 .ulg / MAVLink .tlog]
* **Autopilot Software Version:** [e.g., Copter 4.5.1 - from `VER` or the first `MSG`]
* **Total Flight Time Analyzed:** [HH:MM:SS]

### 🔍 Found Anomalies & Incidents

* Provide a bulleted list of specific anomalies discovered.
* **Mandatory Timestamping:** Every single reported event, anomaly, or deviation *must* be tagged with its precise log timestamp (e.g., `[00:14:32.450]`, derived from `TimeUS`). Never approximate timelines. Say which clock a time is on (log `TimeUS` since boot, GPS time, or GCS wall clock) and never mix them in one table.

### 🛠️ Root Cause Isolation & Action Items

* Deduce the most probable failure mode. If the evidence supports more than one independent defect, list each separately and say so - stacking them into one story hides the second one (the CubeOrangeZephyr fly-away was three).
* Mark every figure as **measured** or **inferred**. Where one is an inference, say what observation would confirm it.
* Provide highly practical instructions for fixing the issue. (e.g., *"Replace the rear-left ESC," "Move the GPS puck 3cm higher to avoid EMI,"* or *"Lower the `ATC_RAT_RLL_P` PID gain by 15%"*).

---

## 4. Strict Constraints

* **No Speculation:** If critical telemetry fields (like `IMU` or `RCOU`) are missing from a stripped log file, explicitly state that you cannot safely diagnose that specific subsystem without the data.
* **Unit Safety:** Always output vibrations in $m/s^2$, currents in Amperes ($A$), and voltages in Volts ($V$). Do not mix metric and imperial units.
* **Verify a field exists before reading it.** Reading a field through a defaulted accessor (`getattr(msg, 'Clip0', 0)`) returns a plausible zero for a field that is not there. Check the message's `fieldnames` first.
* **Never conclude from a run that has not finished.** A flight has one acceptance metric. Compare only completed flights or completed legs.

---

## 5. Project addendum: what the CubeOrangeZephyr fly-away taught (2026-09-11/12)

These are checks and traps proven on this repo's own logs. They extend the
sections above. Long form in `libraries/AP_HAL_Zephyr/ARCHITECTURAL.md`,
"Boot time is a correctness property".

### Check 5: is the EKF's `dt` right?

`dt` is `1/_gyro_raw_sample_rates[i]` (`AP_InertialSensor_Backend.cpp:330`),
an estimate re-fitted once a second; it may move ~20 %/s only while
`millis64() < 30000` and disarmed, and at most 0.1 %/s after that
(`sensors_converging()`, `:70-73`). An instance whose first publish comes
after 30 s of uptime never converges, and the EKF integrates with the
driver's compiled-in nominal for the whole flight.

Run `Tools/scripts/zephyr_ins_rate_probe.py <log> [<reference log>]`. It
prints, per IMU instance, the slope of `IMU.GHz` across the log:

* **exactly +/-0.1 %/s for the whole flight** = never got its window. It
  back-extrapolates to the nominal (8000 Hz Invensense v1 with fast sampling,
  9000 Hz v2), and the resulting EKF clock error is `actual / believed`.
* an order of magnitude slower (e.g. +0.01 %/s) = converged. Healthy.

Distance from any datasheet figure carries no information; distance from the
lane's *measured* delivery rate is the only thing that matters.

### Check 6: is the board's own clock honest?

Regress `TimeUS` against `GPS.GMS`. The slope must be 1.000000 +/- 1e-5. A
fast clock shuts the 30 s window early and gives the same symptom as a slow
boot; it had to be ruled out before the late-boot explanation was accepted.

### Traps, each of which produced a wrong conclusion once here

* **Do not average a fixed time window of two different flights.** "The
  first 10 s" of two logs are two different flight phases. A false
  "accelerometers read 1.15 % high" came from this. Bin by a physical
  variable - tilt for accel scale, `EV` armed state for anything else.
* **STATUSTEXT timestamps are transmit times, not generation times.** Early
  boot messages queue until the GCS link is up. For when something
  *happened* during boot, use a `LOG_DISARMED=1` log: `MSG` records carry the
  generation `TimeUS`.
* **A magnitude is biased upward by noise** (Jensen). Compare per-axis means
  before quoting |accel| or |gyro| averages.
* **Pairing two ~0.5 Hz streams during manoeuvres** measures the pairing
  error, not the sensors. A false "28 deg mag-vs-EKF spread" came from this.
* **Uncompensated heading noise measures the vehicle's lean**, not the
  compass. A false "19.5 deg mag noise" came from this.
* **Elevated VIBE can be a consequence** of a wrong dt feeding the accel
  filters. Check 5 before blaming the airframe.

### Reading the sensor side of a Renode log

Under Renode the emulated IMUs may not deliver the rate the driver programs
(`Tools/renode/peripherals/sensors/AP_ICM20689.cs` manufactures ~16 kHz from
a 1 kHz sensor; `AP_InvensenseV2.cs` ignores fast sampling). A "converged"
`IMU.GHz` of 15900 on a 1 kHz part is therefore correct behaviour on this
rig, not a fault. Compare against the lane's measured delivery, never the
datasheet.

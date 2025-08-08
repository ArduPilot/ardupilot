# Actuator Chirp (Lua Applet)

## Overview
**Actuator Chirp** excites a selected output channel (servo or ESC) with an exponential frequency sweep (“chirp”) and logs the response so you can estimate the actuator/plant frequency response. It emits live **named floats** (PWM, FREQ, RPM/POS) to the GCS and writes a dataflash log for offline analysis.

> **Important:** This applet does **not** automatically inhibit when armed. Be very careful if testing with propellers on.

---

## Installation
1. Copy `actuator_chirp.lua` to the ArduPilot **`APM/scripts`**
   directory on the vehicle microSD card
2. Restart scripting or reboot
3. Confirm you see **“ActuatorChirp loaded”** in the GCS messages.

---

## Parameters
Use the following parameters to configure the chirp. Set values via GCS (PARAM tab) or the CLI. Links below use the ArduPilot docs style.

- ACHRP_CHAN — **Chirp output channel**  
  1‑based servo/ESC output channel to drive with the chirp. Set `0` to disable/idle.  
  Range: `0–32`

- ACHRP_PWM_MIN — **Minimum PWM for chirp**  
  Lower PWM bound used to center and scale the chirp waveform.  
  Range: `800–2200` µs; Units: `pwm`

- ACHRP_PWM_MAX — **Maximum PWM for chirp**  
  Upper PWM bound used to center and scale the chirp waveform.  
  Range: `800–2200` µs; Units: `pwm`

- ACHRP_F_START — **Start frequency**  
  Chirp start frequency (Hz). Script adds a short dwell at this frequency before the sweep.  
  Range: `0.05–200` Hz

- ACHRP_F_STOP — **Stop frequency**  
  Chirp stop frequency (Hz). Must be ≥ start frequency.  
  Range: `0.05–500` Hz

- ACHRP_F_FADE_IN — **Fade‑in time**  
  Duration (s) to ramp amplitude from 0 to full to avoid step transients.  
  Range: `0–30` s

- ACHRP_F_FADE_OUT — **Fade‑out time**  
  Duration (s) to ramp amplitude back to 0 at the end.  
  Range: `0–30` s

- ACHRP_TIME — **Total chirp duration**  
  Total time (s), including initial dwell and fade windows.  
  Range: `1–600` s

- ACHRP_TYPE — **Type of actuator**  
  Selects feedback source and log type.  
  Values: `0: ESC`, `1: Servo`

---

## How it works
- The script generates an **exponential chirp** from `F_START` → `F_STOP` over `TIME`, with tapered **fade‑in/out** windows.
- It drives the chosen **output channel** which should be an ESC or servo
- Live telemetry is sent as NAMED_VALUE_FLOAT MAVLink telemetry: `PWM`, `FREQ`, plus `RPM` (for ESC) or `POS` (for servo).
- Dataflash logging:
  - **ESC mode** (`ACHRP_TYPE=0`): log type **`ECRP`**, fields `PWM, Freq, RPM` (format `fff`).  
  - **Servo mode** (`ACHRP_TYPE=1`): log type **`SCRP`**, fields `PWM, Freq, Pos` (format `fff`).

When the chirp completes, the script prints **“Chirp Finished”** and automatically **sets `ACHRP_CHAN` to 0** to stop output.

---

## Usage
1. **Bench‑prepare**
   - Remove props / disconnect linkages as appropriate.
   - For ESC tests, ensure telemetry is wired and enabled to give RPM feedback on the target channel.
   - For servo tests, ensure `servo_telem` provides measured_position for the target channel (if available) (DroneCAN servos)

2. **Configure**
   - Set `ACHRP_PWM_MIN` / `ACHRP_PWM_MAX` to safe bounds for the device under test.
   - Choose a reasonable sweep (e.g., `ACHRP_F_START=0.5`, `ACHRP_F_STOP=20`, `ACHRP_TIME=60`, `ACHRP_F_FADE_IN=5`, `ACHRP_F_FADE_OUT=1`).
   - Select actuator type: `ACHRP_TYPE=0` for ESC, `1` for servo.

3. **Run**
   - Set ACHRP_CHAN to the **1‑based** output channel to start the chirp.
   - Watch GCS named floats (`FREQ`, `PWM`, and `RPM`/`POS`).

4. **Finish**
   - On completion the script sets `ACHRP_CHAN=0` automatically. You can also stop early by manually setting `ACHRP_CHAN=0`.

Note that for many DroneCAN ESCs you need to be armed to run the motor. Either arm when testing or disable this requirement in the ESC settings.

---

## Data analysis tips
- Use the `ECRP`/`SCRP` log messages for offline analysis. The **`Freq`** field is the instantaneous excitation frequency in Hz.
- For a quick view, plot `RPM` (or `Pos`) vs. `Freq` to estimate magnitude response. For a rough phase estimate, compare response to the drive waveform timing.
- Keep amplitude modest (i.e., tight `PWM_MIN/MAX`) to stay within **linear** range of the plant.

---

## Safety notes
- Disable motor outputs or remove props where possible.  
- Verify `ACHRP_PWM_MIN/MAX` are safe for the actuator.  
- Power components adequately; long sweeps can heat motors/servos.  
- Ensure radio failsafe cannot take control of the tested channel during the sweep.


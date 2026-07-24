# Xsens MTi External AHRS Driver for ArduPilot

This driver lets ArduPilot use an Xsens MTi IMU / AHRS / GNSS-INS sensor as its
primary attitude and navigation source. The Xsens sensor does its own sensor
fusion onboard and feeds attitude, position, and velocity straight into
ArduPilot over UART or SPI.

If you just want to get it working, follow the **Quick Start** below in order.
The sections after it explain the details, the wiring options, and what each
setting does.

---

## Supported Devices

| Device | Type | GNSS | Interface | Series |
|---|---|---|---|---|
| MTi-7 | AHRS / GNSS-INS | External | UART / SPI | MTi 1-series |
| MTi-8 | AHRS / RTK GNSS-INS | External RTK | UART / SPI | MTi 1-series |
| MTi-670(G) | GNSS/INS | Internal | UART (RS232) | MTi 600-series |
| MTi-680(G) | RTK GNSS/INS | Internal RTK | UART (RS232) | MTi 600-series |
| MTi-G-710 | GNSS/INS | Internal | UART | MTi 100-series |

All devices use the Xbus protocol. The driver configures the sensor outputs
automatically when it connects.

---

## Quick Start

This is the shortest path from nothing to a working setup. Do the five steps in
order.

**You will need:**

- A Pixhawk-compatible flight controller (tested on Holybro Pixhawk V6X and
  CUAV V6X)
- An Xsens MTi sensor from the table above
- A PC with Mission Planner installed
- A computer that can build ArduPilot (Ubuntu, or Windows with WSL2 — native
  Windows cannot build ArduPilot)

### Step 1 — Build the firmware

The standard ArduPilot download does **not** include the Xsens driver yet, so
you build it once from this branch. Run these commands on Ubuntu (or inside
WSL2 on Windows):

```bash
git clone https://github.com/jiminghe/ardupilot.git
cd ardupilot
git checkout AP_ExternalAHRS_Xsens
git submodule update --init --recursive

# Install the build tools (first time only; takes a while)
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile

# Build for Pixhawk V6X (use plane instead of copter if you fly fixed-wing)
./waf configure --board Pixhawk6X
./waf copter
```

When it finishes, your firmware file is here:

```
build/Pixhawk6X/bin/arducopter.apj
```

Copy that `.apj` file somewhere you can reach it from Mission Planner.

> **Building on Windows?** Run the commands above inside WSL2, then copy the
> `.apj` out to Windows, e.g.
> `cp build/Pixhawk6X/bin/arducopter.apj /mnt/c/Users/<you>/Desktop/`

> **Using SPI instead of UART?** See [SPI notes](#spi-notes) before building —
> one line in `hwdef.dat` may need to be commented out.

### Step 2 — Flash the firmware

1. **Disconnect** Mission Planner if it is connected (flashing uses a different
   mode — you must not be connected while flashing).
2. Go to **SETUP → Install Firmware**.
3. Click **Load custom firmware** and pick your `.apj` file.
4. Follow the prompts (unplug the board, click OK, plug it back in).

### Step 3 — Wire the sensor

> **Power first:** MTi-7 / MTi-8 modules need a **3.3 V** supply. A 5 V supply
> will light the LED but the data link will **not** work. Power is separate
> from the data interface — using UART or SPI does not change how you power the
> module.

**For UART** (simplest — start here):

| Flight controller | -> | Xsens sensor |
|---|---|---|
| TX | -> | RX |
| RX | -> | TX |
| GND | -> | GND |

On the MTi 1-series, set the interface-select pins to UART:
`PSEL0 = GND`, `PSEL1 = GND`.

(For SPI wiring, see [SPI notes](#spi-notes).)

### Step 4 — Set the parameters

1. Connect Mission Planner to the flight controller (top-right **Connect**).
2. Go to **CONFIG -> Full Parameter List**.
3. Set the parameters below, then click **Write Params**.

**UART setup (Pixhawk V6X, sensor on the GPS2 port = SERIAL4):**

| Parameter | Value |
|---|---|
| `EAHRS_TYPE` | `12` |
| `AHRS_EKF_TYPE` | `11` |
| `EAHRS_OPTIONS` | `0` |
| `SERIAL4_PROTOCOL` | `36` |
| `SERIAL4_BAUD` | `115` |

4. **Reboot** the flight controller (unplug/replug, or use Mission Planner's
   reboot option).

> Using a different port than GPS2? Change `SERIAL4_*` to match the port you
> wired to. See [Serial port mapping](#serial-port-mapping).

### Step 5 — Check that it works

After reboot, open the **Messages** tab in Mission Planner. You should see lines
like:

- `Xsens ExternalAHRS initialised`
- `IMU0: External: Xsens`
- `Baro count: 3`

Now tilt the sensor by hand and watch the artificial horizon (HUD). It should
follow the sensor's movement in real time. **That's it — you're done.**

---

## SPI Notes

SPI is an alternative to UART for the MTi 1-series. Use it if your design needs
the faster interface.

**Before building (Step 1):** open the `hwdef.dat` for your board and check the
DRDY line. If the DRDY pin is not wired or not available, comment it out so the
firmware uses polling mode:

```
# define HAL_GPIO_XSENS_DRDY 94
```

Once this line is commented out, the firmware ignores DRDY completely — it does
not matter whether the DRDY wire is connected.

**Wiring (Step 3):** set the interface-select pins to SPI:
`PSEL0 = GND`, `PSEL1 = float`.

| Signal | Xsens sensor |
|---|---|
| NCS (chip select) | SPI CS |
| SCLK | SPI clock |
| MOSI | data: controller -> sensor |
| MISO | data: sensor -> controller |
| GND | GND |

Make sure the sensor, its power supply, and the flight controller all share a
common ground.

**Parameters (Step 4):** same as UART, except `EAHRS_OPTIONS`:

| Parameter | Value |
|---|---|
| `EAHRS_TYPE` | `12` |
| `AHRS_EKF_TYPE` | `11` |
| `EAHRS_OPTIONS` | `16` |

In SPI mode you do not need the `SERIALn_PROTOCOL` / `SERIALn_BAUD` settings.

---

## Reference

### EAHRS_OPTIONS bitmask

| Bit | Value | Meaning |
|---|---|---|
| 3 | 8 | Sensor mounted facing downward |
| 4 | 16 | Use SPI instead of UART |

Add the values for the options you want:

| Setup | Value |
|---|---|
| UART, sensor upward | `0` |
| UART, sensor downward | `8` |
| SPI, sensor upward | `16` |
| SPI, sensor downward | `24` |

### Serial port mapping

On Pixhawk V6X the labelled ports map to ArduPilot serial numbers as follows:

| Physical port | ArduPilot |
|---|---|
| TELEM1 | SERIAL1 |
| TELEM2 | SERIAL2 |
| GPS2 | SERIAL4 |

Set `SERIALn_PROTOCOL = 36` and `SERIALn_BAUD = 115` on whichever port you wire
the sensor to. Other boards differ — check your board's documentation.

### What the driver provides

The Xsens sensor feeds ArduPilot acceleration, rate of turn, quaternion
(attitude), magnetic field, barometric pressure, and (on GNSS models) position
and velocity. Setting `AHRS_EKF_TYPE = 11` tells ArduPilot to use the Xsens
attitude solution directly instead of running its own EKF.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| LED on, but no data | Module powered from 5 V | Use a 3.3 V supply |
| No data (UART) | Wrong PSEL, or TX/RX swapped | Set `PSEL0=GND, PSEL1=GND`; swap TX/RX |
| No data (SPI) | Wrong PSEL or wiring | Set `PSEL0=GND, PSEL1=float`; recheck CS/SCLK/MOSI/MISO |
| HUD attitude frozen | `AHRS_EKF_TYPE` not set | Set `AHRS_EKF_TYPE = 11` and reboot |
| Nothing in Messages tab | Firmware not flashed, or wrong port | Re-flash; confirm `SERIALn_PROTOCOL = 36` on the right port |

---

## Notes

- This driver is not in the standard ArduPilot release yet. You must build from
  a branch that includes it (such as `AP_ExternalAHRS_Xsens`) until it is merged
  upstream.
- The values above (`EAHRS_TYPE = 12`, SPI option `16`) apply to firmware synced
  with recent ArduPilot upstream. Older builds of this branch used
  `EAHRS_TYPE = 11` and `EAHRS_OPTIONS = 4` for SPI. If you are upgrading, update
  your parameters to match.

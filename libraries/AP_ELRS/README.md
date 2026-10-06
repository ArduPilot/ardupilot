# AP_ELRS

Experimental ExpressLRS 2.4 GHz reception for AP_Periph using a single SX1280,
with RC channels published over DroneCAN.

## Supported features

- Automatic RF-rate detection and frequency hopping.
- LoRa: 50, 150, 250 and 500 Hz; 100 and 333 Hz Full Resolution.
- FLRC: F500, F1000, D250 and D500.
- Hybrid/Wide: 12 channels. Full Resolution: up to 16 channels.
- Over-air binding, model matching and link-statistic telemetry (RSSI, SNR, LQ).
- Telemetry Off and ratios from 1:128 to 1:2.

## Setup

Use an AP_Periph board with SX1280 hardware support and the `Periph_ELRS`
build option. It is not needed for a normal ELRS receiver connected over UART
using CRSF. When enabled, ELRS owns RC input; there is no serial RC fallback.

| Parameter | Usage |
| --- | --- |
| `RC_ELRS_VER` | Compatibility version. Only `4` is supported; other values prevent reception and binding. Reboot after changing. |
| `RC_ELRS_MODEL` | TX model-match ID, or `255` to disable model matching. Reboot after changing. |
| `RC_ELRS_BIND` | Set to `1` and reboot to enter binding mode. |
| `RC_MSGRATE` | Maximum DroneCAN RC output rate in Hz; default `50`, range `0–127`. `0` disables output. |

To bind:

1. Set `RC_ELRS_BIND=1` and reboot, or hold the board's binding button during
   power-on if fitted. Release the button when the binding indication starts.
2. Trigger Bind on the transmitter.
3. The receiver saves the learned identity and reboots automatically.

Binding uses DroneCAN maintenance mode and suspends RC output. Reboot to cancel
binding without changing the saved identity. Reconnection does not require
rebinding. Binding credentials are hidden from normal parameter listings.

## Behavior and limitations

RF rate and channel update rate differ from DroneCAN output rate. `RC_MSGRATE`
limits output and cannot create new RC samples. Full16 output requires fresh
CH1–8 and CH9–16; receiving only one half does not refresh the complete set.

Stale channel sets are not published. After previously valid input has been
unavailable for four seconds, the receiver reports FAILSAFE with no channels.
Flight-controller failsafe behavior must be validated separately.

Initialization and terminal radio faults report DroneCAN health ERROR. Normal
binding, searching and RF link loss do not themselves indicate a hardware fault.

Flight-controller telemetry forwarding, MAVLink link mode, Gemini, dual-radio
diversity, sub-GHz radios and external PA/LNA switching are not supported.

ExpressLRS 4.1.0 was used for prototype bench testing. Not all RF modes or
in-vehicle failsafe behavior have been validated; F1000 was unavailable on the
test transmitter.

## Upstream reference

Based on [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) (GPLv3):

- Development baseline: `9ff3fa4cdb184c92eea95e9db5cfe231da3714da`.
- Release comparison: `4.1.0`, commit `a9d4a9cb5b5687c4c9d7e9e7fbdf44ad93651da6`.

These are fixed references, not a guarantee of compatibility with future
ExpressLRS releases. Upstream SX1280 support also includes Semtech Revised BSD
sources; original copyright and license notices must be retained.

Development was AI-assisted.

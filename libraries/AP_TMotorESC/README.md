# T-Motor Alpha ESC serial telemetry input

This library decodes the T-Motor manufactured Alpha [ESC](https://en.wikipedia.org/wiki/Electronic_speed_control)'s serial telemetry communication protocol.
It is a (unidirectional) [digital asynchronous serial communication protocol](https://en.wikipedia.org/wiki/Asynchronous_serial_communication) running at XXX bit/s Baudrate. It requires three wires (RX, TX and GND) connection.

## Features of this device driver

- use ArduPilot's coding guidelines and naming conventions
- Use the AP_ESC_Telem base class to:
  - copy ESC telemetry data into MAVLink telemetry
  - save ESC telemetry data in dataflash logs
  - use RPM telemetry for dynamic notch filter frequencies
  - sum the current telemetry info from all ESCs and use it as virtual battery current monitor sensor
  - average the voltage telemetry info and use it as virtual battery voltage monitor sensor
  - average the temperature telemetry info and use it as virtual battery temperature monitor sensor
- pre-arm checks:
  - Check that UART is available
  - check that the ESCs are periodically sending telemetry data

## ESC to ArduPilot protocol

ESC telemetry information is sent back to the autopilot:

- Electronic rotations per minute (eRPM/100) (must be divided by number of motor poles to translate to propeller RPM)
- Input voltage (V/10)
- Current draw (A/10)
- Power consumption (mAh)
- Temperature (°C/10)
- CRC errors (ArduPilot->ESC) counter

This information is used by ArduPilot to:

- log the status of each ESC to the SDCard or internal Flash, for post flight analysis
- send the status of each ESC to the ground station or companion computer for real-time monitoring
- Optionally dynamically change the center frequency of the notch filters used to reduce frame vibration noise in the gyros
- Optionally measure battery voltage and power consumption


## Function structure

There are two public top level functions `update()` and `pre_arm_check()`.
And these two call all other private internal functions.
A single (per ESC) state variable (`_escs[i]._state`) is used in both the RX and TX state machines.
Here is the call graph:

```
update()
  init()
    init_uart()
  read_data_from_uart()
    move_frame_source_in_receive_buffer()
      consume_bytes()
    consume_bytes()
    handle_message()             <-- RX state machine
      buffer_contains_ok()
      handle_message_telem()
pre_arm_check()
```

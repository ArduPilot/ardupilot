# FETtec OneWire

FETtec OneWire is an [ESC](https://en.wikipedia.org/wiki/Electronic_speed_control) communication protocol created by Felix Niessen (former Flyduino KISS developer) from [FETtec](https://fettec.net).
It is a (bidirectional) [digital full-duplex asynchronous serial communication protocol](https://en.wikipedia.org/wiki/Asynchronous_serial_communication) running at 500Kbit/s Baudrate. It requires three wire (RX, TX and GND) connection (albeit the name OneWire) regardless of the number of ESCs connected.
Unlike bidirectional-Dshot, the FETtec OneWire protocol does not need one DMA channel per ESC for bidirectional communication. 

For purchase, connection and configuration information please see the [ArduPilot FETtec OneWire wiki page](https://ardupilot.org/copter/docs/common-fettec-onewire.html).

## Features of this device driver

- use ArduPilot's coding guidelines and naming conventions
- control motor speed
- Use the AP_ESC_Telem base class to:
  - copy ESC telemetry data into MAVLink telemetry
  - save ESC telemetry data in dataflash logs
  - use RPM telemetry for dynamic notch filter frequencies
  - sum the current telemetry info from all ESCs and use it as virtual battery current monitor sensor
  - average the voltage telemetry info and use it as virtual battery voltage monitor sensor
  - average the temperature telemetry info and use it as virtual battery temperature monitor sensor
- Obey the safety switch. Keeps motors from turning
- Use `SERVO_FWT_MASK` to define which servo output should be routed to FETtec ESCs
- Use `SERVO_FWT_RVMASK` to define the rotation direction of the motors
  - `SERVO_FWT_RVMASK` changes only take effect when disarmed
- Can be compiled when `HAL_WITH_ESC_TELEM` is disabled. No telemetry data will be available but it saves a lot of memory
- pre-arm checks:
  - Check that UART is available
  - check that the desired motor channels parameter (`SERVO_FWT_MASK`) is valid
  - check that the desired motor poles parameter (`SERVO_FWT_POLES`) is valid
  - check that the all desired ESCs are found and configured
  - check that the ESCs are periodically sending telemetry data
- re-init and configure an ESC(s) if not armed (motors not spinning) when
  - telemetry communication with the ESC(s) is lost
- adds a serial simulator (--uartF=sim:fetteconewireesc) of FETtec OneWire ESCs
- adds autotest (using the simulator) to:
  - simulate telemetry voltage, current, temperature, RPM data using SITL internal variables
  - test the safety switch functionality
  - test ESC power outages
  - test `SERVO_FWT_MASK` parameter validation
  - fly a copter over a simulated serial link connection


## ArduPilot to ESC protocol

The FETtec OneWire protocol supports up to 24 ESCs. As most copters only use at most 12 motors, ArduPilot's implementation supports only 12 (`ESC_TELEM_MAX_ESCS`)to save memory.

There are two types of messages sent to the ESCs configuration and fast-throttle messages:

### Configuration message frame
Consists of six frame bytes + payload bytes.

```
    Byte 0 is the source type
    Byte 1 is the ID
    Byte 2 is the frame type (Low byte)
    Byte 3 is the frame type (High byte)
    Byte 4 is the frame length
    Byte 5-254 is the payload
    Byte 6-255 is CRC (last Byte after the Payload). It uses the same CRC algorithm as Dshot.
```

#### Check if bootloader or ESC firmware is running
To check which firmware is running on the FETtec an ESCs `PackedMessage<OK>` is sent to each ESC.
If the answer frame `frame_source` is FrameSource::BOOTLOADER we are in bootloader and need to send an `PackedMessage<START_FW>` to start the ESC firmware.
If the answer frame `frame_source` is FrameSource::ESC we are already running the correct firmware and can directly configure the telemetry.

#### Start the ESC firmware
If the ESC is running on bootloader firmware we need to send an `PackedMessage<START_FW>` to start the ESC firmware.
The answer must be a `PackedMessage<OK>` with `frame_source` equal to FrameSource::ESC. If not, we need to repeat the command.

#### Configure Full/Alternative Telemetry
The telemetry can be switched to "per ESC" Mode, where one ESC answers with it's full telemetry as oneWire package including CRC and additionally the CRC Errors counted by the ESC.
To use this mode, `PackedMessage<SET_TLM_TYPE>` is send to each ESC while initializing.
If this was successful the ESC responds with `PackedMessage<OK>`.

#### Configure Fast throttle messages
To configure the fast-throttle frame structure a `PackedMessage<SET_FAST_COM_LENGTH>` is send to each ESC while initializing.
If this was successful the ESC responds with `PackedMessage<OK>`.

### Fast-throttle message frame

```
    Byte 0 is the frame header
    Byte 1 is the telemetry request and part of fast throttle signal
    Byte N is CRC (last Byte after the Payload). It uses the same CRC algorithm as Dshot.
```
The first two bytes are frame header and telemetry request as well as the first parts of the throttle signal.
The following bytes are transmitting the throttle signals for the ESCs (11bit per ESC) followed by the CRC.
The signal is used to transfer the eleven bit throttle signals with as few bytes as possible:

```
    [990  ..    0] - negative throttle, rotation in one direction (depends on the motor wiring connection). 980 minimum throttle, 00 maximum throttle
    [991  .. 1009] - no rotation, dead-band
    [1010 .. 2000] - positive throttle, rotation in the other direction. 1020 minimum throttle, 2000 maximum throttle
```
All motors wait for the complete message with all throttle signals before changing their output.

If telemetry is requested the ESCs will answer them in the ESC-ID order.
See *ESC to ArduPilot Protocol* section below and comments in `AP_FETtecOneWire.cpp` for details.


### Timing

Four ESCs need 90us for the fast-throttle request and telemetry reception. With four ESCs 11kHz update would be possible.
Each additional ESC adds 11 extra fast-throttle command bits, so the update rate is lowered by each additional ESC.
If you use 8 ESCs, it needs 160us including telemetry response, so 5.8kHz update rate would be possible.
The FETtec ArduPilot device driver limits the message transmit period to `_min_fast_throttle_period_us` according to the number of ESCs used.
The update() function has an extra invocation period limit so that even at very high loop rates the ESCs will still operate correctly albeit doing some decimation.
The current update rate for Copter is 400Hz (~2500us) and for other vehicles is 50Hz (~20000us) so we are bellow device driver limit.

**Note:** The FETtec ESCs firmware requires at least a 4Hz fast-throttle update rate (max. 250ms between messages) otherwise the FETtec ESC disarm (stop) the motors.

## ESC to ArduPilot protocol

OneWire ESC telemetry information is sent back to the autopilot:

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


### Full/Alternative Telemetry
The answer to a fast-throttle command frame is a `PackedMessage<TLM>` message frame, received and decoded in the `FETtecOneWire::handle_message_telem()` function.
The data is forwarded to the `AP_ESC_Telem` class that distributes it to other parts of the ArduPilot code.


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
  configure_escs()               <-- TX state machine
    transmit_config_request()
      transmit()
  escs_set_values()
    pack_fast_throttle_command()
    transmit()
pre_arm_check()
```


## Device driver parameters

The `SERVO_FTW_MASK` parameter selects which servo outputs, if any, will be routed to FETtec ESCs.
You need to reboot after changing this parameter.
When `HAL_WITH_ESC_TELEM` is active (default) only the first 12 (`ESC_TELEM_MAX_ESCS`) can be used.
The FETtec ESC IDs set inside the FETtec firmware by the FETtec configuration tool are one-indexed.
These IDs must start at ID 1 and be in a single contiguous block.
You do not need to change these FETtec IDs if you change the servo output assignments inside ArduPilot the using the `SERVO_FTW_MASK` parameter.

The `SERVO_FTW_RVMASK` parameter selects which servo outputs, if any, will reverse their rotation.
This parameter effects the outputs immediately when changed and the motors are not armed.
This parameter is only visible if the `SERVO_FTW_MASK` parameter has at least one bit set.

The `SERVO_FTW_POLES` parameter selects Number of motor electrical poles.
It is used to calculate the motors RPM
This parameter effects the RPM calculation immediately when changed.
This parameter is only visible if the `SERVO_FTW_MASK` parameter has at least one bit set.

## Extra features

### Read type, firmware version and serial number
To control this you must activate the code in the header file:
```C++
// Get static info from the ESCs (optional feature)
#ifndef HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO
#define HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO 1
#endif
```
Or just set the `HAL_AP_FETTEC_ONEWIRE_GET_STATIC_INFO` macro in the compiler toolchain.
After that you will be able to access this information in the `_escs[]` datastructure.

### Beep
To control this you must activate the code in the header file:
```C++
// provide beep support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_BEEP
#define HAL_AP_FETTEC_ESC_BEEP 1
#endif
```

Or just set the `HAL_AP_FETTEC_ESC_BEEP` macro in the compiler toolchain.
After that you will be able to call the public function:

```C++
/**
    makes all connected ESCs beep
    @param beep_frequency a 8 bit value from 0-255. higher make a higher beep
*/
    void beep(const uint8_t beep_frequency);
```

### Multicolor RGB Led light
To control this you must activate the code in the header file:
```C++
// provide light support (optional feature)
#ifndef HAL_AP_FETTEC_ESC_LIGHT
#define HAL_AP_FETTEC_ESC_LIGHT 1
#endif
```

Or just set the `HAL_AP_FETTEC_ESC_LIGHT` macro in the compiler toolchain.
After that you will be able to call the public function:

```C++
/**
    makes all connected ESCs beep
    @param beep_frequency a 8 bit value from 0-255. higher make a higher beep
*/
    void beep(const uint8_t beep_frequency);

/**
    sets the racewire color for all ESCs
    @param r red brightness
    @param g green brightness
    @param b blue brightness
*/
    void led_color(const uint8_t r, const uint8_t g, const uint8_t b);
```

You need to call these functions on your own code according to your requirements.

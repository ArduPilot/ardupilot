# FETtec OneWire

FETtec OneWire is an [ESC](https://en.wikipedia.org/wiki/Electronic_speed_control) communication protocol created by Felix Niessen (former Flyduino KISS developer) from [FETtec](https://fettec.net).
It is a (bidirectional) [digital full-duplex asynchronous serial communication protocol](https://en.wikipedia.org/wiki/Asynchronous_serial_communication) running at 500Kbit/s Baudrate. It requires three wire (RX, TX and GND) connection (albeit the name OneWire) regardless of the number of ESCs connected.
Unlike bidirectional-Dshot, the FETtec OneWire protocol does not need one DMA channel per ESC for bidirectional communication. 

For purchase, connection and configuration information please see the [Ardupilot FETtec OneWire wiki page](https://ardupilot.org/copter/docs/common-fettec-onewire.html).

## Features of this device driver

- use ArduPilot's coding guidelines and naming conventions
- control motor speed
- copy ESC telemetry data into MAVLink telemetry
- save ESC telemetry data in dataflash logs
- use RPM telemetry for dynamic notch filter frequencies
- sum the current telemetry info from all ESCs and use it as virtual battery current monitor sensor
- average the voltage telemetry info and use it as virtual battery voltage monitor sensor
- average the temperature telemetry info and use it as virtual battery temperature monitor sensor
- report telemetry communication error rate in the dataflash logs
- warn the user if there is a gap in the bitmask parameter.
- re-enumerate all ESCs if not armed (motors not spinning) when
  - there is a gap in their address space IDs
  - communication with one of the ESCs is lost
  - some of the configured ESCs are not found
  - some of the configured ESCs are not correctly configured
- allows the user to configure motor rotation direction per ESC (only gets updated if not armed)
- adds a serial simulator of FETtec OneWire ESCs
- adds autotest (using the simulator) to fly a copter over a simulated serial link connection

## Ardupilot to ESC protocol

The FETtec OneWire protocol supports up to 24 ESCs. As most copters only use at most 12 motors, Ardupilot's implementation supports only 12 to save memory.

On this device driver the `SERVO_FTW_MASK` parameter must contain a single contiguous block of bits set, and bit0 (zero-indexed) must be a part of that set.
The ESC IDs (one-indexed) set on the ESCs must also be a single contiguous block, but it can start at an ID different from 1. The max ID must be lower than 12.

There are two types of messages sent to the ESCs:

### Configuration message
Consists of six frame bytes + payload bytes.

```
    Byte 0 is the transfer direction (e.g. 0x01 Master to Slave)
    Byte 1 is the ID
    Byte 2 is the frame type (Low byte)
    Byte 3 is the frame type (High byte)
    Byte 4 is the frame length
    Byte 5-254 is the payload
    Byte 6-255 is CRC (last Byte after the Payload). It uses the same CRC algorithm as Dshot.
```	

### Fast Throttle Signal

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
See *ESC to Ardupilot Protocol* section below and comments in `FETtecOneWire.cpp` for details.


### Timing

Four ESCs need 90uS for the throttle request and telemetry reception. With four ESCs 11kHz are possible.
As each additional ESC adds 11 extra fast-throttle command bits, so the rate is lowered by each ESC.
If you use 8 ESCs, it needs 160uS including telemetry response, so 5.8kHz are possible.

**Note:** You need at least a 4Hz motor signal (max 250ms between messages) before the motors disarm.

## ESC to Ardupilot protocol

OneWire ESC telemetry information is sent back to the autopilot:

- Electronic rotations per minute (eRPM/100) (must be divided by number of motor poles to translate to propeller RPM)
- Input voltage (V/10)
- Current draw (A/10)
- Power consumption (mAh)
- Temperature (°C/10)
- CRC errors (ArduPilot->ESC) counter

This information is used by Ardupilot to:

- log the status of each ESC to the SDCard or internal Flash, for post flight analysis
- send the status of each ESC to the ground station or companion computer for real-time monitoring
- Optionally dynamically change the center frequency of the notch filters used to reduce frame vibration noise in the gyros
- Optionally measure battery voltage and power consumption


## Full/Alternative Telemetry
The telemetry can be switched to "per ESC" Mode, where one ESC answers with it's full telemetry as oneWire package including CRC and additionally the CRC Errors counted by the ESC.
To use this mode, `msg_type::SET_TLM_TYPE` is send to each ESC while initializing.
If this was successful set the ESC response with `msg_type::OK`.

The answer is packed inside a OneWire package, that can be received with the `FETtecOneWire::receive()` function, that also checks the CRC.

As the packages are send in an uInt8_t array the values must be restored like as only temp is one byte long:

```C++
    Telemetry[0]= telem[0];              // Temperature [°C/10]
    Telemetry[1]=(telem[1]<<8)|telem[2]; // Voltage [V/10]
    Telemetry[2]=(telem[3]<<8)|telem[4]; // Current [A/10]
    Telemetry[3]=(telem[5]<<8)|telem[6]; // ERPM/100 (must be divided by number of motor poles to translate to propeller RPM)
    Telemetry[4]=(telem[7]<<8)|telem[8]; // Consumption [mA.h]
    Telemetry[5]=(telem[9]<<8)|telem[10];// CRC error (ArduPilot->ESC) counter
```




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
After that you will be able to access this information in the `_found_escs[]` datastructure.

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

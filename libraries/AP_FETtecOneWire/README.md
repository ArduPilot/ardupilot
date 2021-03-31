
==============
FETtec OneWire
==============

This is a digital half duplex serial protocol with a 2Mbit/s Baudrate created by FETtec.

For purchase, connection and configuration information please see the Ardupilot wiki page.


Ardupilot to ESC protocol
=========================

Unlike Dshot, the FETtec OneWire protocol does not need one DMA channel per ESC for bidirectional communication. It requires a single wire connection regardless of the number od ESCs connected.
This single wire is used in half duplex serial communication mode allowing bidirectional communication to and from all the FETtec OneWire ESCs.
FETtec OneWire protocol supports up to 22 ESCs. As most copters only use at most 12 motors the Ardupilot implementation suports only 12 to save memory.

There are two types of messages sent to the ESCs:

1. Configuration message consists of six frame bytes + payload bytes.

```
    Byte 0 is the transfer direction (e.g. 0x01 Master to Slave)
    Byte 1 is the ID
    Byte 2 is the frame type (Low byte)
    Byte 3 is the frame type (High byte)
    Byte 4 is the frame length
    Byte 5-254 is the payload
    Byte 6-255 is CRC (last Byte after the Payload). It uses the same CRC algorithm as Dshot.
```	

2. Fast Throttle Signal

```
    Byte 0 is the frame header
    Byte 1 is the telemetry request and part of fast throttle signal
    Byte N is CRC (last Byte after the Payload). It uses the same CRC algorithm as Dshot.
```	
    The structure is a bit more complex. In the first two bytes are frame header and telemetry request as well as the first parts of the throttle
    signal. The following bytes are transmitting the throttle signals for the ESCs (11bit per ESC) followed by the CRC.
    The signal is used to transfer the eleven bit throttle signals with as few bytes as possible:
```
       [0    ..  980] - negative throttle, rotation in one direction (depends on the motor wiring connection)
       [    1000    ] - no rotation
       [1020 .. 2000] - negative throttle, rotation in the other direction
```
    All motors wait for the complete message with all throttle signals before changing their output.

    If telemetry is requested the ESCs will answer them in the ESC-ID order. See *ESC to Ardupilot Protocol* section below and comments in `FETtecOneWire.cpp` for details.
	

Timming:

Four ESCs need 90uS for the throttle request and telemetry reception. With four ESCs 11kHz are possible. As each additional ESC adds 11bits	and 16 telemetry bits, so the rate is lowered by each ESC. If you use 8 ESCs, it needs 160uS including telemetry response, so 5.8kHz are possible. 
	
Note: You need at least a 4Hz motor signal (max 250ms between messages) before the motors disarm.

The FETtec configurator offers all settings. Configuration and Firmware updates are also made via OneWire so a passthough is possible.
Often used parameters are for example: 
-> Motor direction. If your motor spins in the wrong direction you can change it easily without rewiring
-> Motor beeps. Enables or disables Motor beeps
-> Soft brake. If you have props that "unscrew" if they are stopped to fast you can use the Softbreak option. 

3D Mode is not required with OneWire as it is standard. (1020-2000 is throttle forward, 980-0 is throttle backward)
	
	
ESC to Ardupilot protocol
=========================

OneWire supports ESC telemetry, so information from the ESC status is sent back to the autopilot:

- Electronic rotations per minute (eRPM/100) (must be divided by number of motor poles to translate to propeller RPM)
- Input voltage (V/10)
- Current draw (A/10)
- Power consumption (mAh)
- Temperature (°C)

This information allows the autopilot to:

- dynamically change the center frequency of the notch filters used to reduce frame vibration noise in the gyros
- log the status of each ESC to the SDCard or internal Flash, for post flight analysis
- send the status of each ESC to the Ground Station or companion computer for real-time monitoring
- measure battery voltage and power consumption


Extra features
==============

The ESC can beep and have lights. To control this you must add this code snippet to the header file:

```
public:
/**
    makes all connected ESCs beep
    @param beepFrequency a 8 bit value from 0-255. higher make a higher beep
*/
    void Beep(uint8_t beepFrequency);

/**
    sets the racewire color for all ESCs
    R, G, B = 8bit colors
*/
    void RW_LEDcolor(uint8_t R, uint8_t G, uint8_t B);
```

And this code snippet to the .cpp file:

```
/**
    makes all connected ESCs beep
    @param beepFrequency a 8 bit value from 0-255. higher make a higher beep
*/
void AP_FETtecOneWire::Beep(uint8_t beepFrequency)
{
    if (_IDcount > 0) {
        const uint8_t request[2] = {OW_BEEP, beepFrequency};
        const uint8_t request_len[1] = {2};
        const uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _minID; i < _maxID + 1; i++) {
            Transmit(i, request, request_len);
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, 2);
            _IgnoreOwnBytes += 2;
        }
    }
}

/**
    sets the racewire color for all ESCs
    @param R red brightness
    @param G green brightness
    @param B blue brightness
*/
void AP_FETtecOneWire::RW_LEDcolor(uint8_t R, uint8_t G, uint8_t B)
{
    if (_IDcount > 0) {
        const uint8_t request[4] = {OW_SET_LED_TMP_COLOR, R, G, B};
        const uint8_t request_len[1] = {4};
        const uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _minID; i < _maxID + 1; i++) {
            Transmit(i, request, request_len);
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, 2);
            _IgnoreOwnBytes += 2;
        }
    }
}
```

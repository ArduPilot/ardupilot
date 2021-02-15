==============
FETtec OneWire
==============

This is a half duplex serial protocol with a 2Mbit/s Baudrate created by Felix Niessen (former Flyduino KISS developer) from `FETtec <https://fettec.net/>`_.


Ardupilot to ESC protocol
=========================

How many bytes per message?
Which CRC? What proprieties has this CRC?
How many ESCs are supported? At which Rate?
What is the pause time between messages?
What can we configure in the ESCs?
What is the signal resolution?
Can the motors rotate in both directions? How is that done ?

ESC to Ardupilot protocol
=========================

How many bytes per message?
Which CRC? What proprieties has this CRC?

It supports ESC telemetry, so information from the ESC status can be sent back to the autopilot:

- Electronic rotations per minute (RPM) (must be divided by number of motor poles to translate to propeller RPM)
- Input voltage (V)
- Current draw (A)
- Power consumption (W)
- Temperature (°C)

At which Rate?

This information allows the autopilot to:

- dynamically change the center frequency of the notch filters used to reduce frame vibration noise in the gyros
- log the status of each ESC to the SDCard or internal Flash, for post flight analysis
- send the status of each ESC to the Ground Station or companion computer for real-time monitoring


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
        uint8_t request[2] = {OW_BEEP, beepFrequency};
        uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _minID; i < _maxID + 1; i++) {
            Transmit(i, request, FETtecOneWire_RequestLength[request[0]]);
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
        uint8_t request[4] = {OW_SET_LED_TMP_COLOR, R, G, B};
        uint8_t spacer[2] = {0, 0};
        for (uint8_t i = _minID; i < _maxID + 1; i++) {
            Transmit(i, request, FETtecOneWire_RequestLength[request[0]]);
            // add two zeros to make sure all ESCs can catch their command as we don't wait for a response here
            _uart->write(spacer, 2);
            _IgnoreOwnBytes += 2;
        }
    }
}
```

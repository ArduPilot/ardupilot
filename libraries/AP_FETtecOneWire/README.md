# FETtec OneWire

FETtec OneWire is an [ESC](https://en.wikipedia.org/wiki/Electronic_speed_control) communication protocol created by Felix Niessen (former Flyduino KISS developer) from [FETtec](https://fettec.net).
It is a (bidirectional) [digital half-duplex asynchronous serial communication protocol](https://en.wikipedia.org/wiki/Asynchronous_serial_communication) running at 2Mbit/s Baudrate. It requires a single wire connection (hence the name OneWire) regardless of the number of ESCs connected.
Unlike bidirectional-Dshot, the FETtec OneWire protocol does not need one DMA channel per ESC for bidirectional communication. 

For purchase, connection and configuration information please see the [Ardupilot FETtec OneWire wiki page](https://ardupilot.org/copter/docs/common-fettec-onewire.html).





## Ardupilot to ESC protocol

The FETtec OneWire protocol supports up to 22 ESCs. As most copters only use at most 12 motors, Ardupilot's implementation suports only 12 to save memory.

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
    The first two bytes are frame header and telemetry request as well as the first parts of the throttle signal.
    The following bytes are transmitting the throttle signals for the ESCs (11bit per ESC) followed by the CRC.
    The signal is used to transfer the eleven bit throttle signals with as few bytes as possible:
```
       [0    ..  980] - negative throttle, rotation in one direction (depends on the motor wiring connection)
       [981  .. 1019] - no rotation, dead-band
       [1020 .. 2000] - negative throttle, rotation in the other direction
```
    All motors wait for the complete message with all throttle signals before changing their output.

    If telemetry is requested the ESCs will answer them in the ESC-ID order.
    See *ESC to Ardupilot Protocol* section below and comments in `FETtecOneWire.cpp` for details.
	

### Timing

Four ESCs need 90uS for the throttle request and telemetry reception. With four ESCs 11kHz are possible. As each additional ESC adds 11bits	and 16 telemetry bits, so the rate is lowered by each ESC. If you use 8 ESCs, it needs 160uS including telemetry response, so 5.8kHz are possible. 
	
**Note:** You need at least a 4Hz motor signal (max 250ms between messages) before the motors disarm.

### Connection and Halfduplex with STM

To have reliable 2Mbit/s Baudrate the GPIO should be set to PushPull.
On STM it is a special mode that is initialized like this:

```
    gpioInit.mode = LL_GPIO_MODE_ALTERNATE;
    gpioInit.Pull = LL_GPIO_MODE_ALTERNATE;
    gpio.Pull = LL_GPIO_PULL_UP;
    gpioInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpioInit.Alternate = LL_GPIO_AF_1;
```	
	
## ESC to Ardupilot protocol

OneWire ESC telemetry information is sent back to the autopilot:

- Electronic rotations per minute (eRPM/100) (must be divided by number of motor poles to translate to propeller RPM)
- Input voltage (V/10)
- Current draw (A/10)
- Power consumption (mAh)
- Temperature (°C/10)

This information is used by Ardupilot to:

- log the status of each ESC to the SDCard or internal Flash, for post flight analysis
- send the status of each ESC to the ground station or companion computer for real-time monitoring
- Optionaly dynamically change the center frequency of the notch filters used to reduce frame vibration noise in the gyros
- Optionaly measure battery voltage and power consumption


## Full/Alternative Telemetry
The telemetry can be switched to "per ESC" Mode, where one ESC answers with it's full telemetry as oneWire package including CRC and additionally the CRC Erros counted by the ESC..
To use this mode OW_SET_TLM_TYPE must be send to each ESC. It make sense to set it while initializing.
If this was successful set the ESC response with "OW OK".

The answer is packed inside a OW package, that can be received with the FETtecOneWire::receive function, that also checks the CRC.

As the packages are send in an uInt8_t array the values must be restored like as only temp is one byte long:
        Telemetry[0]= telem[0]; //Temp
        Telemetry[1]=(telem[1]<<8)|telem[2];//Volt
        Telemetry[2]=(telem[3]<<8)|telem[4];//Current
        Telemetry[3]=(telem[5]<<8)|telem[6];//ERPM
        Telemetry[4]=(telem[7]<<8)|telem[8];//Consumption
        Telemetry[5]=(telem[9]<<8)|telem[10];//CRCerr




## Extra features

The ESC can beep and have lights. To control this you must add this code snippet to the header file:

```C++
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

```C++
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

After that you must add custom code to call these functions according to your requirements.

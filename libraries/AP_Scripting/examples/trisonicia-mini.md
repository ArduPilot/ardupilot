# Trisonica LI-550 Mini Wind Sensor Logger

This Lua script reads and logs wind sensor data from the **Trisonica LI-550 Mini** ultrasonic wind sensor connected via serial. It decodes ASCII-formatted key-value pairs and logs the data using dynamic field tags on the `W3D` logging stream.

For sensor details, visit:  
🔗 https://www.licor.com/products/trisonica/LI-550-mini

## Features

- Parses and logs ASCII data strings from the Trisonica LI-550 Mini in BIN logs 
- Logs fields using a dynamically generated tag list from the **first packet**
- Ignores extraneous fields in subsequent messages not present in the first
- Compatible with **ArduPilot scripting serial port** interface
- Supports high-speed 230400 baud data stream
- Uses BIN log name `W3D` for all data

## Parameters

Set the following parameters:

| Parameter       | Value   | Description                    |
|----------------|----------|--------------------------------|
| `SCR_ENABLE`   | `1`      | Enable Lua scripting           |
| `SERIALx_PROTOCOL` | `28` | Scripting protocol for SERIALx |


> Replace `SERIALx` with the appropriate serial port used on your hardware.

## Sensor configuration

Use Trisonica's [CLI](logger:write) to configure the unit. On Linux, you can use `screen` to interract with the device:
```bash
screen /dev/ttyUSB0 230400
```
You should see data displayed. Enter configuration mode with `Ctrl+C`.
You should now see data streaming stop and a terminal prompt `>`.

Set the Trisonica sensor baudrate to **230400** in its configuration software using the `baudrate` command like so:
```bash
baudrate 230400
```

You must also **enable all fields you want to log**. A recommended set is as follows. See the `Enabled` column.

```bash
display
```

```
-----------------------------------------------------------------------------------------
|     Name |            Description |  Tagged |      Tag | Decimals | Enabled |  Units  |
-----------------------------------------------------------------------------------------
|    IDTag |                 ID Tag |   Yes   |          |          |         |         |
|        S |          Wind Speed 3D |   Yes   |        S |     2    |         | m/s     |
|      S2D |          Wind Speed 2D |   Yes   |       S2 |     2    |         | m/s     |
|        D |   Horiz Wind Direction |   Yes   |        D |     0    |         | Degrees |
|       DV |    Vert Wind Direction |   Yes   |       DV |     0    |         | Degrees |
|        U |               U Vector |   Yes   |        U |     2    |   Yes   | m/s     |
|        V |               V Vector |   Yes   |        V |     2    |   Yes   | m/s     |
|        W |               W Vector |   Yes   |        W |     2    |   Yes   | m/s     |
|        T |            Temperature |   Yes   |        T |     2    |   Yes   | C       |
|       Cs |         Speed of Sound |   Yes   |        C |     2    |         | m/s     |
|   RHTemp |         RH Temp Sensor |   Yes   |     RHST |     2    |         | C       |
|       RH |     RH Humidity Sensor |   Yes   |     RHSH |     2    |         | %       |
|        H |               Humidity |   Yes   |        H |     2    |   Yes   | %       |
|       DP |               DewPoint |   Yes   |       DP |     2    |         | C       |
|    PTemp |   Pressure Temp Sensor |   Yes   |      PST |     2    |         | C       |
|        P |        Pressure Sensor |   Yes   |        P |          |   Yes   | hPa     |
|  Density |            Air Density |   Yes   |       AD |          |   Yes   | kg/m^3  |
|   LevelX |                Level X |   Yes   |       AX |          |   Yes   |         |
|   LevelY |                Level Y |   Yes   |       AY |          |   Yes   |         |
|   LevelZ |                Level Z |   Yes   |       AZ |          |   Yes   |         |
|    Pitch |                  Pitch |   Yes   |       PI |     1    |   Yes   | Degrees |
|     Roll |                   Roll |   Yes   |       RO |     1    |   Yes   | Degrees |
|    CTemp |           Compass Temp |   Yes   |       MT |     1    |         | C       |
|     MagX |              Compass X |   Yes   |       MX |          |         |         |
|     MagY |              Compass Y |   Yes   |       MY |          |         |         |
|     MagZ |              Compass Z |   Yes   |       MZ |          |         |         |
|  Heading |        Compass Heading |   Yes   |       MD |     0    |   Yes   | Degrees |
| TrueHead |           True Heading |   Yes   |       TD |     0    |   Yes   | Degrees |
-----------------------------------------------------------------------------------------
```

You can enable tags like so:
```bash
show T
```

Or disable them from the data stream like so:
```bash
hide T
```

You must not enable more than 12 outputs due to limitations in this script and ArduPilot's dataflash logger.

Finally, increase the output rate to the maximum of **10 Hz**.
```bash
outputrate 10
```

Once configuration is complete, exit screen with
```bash
ctr+A k y
```

If you take longer than a minute to configure, the unit will re-enter streaming mode.

## SITL Testing

You can connect a real serial device to SITL using a passthrough UART setup.
For example, using SERIAL5 connected to ttyUSB0.

```bash
./Tools/autotest/sim_vehicle.py -v Plane --console --map -A "--serial5=uart:/dev/ttyUSB0" -D
```

Then view the `W3D` entries in your favorite dataflash (BIN) log analyzer.
For  MAVExplorer, try graphing wind speeds against airspeed.

```bash
graph W3D.U W3D.W W3D.V ARSP.Airspeed
```

## Future Improvements

* Use the wind data as input to the EKF for wind speed
* Use the Trisonica API to have ArduPilot configure the correct data outputs and rates

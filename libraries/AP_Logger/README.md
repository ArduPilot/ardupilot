# Logger Notes

## Format Types

The format type specifies the amount of storage required for the entry
and how the content should be interpreted.

| Char | C Type |
|------|--------|
|a   | int16_t[32]|
|b   | int8_t|
|B   | uint8_t|
|h   | int16_t|
|H   | uint16_t|
|i   | int32_t|
|I   | uint32_t|
|f   | float|
|d   | double|
|n   | char[4]|
|N   | char[16]|
|Z   | char[64]|
|L   | int32_t latitude/longitude (so -35.1332423 becomes -351332423)|
|M   | uint8_t flight mode|
|q   | int64_t|
|Q   | uint64_t|

Legacy field types - do not use.  These have been replaced by using  the base C type and an appropriate multiplier column entry.

| Char | CType+Mult   |
|------|--------------|
|  c   | int16_t * 100|
|  C   | uint16_t * 100|
|  e   | int32_t * 100|
|  E   | uint32_t * 100|

## Units

All units here should be base units
This does mean battery capacity is here as "amp*second"
Please keep the names consistent with Tools/autotest/param_metadata/param.py:33

| Char | Unit Abbrev. | Description |
|-----|---|---|
| '-' | "" | no units e.g. Pi | or a string|
| '?' | "UNKNOWN" | Units which haven't been worked out yet....|
| 'A' | "A" | Ampere|
| 'd' | "deg" | of the angular variety | -180 to 180|
| 'b' | "B" | bytes|
| 'k' | "deg/s" | degrees per second. Degrees are NOT SI | but is some situations more user-friendly than radians|
| 'D' | "deglatitude" | degrees of latitude|
| 'e' | "deg/s/s" | degrees per second per second. Degrees are NOT SI | but is some situations more user-friendly than radians|
| 'E' | "rad/s" | radians per second|
| 'G' | "Gauss" | Gauss is not an SI unit | but 1 tesla = 10000 gauss so a simple replacement is not possible here|
| 'h' | "degheading" | 0.? to 359.?|
| 'i' | "A.s" | Ampere second|
| 'J' | "W.s" | Joule (Watt second)|
| 'l' | "l" | litres|
| 'L' | "rad/s/s" | radians per second per second|
| 'm' | "m" | metres|
| 'n' | "m/s" | metres per second|
| 'N' | "N" | Newton|
| 'o' | "m/s/s" | metres per second per second|
| 'O' | "degC" | degrees Celsius. Not SI | but Kelvin is too cumbersome for most users|
| '%' | "%" | percent|
| 'S' | "satellites" | number of satellites|
| 's' | "s" | seconds|
| 'q' | "rpm" | rounds per minute. Not SI | but sometimes more intuitive than Hertz|
| 'r' | "rad" | radians|
| 'U' | "deglongitude" | degrees of longitude|
| 'u' | "ppm" | pulses per minute|
| 'v' | "V" | Volt|
| 'P' | "Pa" | Pascal|
| 'w' | "Ohm" | Ohm|
| 'Y' | "us" | pulse width modulation in microseconds|
| 'z' | "Hz" | Hertz|
| '#' | "instance" | (e.g.)Sensor instance number|

## Multipliers

This multiplier information applies to the raw value present in the
log.  Any adjustment implied by the format field (e.g. the "centi"
in "centidegrees" is *IGNORED* for the purposes of scaling.
Essentially "format" simply tells you the C-type, and format-type h
(int16_t) is equivalent to format-type c (int16_t*100)
tl;dr a GCS shouldn't/mustn't infer any scaling from the unit name

| Char | Multiplier |
|------|------------|
| '-' | 0 |       // no multiplier e.g. a string|
| '?' | 1 |       // multipliers which haven't been worked out yet....|
| '2' | 1e2 ||
| '1' | 1e1 ||
| '0' | 1e0 ||
| 'A' | 1e-1 ||
| 'B' | 1e-2 ||
| 'C' | 1e-3 ||
| 'D' | 1e-4 ||
| 'E' | 1e-5 ||
| 'F' | 1e-6 ||
| 'G' | 1e-7 ||
| '!' | 3.6 | // (ampere*second => milliampere*hour) and (km/h => m/s)|
| '/' | 3600 | // (ampere*second => ampere*hour)|

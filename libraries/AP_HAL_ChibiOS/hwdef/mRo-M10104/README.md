# mRo CAN Power Board - M10104

The sensor is based on Allegro Micro ACS37800 power monitoring IC, which provides 0.85 mΩ primary conductor resistance for low power loss and high inrush current capabilities. 

## Electrical: 

- 50.4V 12S LiPo
- 60 Amps**Continuous**- 90 Amps Max Current Sensing
- 5.3V, 3A ultra low noise power supply

## Mechanical:

- Board Weight: 5.90g (0.208 oz)
- Board Dimensions:
  - 26mm x 26mm (1.02"x1.02")

## Parameters:

- Set `BATT_MONITORx=8` (CAN)

NOTE: The module comes with factory calibrated values. End users are encouraged to verify and tune the parameters for the best accuracy.
`BATT_V_FINE_M` and `BATT_C_FINE_M` are multipliers set to 1.0 by default to fine tune your power readings. Keep a safe margin of battery capacity for your tests until verified.

## Port pinout

### CAN+PWR Connector

6-pin Molex Clik-Mate

| Pin | Color | Signal | TTL/Voltage Level |
| --- | ----- | ------ | ----------------- |
| 1   | red   | VCC    | 5.3V              |
| 2   | black | VCC    | 5.3V              |
| 3   | black | CAN_H  |                   |
| 4   | black | CAN_L  |                   |
| 5   | black | GND    | N/A               |
| 6   | black | GND    | N/A               |

### CAN

| Pin | Color | Signal | TTL/Voltage Level |
| --- | ----- | ------ | ----------------- |
| 1   | red   | VCC    | 5V                |
| 2   | black | CAN_H     |
| 3   | black | CAN_L     |
| 4   | black | GND    | N/A

## More questions?
Contact us [info@3dr.com](mailto:info@3dr.com)

[Designed and assembled in USA]

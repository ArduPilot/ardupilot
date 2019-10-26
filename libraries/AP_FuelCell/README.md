# AP_FuelCell

support for complex power systems

## Paramiters

There are currently three parameters the first is FUELCEL_TYPE set this to 1 for Intelligent Energy 650w / 800w.

The second two parameters are bit masks for what vehicle battery failsafe actions should be taken when the fuel cell reports it internal failsafes. FUELCEL_FS_LOW is
a bitmask for internal fuel cell failsafes to the battery low failsafe action. FUELCEL_FS_CRIT is a bitmask to battery failsafe critical actions. Note that the apropate failsafe action must be set with the BATT_FS_LOW_ACT and BATT_FS_CRT_ACT parameters. Bellow is a table of values to set the FUELCEL_FS_LOW and FUELCEL_FS_CRIT and the corresponding failsafe description. As the parameters are bitmask multiple values can be added.

| Hex   | Decimal | description |
|-------|---------|-------------|
|0x80000000| 2147483648 | Stack OT #1|
|0x40000000| 1073741824 | Stack OT #2|
|0x20000000| 536870912 | Battery UV|
|0x10000000| 268435456 | Battery OT|
|0x08000000| 134217728 | No Fan|
|0x04000000| 67108864 | Fan Overrun|
|0x02000000| 33554432 | Stack OT #1|
|0x01000000| 16777216 | Stack OT #2|
|0x00800000| 8388608 | Battery UV|
|0x00400000| 4194304 | Battery OT|
|0x00200000| 2097152 | Master Start Timeout|
|0x00100000| 1048576 | Master Stop Timeout|
|0x00080000| 524288 | Start Under Pressure|
|0x00040000| 262144 | Tank Under Pressure|
|0x00020000| 131072 | Tank Low Pressure|
|0x00010000| 65536 | Safety Flag Before Master EN|

## Setup

Battery monitor type 13 must be selected for the fuel cell tank and type 14 for the fuel cell battery. Note that only type 13 will trigger failsafes. As the fuel cell provides no information about voltage the battery monitor will report a fixed voltage of 1v. This requires the battery failsafe voltage to be disabled, BATT_CRT_VOLT = 0, BATT_LOW_VOLT = 0 and BATT_ARM_VOLT = 0. Again as the fuel cell provides no information about capacity it is convenient to set the capacity to 100, BATT_CAPACITY. This allows the low and critical capacity's to map to the percentage given by the fuel cell, BATT_CRT_MAH and BATT_LOW_MAH. The fuel cell must be connected to a serial port with protocl 25: FuelCell.

##  Usage

The battery monitor will fail arming checks if:
- The diver is not healthy, it has not received any valid communication from the fuel cell in the last 5 seconds.
- The fuel cell is in any other state except running
- There is any fuel cell failsafe

The fuel cell will report any change in state and any failsafes that happen. It will not report failsafes when they clear. The fuel cell will take no action on change of state.

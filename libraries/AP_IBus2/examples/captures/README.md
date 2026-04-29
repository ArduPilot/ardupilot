# Data captures for iBus2 protocol

## ibus2-FTr8B.csv

This is a capture file from Salea's async-serial analyzer interface.

See the Wiki's [Salea replay page](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#replaying-serial-data-from-saleae-logic-data-captures)

```sh
./Tools/autotest/sim_vehicle.py  --gdb --debug -v plane -A --serial5=logic_async_csv:libraries/AP_IBus2/examples/captures/ibus2-FTr8B.csv --speedup=1 -B AP_IBus2_Slave::process_rx
```

and

```text
param set SERIAL5_PROTOCOL 52
```

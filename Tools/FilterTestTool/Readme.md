# ArduPilot IMU Filter Test Tool

**Warning: always check the onboard FFT to setup filters, this tool only simulates the effects of filtering.**

This is a tool to simulate IMU filtering on a raw IMU log.
This requires activating the RAW_IMU bit on the LOG_BITMASK parameter.
Currently only supports the primary IMU and does not use batch sampling.

To run it:

```bash
 python run_filter_test.py
```

This will open a file chooser dialog to select a log file.


Log file can also be specified from command line 

```bash
 python run_filter_test.py logfile.bin
```

To choose a smaller section of the log begin and/or end time can be specified in seconds.
E.g. to open only the log section between 60 and 120 seconds:

```bash
 python run_filter_test.py logfile.bin -b 60 -e 120
```

More info here:

  https://discuss.ardupilot.org/t/imu-filter-tool/43633
  


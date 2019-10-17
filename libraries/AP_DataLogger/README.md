# AP_DataLogger

This is a library for logging sensors that are not needed for the operation of the vehicle. The data is logged only, no action can be taken based on its value.

## Parameters

There are two parameters for this library.

DATALG_ENABLED: this should be set to 1 to enable the library.

DATALG_TYPE: this is a bitmask that will allow logging from different sources simultaneously. So far there is only one option.

- bit 1: Serial ascii logging. This will log all data received over the first serial port with protocol 24: Data Logger. The data must terminated by '\r' or '\n'. The data will be
saved to dataflash log with the name 'DTA0'. Each single log entry can have up to a maximum of 64 characters, if this number is exceeded the message will be continued in a subsequent
log entry with the same received timestamp. The data is in the format: 'logging timestamp','received timestamp','ascii data'.

## Reviewing data

The recorded data can be extracted from the dataflash log using mission planner. Load the log in the log browser (DataFlash Logs -> Review a Log). Show the data table and click on name.
This will allow the log to be filtered by name. Select 'DTA0' and filter. Right click on the data and select 'export visible', this saves the data to a csv format. Note that if your data
is recorded in csv format this will split the ascii data as well as the log information.

The resulting csv is in the format: 'Line number','Date and time','Log name','Logged timestamp','Received timestamp','data'

# DroneCAN DSDL Code Generator

This project contains tools to generate C code for packing DroneCAN messages.

## Dependencies
`pip install -r requirements.txt`

or

* `pip install empy`
* `pip install pexpect`

## How To Use

To generate C code please ensure that you have https://github.com/dronecan/pydronecan https://github.com/dronecan/DSDL 
https://github.com/dronecan/libcanard cloned alongside this project. Then run the following 
command:
```
python dronecan_dsdlc/dronecan_dsdlc.py -O <output directory> <list of namespace dirs>
# e.g. python dronecan_dsdlc/dronecan_dsdlc.py -O dsdlc_generated libraries/AP_UAVCAN/dsdl/ardupilot DSDL/uavcan
```

To run the test simply execute the following command

```
python dronecan_dsdlc/dronecan_dsdlc.py -O <output directory> <list of namespace dirs> --run-test
```

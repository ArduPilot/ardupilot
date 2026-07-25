UAVCAN namespace
================

For details, please refer to the [DroneCAN specification](http://dronecan.org/).

## Standard ID grouping

The grouping documented here is essentially a mere guideline on what data type ID should be assigned to newly added
data types. The unallocated segments of the groups can be safely changed in the future, as such a change won't affect
backward compatibility in any way.

### Messages

Currently used ranges are the following:

- [0, 16384)

| ID range             | Types                                    |
| -------------------- | ---------------------------------------- |
| [0, 4)               | protocol.dynamic_node_id.*               |
| [4, 400)             | protocol.*                               |
| [390, 400)           | protocol.dynamic_node_id.server.*        |
| [1000, 2000)         | equipment.*                              |
| [2000, 2010)         | navigation.*                             |
| [2010, 2012)         | tunnel.*                                 |
| [16370, 16384)       | protocol.debug.*                         |

### Services

Currently used ranges are the following:

- [0, 64)

| ID range/set         | Types                                    |
| -------------------- | ---------------------------------------- |
| [0, 50)              | protocol.*                               |
| [10, 20)             | protocol.param.*                         |
| [30, 40)             | protocol.dynamic_node_id.*               |
| [40, 50)             | protocol.file.*                          |
| [50, 60)             | equipment.*                              |
| 63                   | tunnel.*                                 |

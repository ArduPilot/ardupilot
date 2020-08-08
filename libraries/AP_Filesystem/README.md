# AP_Filesystem: The ArduPilot virtual filesystem layer

AP_Filesystem is a filesystem abstraction for ArduPilot that gives
ArduPilot access to both local filesystems on the flight controller
and a set of virtual filesystem abstractions.

This document is primarily intended to document the VFS interfaces
available to ground station authors via the MAVLink FTP transport.

## MAVLink FTP

ArduPilot implements the FILE_TRANSFER_PROTOCOL MAVLink message to
allow for remote file operations. This protocol allows a GCS to
transfer files to and from a flight controller. It also allows the GCS
to access some special purpose VFS interfaces for efficient access to
flight controller data using a file API.

The VFS interfaces that don't represent local filesystem objects on
the flight controller are prefixed with an '@' symbol. Currently there
are two interfaces, the @PARAM interface and the @SYS interface.

### FTP Protocol Extension

To facilitate more efficient file transfer over commonly used SiK
radios I have added an extension to the ftp burst protocol where the
'size' field in the burst read request sets the block size of the
burst replies. This helps as SiK radios do badly with very large
packets. I have found that the best results with SiK radios is
achieved with a burst read size of 110. If the size field is set to
zero then the default of the max size (239) is used.

## The @PARAM VFS

The @PARAM VFS allows a GCS to very efficiently download full or
partial parameter list from the flight controller. Currently the
@PARAM filesystem offers only a single file, called @PARAM/param.pck,
which is a packed representation of the full parameter
list. Downloading the full parameter list via this interface is a lot
faster than using the traditional mavlink parameter messages.

The @PARAM/param.pck file has a special restriction that all reads
from the file on a single file handle must be of the same size. This
allows the server to ensure that filling in of lost transfers cannot
cause a parameter value to be split across a network block, which
prevents corruption. Attempts to vary the read size after the first
read will return a failed read.

The file format of the @PARAM/param.pck file is as follows

### File header

There is a 6 byte header, consisting of 3 uint16_t values
```
  uint16_t magic # 0x671b
  uint16_t num_params
  uint16_t total_params
```
The magic value is used to give the version of the packing format. It
should have a value of 0x671b. The num_params is how many parameters
will be sent (may be less than total if client requests a subset, see
query strings below). The total_params is the total number of
parameters the flight controller has.

The header is little-endian.

### Parameter Block

After the header comes a series of variable length parameter blocks, one per
parameter. The format is:

```
    uint8_t type:4;         // AP_Param type NONE=0, INT8=1, INT16=2, INT32=3, FLOAT=4
    uint8_t flags:4;        // for future use
    uint8_t common_len:4;   // number of name bytes in common with previous entry, 0..15
    uint8_t name_len:4;     // non-common length of param name -1 (0..15)
    uint8_t name[name_len]; // name
    uint8_t data[];         // value, length given by variable type
```

There may be any number of leading zero pad bytes before the start of
the parameter block. The pad bytes are added to ensure that a
parameter value does not cross a MAVLink FTP block boundary. This
padding prevents a re-fetch of a missing block from potentially
leading to a corrupt value.

The packing format avoids sending common leading characters of
parameter names from the previous parameter. It does this by sending a
common_len field which says how many characters should be copied from
the previous parameter. This is always zero in the first block in the
file.

The name_len field is the number of non-common characters, minus one.

### Query Strings

The file name @PARAM/param.pck can optionally be extended with query
string elements to change the result. For example:

 - @PARAM/param.pck?start=50&count=10

that means to download 10 parameters starting with parameter number
50.

### Parameter Client Examples

The script Tools/scripts/param_unpack.py can be used to unpack a
param.pck file. Additionally the MAVProxy mavproxy_param.py module
implements parameter download via ftp.

## The @SYS VFS

The @SYS VFS gives access to flight controller internals. For now the
only file that is accessible is @SYS/threads.txt which gives
information on the remaining stack space for all threads. This file is
only accessible for ChibiOS builds.

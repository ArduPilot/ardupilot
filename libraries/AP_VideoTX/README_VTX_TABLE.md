# User-definable VTX band table

A Betaflight-style, user-definable table of video-transmitter **bands and
channel frequencies**, replacing the previously hardcoded band grid. The table
is the single source of truth for band/channel → frequency, is persisted on the
flight controller, and is read and written whole over MAVLink FTP.

Power levels are **not** part of this table. They are configured with the
`VTX_PWRTBL_EN` and `VTX_PWRTBL1`–`VTX_PWRTBL6` parameters, which work on every
board.

## Why

VTX channel plans vary by region and by hardware. A user table lets a pilot
disable channels they may not use, add a custom band, or match a VTX whose
channel plan is not the standard analog A/B/E/F/R raster.

## Data model (`AP_VideoTX_Table`)

Up to **12 bands × 8 channels**. Each band has a name (8 chars), a single-letter
id, an `is_factory` flag, and up to 8 channel frequencies in MHz (`0` = channel
disabled). `band+channel` resolves to a frequency, and a reverse lookup maps a
frequency back to the first matching band/channel.

`is_factory` decides how a band is commanded, as in Betaflight:

- **Factory** bands are commanded by band/channel index on SmartAudio and CRSF,
  and the VTX tunes the frequency from its own internal band map. Editing the
  frequencies of a factory band therefore changes what ArduPilot reports, not
  what those VTXs transmit.
- **Custom** bands are commanded by their literal frequency, since the VTX's
  own map does not know them. To change a band's frequencies, mark it custom.

IRC Tramp always sends the frequency, and so does SmartAudio when the VTX is in
frequency mode, so both kinds behave the same there: an edited factory band is
retuned to the table's frequency.

The table applies to the SmartAudio, IRC Tramp and CRSF VTX protocols. MSP
VTX control (used by digital video systems) is unchanged and always sends the
configured band/channel index, so custom bands and disabled channels are not
supported there: use only enabled factory bands with an MSP VTX.

When a VTX reports its state, the band/channel indices it reports are read
against the factory band map (they are the VTX's own indices), and a custom
band is recognised from the reported frequency. A custom band that shares a
frequency with a factory channel stays on the configured custom slot.

Selecting a disabled (0 MHz) channel is ignored: nothing is sent and the VTX
stays on its current frequency.

The compiled-in defaults are the historical 11 bands in the same order and with
identical frequencies, so `VTX_BAND` indices and behaviour are unchanged out of
the box.

## Build option

The band model itself is always present when VTX support is built. The
user-definable part (blob validation, persistence and `@VTX` FTP) is controlled
by `AP_VIDEOTX_TABLE_ENABLED`, which defaults on only for boards with 32 KB of
storage (and SITL, for testing). With it disabled the firmware behaves exactly as
before this feature: the default bands, no upload.

## Persistence

A user table is stored as a compact binary blob in a dedicated
`StorageVTXTable` region (see `StorageManager`). No SD card or filesystem is
required.

- The region exists only on boards with **32 KB** of storage (most H7 boards).
  On those boards it is taken from the end of the last mission area, reducing
  mission capacity from 1306 to 1280 items on Copter and from 1312 to 1286 on
  Plane and Rover. A stored mission larger than the new limit is not loaded
  after upgrading.
- Boards with less storage (most F405 boards) do not build the user-definable
  part at all and always use the default bands. If it is enabled on such a
  board anyway, an upload is **refused** rather than accepted and then lost on
  the next reboot.
- The defaults are never written to storage; the region is only written when a
  table is uploaded. A firmware update therefore never overwrites data already
  in that part of storage.

## Wire format (`@VTX/vtxtable.dat`)

The table is exposed as a single virtual file over **MAVLink FTP** at
`@VTX/vtxtable.dat`. A ground station reads the whole blob, edits it, and writes
it back. A rejected upload leaves the existing table untouched:

- On a board with no storage region, opening the file for writing fails
  (`CreateFile` is refused with `EROFS`), so the upload is refused before any
  data is sent.
- A malformed blob can only be detected once it is complete, so it is rejected
  when the file is closed (`TerminateSession` is refused). Some FTP clients,
  including pymavlink's `mavftp` used by MAVProxy, do not report that reply;
  read the table back after uploading to confirm it was accepted.

Blob layout, version 2 — little-endian, no padding:

| offset | field | type | notes |
| --- | --- | --- | --- |
| 0 | magic | u16 | `0x5654` ("VT") |
| 2 | version | u8 | `2` |
| 3 | num_bands | u8 | 1–12 |
| 4 | num_channels | u8 | 1–8 |
| 5 | bands[num_bands] | — | see below |
| … | crc | u32 | see CRC note |

Per band (`10 + num_channels*2` bytes):

| field | type | notes |
| --- | --- | --- |
| name | char[8] | zero-padded, not necessarily NUL-terminated |
| letter | char | single-char id |
| is_factory | u8 | 0 = custom, 1 = factory |
| freq[num_channels] | u16 each | MHz, 0 = channel disabled |

**CRC**: a 32-bit CRC over every byte before it, using ArduPilot's `crc_crc32`
(the standard reflected CRC-32 table, initial value `0`, **no** final XOR — this
is *not* `zlib.crc32`, which XORs the result). Reference:

```python
_TAB = []
for n in range(256):
    c = n
    for _ in range(8):
        c = (0xEDB88320 ^ (c >> 1)) if (c & 1) else (c >> 1)
    _TAB.append(c)
def ap_crc32(data, crc=0):
    for b in data:
        crc = _TAB[(crc ^ b) & 0xff] ^ (crc >> 8)
    return crc & 0xffffffff
```

## Selecting band and channel

`VTX_BAND` and `VTX_CHANNEL` index into the table and `VTX_FREQ` is derived from
them. If a frequency is set directly, it is matched back to the first band and
channel that carry it. When a new table is accepted, `VTX_FREQ` is re-derived
from `VTX_BAND` and `VTX_CHANNEL` under the new table (unless that channel is
disabled), and the VTX is retuned.

## Configurator flow

1. FTP **GET** `@VTX/vtxtable.dat` and parse the blob.
2. Edit the bands and recompute the CRC.
3. FTP **PUT** the whole blob back to `@VTX/vtxtable.dat`. It is validated and
   persisted on close.
4. FTP **GET** it again to confirm the upload was accepted.

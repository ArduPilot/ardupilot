# Interface Control Document (ICD)
## HLK-LD2451 24GHz Vehicle/Motion Speed Radar — UART Output Protocol

| | |
|---|---|
| **Document version** | 1.0 |
| **Device** | Hi-Link HLK-LD2451 |
| **Interface type** | UART (serial), asynchronous |
| **Prepared from** | Manufacturer datasheet + live captured frames |

---

## 1. Purpose

This document defines the electrical interface and UART data protocol of the HLK-LD2451 radar module, so a host microcontroller (e.g. Arduino) can reliably receive and decode target detection frames.

---

## 2. Physical / Electrical Interface

| Pin | Function | Notes |
|---|---|---|
| VIN | Power input | DC 5V, supply must provide **>300mA** |
| GND | Ground | Common ground with host MCU |
| TX | Radar → Host data out | Connect to host RX pin |
| RX | Host → Radar data in | Only needed to send config commands |
| GPIO ×2 | Auxiliary digital outputs | IO level 3.3V (not used in this ICD) |

**Key specs**

| Parameter | Value |
|---|---|
| Operating frequency | 24 GHz – 24.25 GHz (ISM band) |
| Modulation | FMCW (Frequency Modulated Continuous Wave) |
| Max detection distance | Up to 100 m (configurable 10–100 m) |
| Detection angle | ±20° horizontal, 16° vertical |
| Operating temperature | -40°C to 85°C |
| Dimensions | 70mm × 35mm |

---

## 3. UART Configuration

| Parameter | Value |
|---|---|
| Baud rate | 115200 (default) |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Flow control | None |

---

## 4. Frame Structure (Top Level)

Every message from the radar follows this fixed envelope:

```
┌─────────────┬──────────────┬─────────────────┬─────────────┐
│   HEADER    │ DATA LENGTH  │      DATA        │    TAIL     │
│  4 bytes    │   2 bytes    │  N bytes (LE)     │  4 bytes    │
│ F4 F3 F2 F1 │  LE uint16   │  (varies)         │ F8 F7 F6 F5 │
└─────────────┴──────────────┴─────────────────┴─────────────┘
```

| Field | Size | Value | Description |
|---|---|---|---|
| Header | 4 bytes | `F4 F3 F2 F1` | Fixed frame-start marker |
| Data Length | 2 bytes | little-endian uint16 | Number of bytes in the Data field only (excludes header/length/tail) |
| Data | Data Length bytes | variable | Target count + per-target blocks (see §5). Empty (0 bytes) when no target present |
| Tail | 4 bytes | `F8 F7 F6 F5` | Fixed frame-end marker |

**Total frame size** = 4 + 2 + Data Length + 4 bytes.

---

## 5. Data Field Layout

### 5.1 No target present

When Data Length = `00 00`, the Data field is empty — the frame is just header + zero-length + tail (10 bytes total).

```
F4 F3 F2 F1  00 00  F8 F7 F6 F5
```

### 5.2 One or more targets present

```
┌───────────────┬──────────────────────┬──────────────────────┬─────┐
│ Target Count  │   Target Block 1      │   Target Block 2      │ ... │
│    1 byte     │      6 bytes           │      6 bytes           │     │
└───────────────┴──────────────────────┴──────────────────────┴─────┘
```

**Data Length = 1 + (6 × Target Count)**

| Target Count | Data Length |
|---|---|
| 1 | `07 00` (7) |
| 2 | `0D 00` (13) |
| 3 | `13 00` (19) |

---

## 6. Target Block Definition (6 bytes per target)

| Offset (within block) | Byte | Field | Encoding |
|---|---|---|---|
| 0 | Alarm | `00` = inactive, `01` = alarm active |
| 1 | Angle (raw) | uint8, actual angle(°) = raw − 128 (0x80). Negative = one side, positive = other |
| 2 | Distance | uint8, meters |
| 3 | Speed Direction | `00` = departing, `01` = approaching |
| 4 | Speed | uint8, km/h |
| 5 | SNR | uint8, 0–255, higher = stronger/more confident detection |

**Notes:**
- The **Alarm** field tracks with the **Speed Direction** field in practice — when a target switches to *approaching*, Alarm typically flips to `01` (YES).
- Angle formula: raw byte is an offset value centered at `0x80` (128). `angle = raw_byte − 128`. Example: `0x92 = 146` → `146 − 128 = +18°`.
- This is a **motion-only** radar (Doppler/FMCW). A stationary object will not generate a target block — it will fall into the "no target" case (§5.1) even if physically present in the field of view.

---

## 7. Worked Examples (from live captured data)

### 7.1 Single approaching target, alarm active

```
F4 F3 F2 F1  07 00  01 01 80 02 01 03 BA  F8 F7 F6 F5
```

| Bytes | Value | Meaning |
|---|---|---|
| `F4 F3 F2 F1` | Header | Frame start |
| `07 00` | Length | 7 bytes of data follow |
| `01` | Target count | 1 target |
| `01` | Alarm | YES |
| `80` | Angle raw | 128 − 128 = **0°** |
| `02` | Distance | **2 m** |
| `01` | Direction | **Approaching** |
| `03` | Speed | **3 km/h** |
| `BA` | SNR | 186 |
| `F8 F7 F6 F5` | Tail | Frame end |

### 7.2 Single departing target, no alarm

```
F4 F3 F2 F1  07 00  01 00 65 02 00 03 FF  F8 F7 F6 F5
```

- Alarm: `00` → no
- Angle raw `65` (0x65 = 101) → 101 − 128 = **−27°**
- Distance: 2 m, Direction: `00` → **Departing**, Speed: 3 km/h, SNR: 255

### 7.3 No target detected

```
F4 F3 F2 F1  00 00  F8 F7 F6 F5
```

Just header, zero length, tail. No target/alarm/angle/etc. fields present at all.

---

## 8. Parsing State Machine (recommended host implementation)

1. **Byte-scan** incoming UART stream, matching against `F4 F3 F2 F1` sequentially. Reset match progress on any mismatch (unless the mismatching byte is itself `F4`, in which case restart the match at index 1).
2. On full header match, **read 2 bytes** → Data Length (little-endian).
3. If Data Length = 0 → read next 4 bytes, verify they equal `F8 F7 F6 F5` → "no target" frame.
4. If Data Length > 0 → read `Data Length` bytes into a buffer, then read 4 more bytes and verify tail.
5. If tail doesn't match → discard frame as corrupted, resume header scanning.
6. If Data Length exceeds a sane maximum (e.g. 64 bytes) → discard as malformed, resume scanning.
7. Parse buffer[0] as Target Count, then walk `Target Count` blocks of 6 bytes each starting at buffer offset 1.

**Reliability note:** At 115200 baud, software-emulated UART (e.g. Arduino `SoftwareSerial`) can occasionally corrupt a single byte under load — usually visible as a header byte mismatch (e.g. `F3` read as `F3` corrupted to something else), causing one frame to be dropped and correctly resynced on the next header. For zero data loss, use a hardware UART or `AltSoftSerial`.

---

## 9. Field Value Reference Summary

| Field | Byte size | Range | Unit / Encoding |
|---|---|---|---|
| Data Length | 2 | 0, 7, 13, 19 ... | bytes, little-endian |
| Target Count | 1 | 0–N | count |
| Alarm | 1 | 0 or 1 | boolean |
| Angle (raw) | 1 | 0–255 | °, offset by −128 |
| Distance | 1 | 0–100+ | meters |
| Speed Direction | 1 | 0 or 1 | 0=departing, 1=approaching |
| Speed | 1 | 0–255 | km/h |
| SNR | 1 | 0–255 | unitless, higher = better |

---

## 10. Revision History

| Version | Notes |
|---|---|
| 1.0 | Initial ICD compiled from datasheet specs and live captured serial frames |
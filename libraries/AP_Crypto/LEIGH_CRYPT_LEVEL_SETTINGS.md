# LEIGH_CRYPT_LEVEL Parameter Settings

## Parameter Overview

**Parameter Name:** `LEIGH_CRYPT_LEVEL`  
**Display Name:** Encryption Control Level  
**Type:** INT32 (Bitmask)  
**Range:** 0 to 3  
**Default:** 0 (All encryption disabled)  
**User Level:** Standard

## Description

Bitmask parameter that selectively controls what gets encrypted by AP_CRYPTO. When set to 0, all encryption is disabled and the `LEIGH_KEY` parameter value becomes readable via MAVLink for debugging purposes.

## Bitmask Values

The parameter uses a bitmask where each bit controls a different encryption feature:

| Bit | Value | Constant | Description |
|-----|-------|----------|-------------|
| 0 | 0x01 | `LEIGH_CRYPT_LOG_FILES` | Enable encryption for log files written to SD card |
| 1 | 0x02 | `LEIGH_CRYPT_USB_TERMINAL` | Enable encryption for USB terminal output (log downloads) |

## Parameter Values

| Value | Binary | Bits Set | Effect |
|-------|--------|----------|--------|
| **0** | `00` | None | **All encryption disabled**<br>- Log files written in plaintext<br>- USB terminal output (log downloads) sent in plaintext<br>- `LEIGH_KEY` parameter value is readable via MAVLink |
| **1** | `01` | Bit 0 | **Log file encryption only**<br>- Log files encrypted when written to SD card<br>- USB terminal output sent in plaintext<br>- `LEIGH_KEY` parameter value hidden (returns 0) |
| **2** | `10` | Bit 1 | **USB terminal encryption only**<br>- Log files written in plaintext<br>- USB terminal output (log downloads) encrypted<br>- `LEIGH_KEY` parameter value hidden (returns 0) |
| **3** | `11` | Bits 0 & 1 | **All encryption enabled**<br>- Log files encrypted when written to SD card<br>- USB terminal output (log downloads) encrypted<br>- `LEIGH_KEY` parameter value hidden (returns 0) |

## Usage Examples

### Disable All Encryption (Value: 0)
```
LEIGH_CRYPT_LEVEL = 0
```
- Use when you want plaintext logs for debugging
- Allows reading `LEIGH_KEY` value via MAVLink
- No encryption overhead

### Enable Log File Encryption Only (Value: 1)
```
LEIGH_CRYPT_LEVEL = 1
```
- Log files on SD card are encrypted
- Log downloads over USB are plaintext
- Useful when you want encrypted storage but readable downloads

### Enable USB Terminal Encryption Only (Value: 2)
```
LEIGH_CRYPT_LEVEL = 2
```
- Log files on SD card are plaintext
- Log downloads over USB are encrypted
- Useful when you want readable storage but encrypted downloads

### Enable All Encryption (Value: 3)
```
LEIGH_CRYPT_LEVEL = 3
```
- Both log files and USB downloads are encrypted
- Maximum security
- `LEIGH_KEY` value is hidden for security

## Implementation Details

### Code Constants
```cpp
#define LEIGH_CRYPT_LOG_FILES     0x01  // Bit 0: Encrypt log files
#define LEIGH_CRYPT_USB_TERMINAL  0x02  // Bit 1: Encrypt USB terminal
```

### Check Function
```cpp
bool AP_Crypto_Params::is_encryption_enabled(uint32_t feature) const
```
Returns `true` if the specified feature bit is set in the bitmask.

### Where It's Used

1. **Log File Encryption** (`AP_Logger_File.cpp`)
   - Checks `LEIGH_CRYPT_LOG_FILES` (bit 0)
   - Controls encryption when writing log files to SD card

2. **USB Terminal Encryption** (`AP_Logger_MAVLinkLogTransfer.cpp`)
   - Checks `LEIGH_CRYPT_USB_TERMINAL` (bit 1)
   - Controls encryption when downloading logs over USB/MAVLink

3. **LEIGH_KEY Visibility** (`GCS_Param.cpp`)
   - When `LEIGH_CRYPT_LEVEL == 0`: `LEIGH_KEY` returns actual value
   - When `LEIGH_CRYPT_LEVEL != 0`: `LEIGH_KEY` returns 0 (hidden)

## Setting via Mission Planner

1. Open Mission Planner
2. Go to **Config/Tuning** → **Full Parameter List**
3. Find parameter `LEIGH_CRYPT_LEVEL`
4. Set value (0, 1, 2, or 3)
5. Click **Write Params** to save
6. Reboot if required (check parameter description)

## Setting via MAVLink

```bash
# Disable all encryption
mavlink_param_set LEIGH_CRYPT_LEVEL 0

# Enable log file encryption only
mavlink_param_set LEIGH_CRYPT_LEVEL 1

# Enable USB terminal encryption only
mavlink_param_set LEIGH_CRYPT_LEVEL 2

# Enable all encryption
mavlink_param_set LEIGH_CRYPT_LEVEL 3
```

## Notes

- **Default Value:** 0 (all encryption disabled)
- **Security:** When encryption is enabled, `LEIGH_KEY` value is hidden for security
- **Debugging:** Set to 0 to read `LEIGH_KEY` value via MAVLink
- **Performance:** Encryption adds minimal overhead, but plaintext is faster
- **Compatibility:** Works with existing encrypted log files regardless of setting

## Related Parameters

- **LEIGH_KEY**: The encryption key parameter (INT32)
  - When `LEIGH_CRYPT_LEVEL = 0`: Value is readable
  - When `LEIGH_CRYPT_LEVEL != 0`: Value returns 0 (hidden)




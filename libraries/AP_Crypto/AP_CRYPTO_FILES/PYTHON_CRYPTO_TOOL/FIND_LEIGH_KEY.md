# Finding LEIGH_KEY Value in Firmware Logs/Config

## Overview

The `LEIGH_KEY` parameter is hidden when read via MAVLink (always returns 0), but it may be logged or stored in various places. This guide helps you find it.

## Where to Look

### 1. MAVLink Parameter Dump Files

**Location:** Usually saved by Ground Control Stations (GCS)

**File formats:**
- `.param` files (Mission Planner, QGroundControl)
- `.parm` files (some tools)
- Text files with parameter lists

**How to check:**
```bash
# Search for LEIGH_KEY in parameter files
grep -i "LEIGH_KEY" *.param *.parm *.txt 2>/dev/null

# Or search for the parameter value pattern
grep -E "LEIGH_KEY.*[0-9]+" *.param *.parm *.txt 2>/dev/null
```

**Note:** Even if LEIGH_KEY is in the file, it may show as 0 (due to hiding). Check if the parameter exists at all - if it exists, a value was likely set.

### 2. Ground Control Station (GCS) Logs

**Mission Planner:**
- Check `%APPDATA%\Mission Planner\logs\` (Windows)
- Look for parameter change logs
- Check connection logs for parameter sets

**QGroundControl:**
- Check parameter change history
- Look in application logs
- Check saved parameter files

**Other GCS:**
- Check logs for `PARAM_SET` messages
- Look for parameter change history

### 3. Firmware Serial/Console Output

If you have access to firmware console output:

**Look for:**
- Parameter change messages
- Initialization logs showing parameter values
- Debug output (if enabled)

**Example search:**
```bash
# If you have console logs
grep -i "leigh" console.log serial.log 2>/dev/null
grep -i "crypto" console.log serial.log 2>/dev/null
grep -i "key" console.log serial.log 2>/dev/null
```

### 4. SD Card Files

**Check SD card for:**
- Parameter files (`.param`, `.parm`)
- Configuration files
- Log files that might contain parameter dumps

**Location on SD card:**
- Usually in root directory or `APM/` folder
- Look for files with "param" in the name

### 5. Firmware Storage/EEPROM Dump

If you can dump firmware storage:

**Look for:**
- Parameter storage area
- `AP_Param` storage structure
- The actual `AP_Int32 leigh_key` value in memory

**Note:** This requires low-level access to the firmware's storage.

### 6. MAVLink Message Logs

If you have MAVLink message logs (`.tlog`, `.bin` files):

**Look for:**
- `PARAM_SET` messages with `param_id="LEIGH_KEY"`
- Parameter value messages
- Connection logs

**Tools:**
- `mavlogdump.py` (from pymavlink)
- Mission Planner log viewer
- MAVExplorer

**Example:**
```bash
# Extract PARAM_SET messages
mavlogdump.py --types PARAM_SET your_log.tlog | grep -i leigh
```

### 7. Build/Configuration Files

**Check:**
- Build configuration files
- Default parameter files (`.parm` files in `hwdef/` directories)
- Custom parameter sets

**Note:** These are usually defaults, not runtime values.

### 8. Firmware Source Code

**Check if LEIGH_KEY was hardcoded:**
```bash
# Search for LEIGH_KEY in source
grep -r "LEIGH_KEY" --include="*.cpp" --include="*.h" | grep -v "AP_Crypto_Params"
```

**Check for default values:**
```bash
# Look for default parameter values
grep -r "leigh_key.*=" --include="*.cpp" --include="*.h"
```

## Search Script

Here's a script to help search for LEIGH_KEY:

```bash
#!/bin/bash
# search_leigh_key.sh

echo "Searching for LEIGH_KEY references..."

# Search parameter files
echo "=== Parameter Files ==="
find . -name "*.param" -o -name "*.parm" | xargs grep -l "LEIGH_KEY" 2>/dev/null

# Search log files
echo "=== Log Files ==="
find . -name "*.log" -o -name "*.tlog" | xargs grep -l "LEIGH_KEY" 2>/dev/null

# Search text files
echo "=== Text Files ==="
find . -name "*.txt" | xargs grep -l "LEIGH_KEY" 2>/dev/null

# Search source code (excluding AP_Crypto_Params)
echo "=== Source Code ==="
grep -r "LEIGH_KEY" --include="*.cpp" --include="*.h" | grep -v "AP_Crypto_Params" | grep -v "libraries/AP_Crypto"
```

## What to Do If Found

### If LEIGH_KEY value is found:

1. **Use it with decryption tools:**
   ```bash
   python3 decrypt_working.py 00000002.tlog output.log --leigh-key <value>
   ```

2. **Or derive the key manually:**
   ```python
   import struct
   import hashlib
   
   leigh_key = <found_value>
   salt = b'LEIGH_KEY_SALT_1'
   seed_bytes = struct.pack('<i', leigh_key)
   key = hashlib.blake2b(seed_bytes + salt, digest_size=32).digest()
   ```

### If LEIGH_KEY parameter exists but value is 0:

- The parameter was set (it exists in storage)
- The value is hidden (returns 0 when read)
- You need to extract it from firmware storage directly
- Or try to find it in logs from when it was set

### If LEIGH_KEY is not found anywhere:

- The key may have been set via `AP_Crypto::store_key()` directly
- The key may have been generated via `AP_Crypto::generate_and_store_key()`
- The key is in secure storage and needs to be extracted from there

## Alternative: Extract from Firmware Storage

If you can't find LEIGH_KEY in logs, you need to extract the key directly from firmware storage:

1. **Read 40 bytes from `StorageManager::StorageKeys` area**
2. **Use `extract_key_from_storage.py` to extract the 32-byte key**

See `KEY_EXTRACTION_GUIDE.md` for detailed instructions.

## Summary

**Most likely locations:**
1. GCS parameter dump files (`.param`, `.parm`)
2. MAVLink message logs (`.tlog` files)
3. Firmware console/serial logs
4. SD card parameter files

**If not found:**
- Key is in secure storage (extract directly)
- Key was set via other methods (not LEIGH_KEY)
- Key was randomly generated






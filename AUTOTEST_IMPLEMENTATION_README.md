# ArduPlane Short Failsafe Autotest Implementation

## Overview
This implementation adds a comprehensive autotest for ArduPlane's short failsafe behavior in AUTO and GUIDED modes, specifically testing the `FS_SHORT_ACTN=0` parameter which should ignore short RC failsafes in these autonomous modes.

## Issue Addressed
**GitHub Issue:** #19190 - "Write autotest for the no-short-failsafe option in auto mode for Plane"

**Problem:** There was a regression where users setting `FS_SHORT_ACTN=0` (ignore short failsafe in AUTO mode) found that the option had no effect. The vehicle would still change modes on short RC failsafe events when it should have continued the mission uninterrupted.

**Reference:** https://discuss.ardupilot.org/t/fs-long-actn-and-fs-short-actn/77900/8

## Implementation Details

### Test Function: `ThrottleFailsafeIgnoreInAuto`

Located in: `Tools/autotest/arduplane.py` (lines 1351-1498)

The test is structured in three subtests:

#### Subtest 1: Short Failsafe Ignored in AUTO Mode
- **Setup:**
  - `FS_SHORT_ACTN=0` (no mode change in AUTO/GUIDED/LOITER)
  - `FS_LONG_ACTN=1` (RTL on long failsafe)
  - `FS_LONG_TIMEOUT=10` (10 seconds before long failsafe)
  - `THR_FAILSAFE=1` (failsafe enabled)
  
- **Test Sequence:**
  1. Load sample mission and arm in AUTO mode
  2. Wait for vehicle to reach 50-80m altitude
  3. Trigger RC failsafe via `SIM_RC_FAIL=1`
  4. Wait 3 seconds (less than long failsafe timeout)
  5. **Assert:** Vehicle stays in AUTO mode (short failsafe ignored)
  6. Wait for long failsafe timeout (~10 seconds)
  7. **Assert:** Vehicle switches to RTL mode
  8. Restore RC and disarm

#### Subtest 2: Short Failsafe Ignored in GUIDED Mode
- **Setup:** Same parameters as subtest 1
- **Test Sequence:**
  1. Takeoff and switch to GUIDED mode
  2. Send guided waypoint command
  3. Trigger RC failsafe
  4. Wait 3 seconds
  5. **Assert:** Vehicle stays in GUIDED mode

#### Subtest 3: Short Failsafe DOES Trigger When FS_SHORT_ACTN=1
- **Setup:** `FS_SHORT_ACTN=1` (CIRCLE mode on short failsafe)
- **Test Sequence:**
  1. Start mission in AUTO mode
  2. Trigger RC failsafe
  3. **Assert:** Vehicle immediately switches to CIRCLE mode
- **Purpose:** Positive control test ensuring failsafe system is working

### Parameters Tested

| Parameter | Values | Description |
|-----------|--------|-------------|
| `FS_SHORT_ACTN` | 0, 1 | 0=ignore in AUTO/GUIDED/LOITER, 1=CIRCLE |
| `FS_LONG_ACTN` | 1 | RTL on long failsafe |
| `FS_LONG_TIMEOUT` | 10 | Seconds before long failsafe triggers |
| `THR_FS_VALUE` | 960 | PWM threshold for failsafe detection |
| `THR_FAILSAFE` | 1 | Enable RC failsafe |
| `SIM_RC_FAIL` | 0, 1 | Simulator: 0=normal, 1=no pulses |

### Test Registration
The test is added to the `tests1a()` suite in ArduPlane, positioned after `ThrottleFailsafe` and before `NeedEKFToArm` (line 7955).

## Expected Behavior

### With FS_SHORT_ACTN=0 (Ignore Short Failsafe)
```
Mode: AUTO → [Short RC Failsafe] → Mode: AUTO (unchanged)
                  ↓
              [10 seconds]
                  ↓
           [Long RC Failsafe] → Mode: RTL
```

### With FS_SHORT_ACTN=1 (CIRCLE on Short Failsafe)
```
Mode: AUTO → [Short RC Failsafe] → Mode: CIRCLE (immediate)
```

## Running the Test

### Single Test Execution
```bash
cd ardupilot
./waf configure --board=sitl
./waf plane
Tools/autotest/autotest.py --vehicle=ArduPlane --test=ThrottleFailsafeIgnoreInAuto
```

### With Specific Options
```bash
# Run with verbose output
Tools/autotest/autotest.py --vehicle=ArduPlane --test=ThrottleFailsafeIgnoreInAuto --debug

# Run with speedup
Tools/autotest/autotest.py --vehicle=ArduPlane --test=ThrottleFailsafeIgnoreInAuto --speedup=10
```

### As Part of Full Test Suite
```bash
# Run all Plane tests (includes this test)
Tools/autotest/autotest.py --vehicle=ArduPlane
```

## Test Success Criteria

✅ **Test passes if:**
1. Vehicle remains in AUTO mode during short failsafe with `FS_SHORT_ACTN=0`
2. Vehicle remains in GUIDED mode during short failsafe with `FS_SHORT_ACTN=0`
3. Long failsafe correctly triggers RTL after timeout
4. Vehicle switches to CIRCLE mode with `FS_SHORT_ACTN=1` (positive control)
5. No Python exceptions or assertion failures

❌ **Test fails if:**
1. Vehicle changes mode during short failsafe when `FS_SHORT_ACTN=0`
2. Long failsafe doesn't trigger RTL after timeout
3. Positive control (FS_SHORT_ACTN=1) doesn't trigger mode change
4. RC restoration doesn't work properly

## Debugging

### Common Issues

**Issue: Test times out waiting for altitude**
- **Cause:** Mission takeoff waypoint altitude too high or wind conditions
- **Solution:** Check mission file, reduce speedup factor

**Issue: Mode changes unexpectedly**
- **Cause:** Parameter not set correctly or regression in failsafe code
- **Solution:** Check SITL logs for parameter values, verify failsafe messages

**Issue: Long failsafe doesn't trigger**
- **Cause:** `FS_LONG_TIMEOUT` too long or test timing issue
- **Solution:** Verify timeout value, check statustext messages

### Useful Debug Commands
```bash
# Watch SITL output
tail -f /tmp/ArduPlane.log

# Check parameters in test
param show FS_SHORT_ACTN
param show FS_LONG_ACTN
param show FS_LONG_TIMEOUT

# Monitor mode changes
mode
```

## Code Quality

### Style Compliance
- Follows ArduPilot Python style guidelines
- Uses f-strings for formatted output
- Proper indentation (4 spaces)
- Comprehensive comments explaining each test section

### Test Pattern Adherence
- Uses `start_subtest()` and `end_subtest()` for organized output
- Proper cleanup with `reboot_sitl()` at end
- Context collection for statustext verification
- Appropriate timeouts for all wait operations

## Related Files

### Modified Files
- `Tools/autotest/arduplane.py` - Main test implementation

### Related Documentation
- `ArduPlane/Parameters.cpp` - FS_SHORT_ACTN and FS_LONG_ACTN definitions
- `ArduPlane/failsafe.cpp` - Failsafe implementation logic

### Related Tests
- `ThrottleFailsafe` - General RC failsafe tests
- `ThrottleFailsafeFence` - Failsafe interaction with geofence
- `GCSFailsafe` - GCS heartbeat failsafe tests

## Future Enhancements

### Potential Additions
1. **Test LOITER mode:** Add subtest for `FS_SHORT_ACTN=0` in LOITER
2. **Test FS_SHORT_ACTN=2:** Test FBWA mode with zero throttle option
3. **Test FS_SHORT_ACTN=4:** Test FBWB mode option
4. **QuadPlane integration:** Test Q-mode failsafe interactions per `Q_OPTIONS`
5. **Throttle value failsafe:** Test `THR_FS_VALUE` detection in addition to `SIM_RC_FAIL`

### Coverage Expansion
- Test with different mission types (RTL_AUTOLAND, complex waypoints)
- Test failsafe recovery scenarios (RC restoration at various points)
- Test interaction with other safety features (geofence, battery failsafe)

## Contributing

When modifying this test:
1. Ensure all three subtests still pass
2. Maintain clear comments for each test section
3. Add appropriate timeouts for new wait operations
4. Update this documentation if behavior changes
5. Run full autotest suite to check for regressions

## References

- **Issue:** https://github.com/ArduPilot/ardupilot/issues/19190
- **Discussion:** https://discuss.ardupilot.org/t/fs-long-actn-and-fs-short-actn/77900/8
- **Parameter Docs:** https://ardupilot.org/plane/docs/parameters.html#fs-short-actn
- **Autotest Guide:** https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html

## License
This code follows the ArduPilot GPLv3 license.

## Author
Implementation for GitHub Issue #19190

## Version History
- **v1.0** (2025-10-26): Initial implementation
  - Comprehensive test for FS_SHORT_ACTN=0 in AUTO and GUIDED
  - Positive control test with FS_SHORT_ACTN=1
  - Long failsafe verification

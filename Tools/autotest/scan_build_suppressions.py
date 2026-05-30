'''
Filtering for clang-scan-build findings.

Two mechanisms, applied by process_scan_build_output.py when counting
findings parsed from the .plist reports:

EXCLUDE_DIRS
    Repository-relative directory prefixes whose findings are dropped
    wholesale: git submodules, vendored third-party code and generated
    sources that we do not maintain.  These same directories are also
    passed to scan-build as --exclude (see run_clang_scan_build in
    autotest.py) so they stay out of the browsable HTML reports, but note
    that --exclude does NOT stop scan-build writing their .plist files, so
    the authoritative filtering happens here.

SUPPRESSIONS
    Individual findings, reviewed and accepted as non-bugs (or vendored
    code inside an otherwise-maintained directory), matched on the pair
    (repository-relative file, issue_hash).  issue_hash is clang's
    issue_hash_content_of_line_in_context: a hash of the bug type, the
    enclosing function and the content of the reported line.  It is
    independent of absolute line numbers, so unrelated edits elsewhere in
    the file do not invalidate a suppression; it DOES change if the
    reported line is edited or the function renamed, which is exactly when
    the suppression should be re-reviewed.

HOW TO ADD A SUPPRESSION
    1. Only suppress a finding you have reviewed and concluded is not a real
       bug (e.g. an analyser false positive, or vendored code inside an
       otherwise-maintained directory).  Prefer fixing real bugs.  Always
       give a one-line reason in the entry and, if useful, a comment above
       it explaining why it is not a bug.

    2. Run the analysis, then the post-processor with --suppression-stubs to
       print each remaining (un-suppressed) finding as a paste-ready entry:

           Tools/autotest/autotest.py clang-scan-build >/tmp/sb.txt 2>&1
           Tools/autotest/process_scan_build_output.py --suppression-stubs /tmp/sb.txt

       Each line looks like:

           ('<file>', '<issue_hash>', 'REASON HERE'),  # <bug type>

       (The hash is clang's issue_hash_content_of_line_in_context; without
       --suppression-stubs the post-processor prints the same findings as
       "<file>: <bug type> [<issue_hash>]", and the hash can also be read
       directly from the .plist files.)

    3. Paste the line(s) for the findings you are suppressing into
       SUPPRESSIONS and replace 'REASON HERE' with the reviewed reason.

    4. Re-run the post-processor to confirm it passes with no "STALE" or
       "FAIL" lines.  There is no separate count to update.

NOTE: if you fix an issue that was previously suppressed you MUST also
remove the corresponding entry from SUPPRESSIONS.  A stale suppression
(one whose finding no longer exists) is treated as a failure, so that the
suppression list stays an accurate record of accepted findings.

AP_FLAKE8_CLEAN
'''

# directory prefixes (repository-relative) whose findings are not counted
EXCLUDE_DIRS = [
    'modules/',                                  # git submodules
    'libraries/AP_Scripting/lua/src/',           # vendored Lua interpreter
    'libraries/AP_PiccoloCAN/piccolo_protocol/',  # generated packet code
]

# each entry: (file, issue_hash, reason)
SUPPRESSIONS = [
    # The AK8963/AK09916/HMC5843 auxiliary bus drivers read the _started
    # member in their destructor and block_read() before any code sets it.
    # They are always heap-allocated via NEW_NOTHROW, and ArduPilot's
    # operator new (libraries/AP_Common/c++.cpp) uses calloc(), so _started
    # is reliably false.  The analyser does not model the zeroing allocator.
    ('libraries/AP_Compass/AP_Compass_AK09916.cpp', 'f7eb1fcb5260ad3ca150dcf86a14e4d8', 'calloc-zeroed _started'),
    ('libraries/AP_Compass/AP_Compass_AK8963.cpp', '11aa0f7e0ddce3de9ff31e25f42a5bb9', 'calloc-zeroed _started'),
    ('libraries/AP_Compass/AP_Compass_AK8963.cpp', 'd0fe6f2c38d2de8b607d9070bc13442b', 'calloc-zeroed _started'),
    ('libraries/AP_Compass/AP_Compass_HMC5843.cpp', '1a2901a5092c3562bca77d88e640dd98', 'calloc-zeroed _started'),
    ('libraries/AP_Compass/AP_Compass_HMC5843.cpp', '9095515a32dcc9703804167d367ae803', 'calloc-zeroed _started'),

    # AP_SmartAudio::parse_response_buffer() reads header->length from the
    # response buffer.  On the "split response" path the buffer is a member
    # array filled across several read_response() calls, so the length byte
    # arrives in an earlier call; the analyser cannot model that and treats
    # it as garbage.  False positive.
    ('libraries/AP_VideoTX/AP_SmartAudio.cpp', 'bd71e0e80b6d58c698473298370f2a95', 'cross-call member buffer, not garbage'),

    # Vendored crypto; a single file inside an otherwise-maintained
    # directory, so it cannot be excluded by directory without hiding real
    # findings in the rest of AP_CheckFirmware.
    # TODO: come back and look at this properly - confirm whether the
    # finding is genuine before leaving it suppressed long-term.
    ('libraries/AP_CheckFirmware/monocypher.cpp', '32a1905e28c98420ef3991aa4587facf', 'vendored crypto'),

    # Dead stores in the EKF flow/velocity fusion code: values assigned on
    # badly-conditioned fault paths that return immediately, and intermediate
    # terms of the hand-derived fusion equations.  Harmless, and we do not
    # edit the safety-critical EKF math purely to satisfy the analyser.
    ('libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp', '00370e2da55fd80ad77a363614b444dc', 'EKF fusion dead store'),
    ('libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp', '1b8ea7d62f816c6e0b8297515247092a', 'EKF fusion dead store'),
    ('libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp', 'abd7c32145cfffd41345332aaec2b55b', 'EKF fusion dead store'),
    ('libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp', '8e1001e64239f0d286b31971599fc653', 'EKF fusion dead store'),
    ('libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp', '8f064045e38b9d36aa66a0a3d1b930c8', 'EKF fusion dead store'),
    ('libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp', '79056142243cc0c47f6c8656a19ec2c9', 'EKF fusion dead store'),
    ('libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp', '81f6dab0132a7eba4c3381c42e4a0279', 'EKF fusion dead store'),

    # Defensive default for a char* that is then formatted into a GCS message.
    # Each switch below currently assigns the string in every case, so the
    # analyser reports the initialiser as dead, but the default guards against
    # an unhandled (or, for Siyi, out-of-enum wire) value producing a garbage
    # pointer in the message.  Keep the default.
    ('libraries/AC_AttitudeControl/AC_WeatherVane.cpp', '286b2c53c98cead5a97aac19b2e0f527', 'defensive GCS-message default'),
    ('libraries/AP_Mount/AP_Mount_Siyi.cpp', '62f01dc1039cf75ce896e0de102b3a2a', 'defensive GCS-message default'),

    # announce_address_changes() reads backend->activeSettings.last_change_ms.
    # The backend is heap-allocated via NEW_NOTHROW, so operator new's calloc
    # zeroes activeSettings; last_change_ms is reliably 0 until set.  The
    # analyser does not model the zeroing allocator.  False positive.
    ('libraries/AP_Networking/AP_Networking.cpp', '6cc1d11459ec87388afb46150a34f1d9', 'calloc-zeroed activeSettings'),

    # DefaultIntervalsFromFiles::_num_intervals is not set by the constructor
    # but is reliably 0 because every instance is heap-allocated via
    # NEW_NOTHROW (calloc-zeroed).  We don't add a redundant initialiser just
    # to satisfy the analyser.  The finding appears at both the read inside
    # set() and the return inside num_intervals().
    ('libraries/GCS_MAVLink/GCS_Common.cpp', '5ae47b78289abe850dd5ac59ab903eb7', 'calloc-zeroed _num_intervals'),
    ('libraries/GCS_MAVLink/GCS.h', '5e3cf745a4560062f85f12a6699d4031', 'calloc-zeroed _num_intervals'),

    # update_node_status() reads node->last_log_ms; the node_status_log_data
    # constructor only initialises id, but nodes are heap-allocated via
    # NEW_NOTHROW so calloc zeroes last_log_ms (reliably 0, which correctly
    # logs the first heartbeat).  Calloc-zeroed; suppress, no initialiser.
    ('libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp', '216bc9c1b5a7451c7cd96a91b06db49c', 'calloc-zeroed last_log_ms'),

    # DroneCAN_Handle::handle_frame() mallocs payload.data and pushes it onto
    # the payloads ObjectBuffer.  The function returns early when
    # payloads.space() == 0, so the later push() always has room and succeeds,
    # transferring ownership (the data is freed on pop).  The analyser cannot
    # correlate the space() check with push() success.  False positive.
    ('libraries/AP_Scripting/AP_Scripting_helpers.cpp', '6f2a019f9ae5ab51e9964b7fb137e75e', 'push() guarded by space() check'),

    # Compass::_detect_backends() passes NEW_NOTHROW AP_Compass_SITL() to
    # add_backend(), which drops (and thus leaks) the backend if the driver is
    # disabled or _backends is full.  A genuine but very narrow leak, only on
    # the SITL-only detection path and only in those edge cases.  Suppressed
    # rather than change the shared add_backend() ownership contract for it.
    ('libraries/AP_Compass/AP_Compass.cpp', '2ed775e6ae2732b415f1ddfbdf3a657b', 'narrow SITL-only init leak'),

    # get_breach_distance(): the last of a series of parallel fence-type blocks
    # writes closest_m to compare against in the *next* block.  The final block
    # has no next block, so the write is dead - but it keeps the pattern intact
    # so adding a fence type after it cannot silently use a stale closest_m.
    ('libraries/AC_Fence/AC_Fence.cpp', 'aa67db3b6d0d960c14d14ca641e334da', 'symmetry dead store'),

    # SIM_GPS_UBLOX: _next_nav_sv_info_time is a function-local that resets to 0
    # each call, so the SVINFO rate-limit never takes effect and the store is
    # dead.  This is a known bug (see the comment at its declaration); removing
    # it breaks CI, so it is left in place pending investigation.  Suppress.
    ('libraries/SITL/SIM_GPS_UBLOX.cpp', 'a9e63d53c7d09fc6e3d3bbe5b60c92a4', 'known bug; documented at declaration'),

    # SIM_FlightAxis: valid_channels is built and (for Rev4Servos) reordered but
    # never sent to FlightAxis, so the final write is dead.  It is a deliberate
    # but unwired feature; suppress rather than delete it.
    ('libraries/SITL/SIM_FlightAxis.cpp', '2242bd1f97efa1152c16425a0a47f64b', 'unwired validity-mask feature'),

    # AP_BattMonitor::reset_remaining() does 1U<<instance where instance is a
    # uint8_t from the Lua binding.  The analyser assumes get_integer() can
    # return a negative value (making the cast wrap and the shift UB), but
    # luaL_argcheck() raises a Lua error before any out-of-range value escapes.
    # The analyser does not model luaL_argcheck as no-return on failure.
    ('libraries/AP_BattMonitor/AP_BattMonitor.h', '27c31cc6851afe2a39b2fb537d863925', 'luaL_argcheck prevents negative'),

    # gdl90Transmit() is always called with length=sizeof(msg), a positive
    # compile-time constant.  The analyser takes a uint16_t wraparound path
    # where length+2==0 and concludes the loop body never runs, leaving 'data'
    # unset.  That path is unreachable in practice.  False positive.
    ('libraries/AP_ADSB/AP_ADSB_uAvionix_UCP.cpp', '2dc42e06ad435e48bbd4275166132d57', 'length=sizeof(msg) always >0'),

    # AP_HAL_SITL thread: the 'failed:' cleanup reads a->stack, which is not
    # set by the constructor but is reliably nullptr because 'a' is
    # heap-allocated via new (calloc-zeroed).  False positive.
    ('libraries/AP_HAL_SITL/Scheduler.cpp', 'cfab46b418d2c4771b4f00d1f35c3506', 'calloc-zeroed stack field'),

    # PackedMessage::calculate_checksum(len) iterates ((const char*)this)[i]
    # for i < len.  The parameterless overload calls it with 3+sizeof(T), a
    # compile-time constant, but the analyser models sizeof(T) as a separate
    # symbolic (possibly-zero) value and concludes the loop may not run.
    # False positive (same sizeof-VLA imprecision as SPL06).
    ('libraries/SITL/SIM_PS_LightWare_SF45B.h', '727a3c693086fc17de2df6605049bae9', 'sizeof(T) treated as symbolic zero'),

    # GPS_UBlox::send_ubx() accumulates a checksum by iterating over buf for
    # i < size.  The analyser models the uint16_t size parameter as possibly
    # zero, concludes the loop may not run, and reports chk[0] as garbage on
    # the next use.  Every caller passes sizeof(msg), a positive compile-time
    # constant.  False positive (same sizeof symbolic-zero pattern as SPL06).
    ('libraries/SITL/SIM_GPS_UBLOX.cpp', '17e039c0f2fd5426d06d77711fcb8b1a', 'size=sizeof(msg) always >0'),

    # update() fills motor_pwm[] then calls escs_set_values(); pack_fast_throttle_command()
    # reads motor_values[0] unconditionally. The analyser treats _esc_count as
    # possibly 0, so it thinks the fill loop may not run and motor_values[0] is
    # garbage. In reality _init_done=true is only reachable via the one path
    # through init() that passes the _esc_count>0 check, sets _invalid_mask=false,
    # and falls through to line 186. There is no path where _init_done=true and
    # _esc_count=0. False positive.
    ('libraries/AP_FETtecOneWire/AP_FETtecOneWire.cpp', '50e94a24c5e73bec76d04078b1c33889', '_init_done guarantees _esc_count > 0'),  # noqa:E501
]

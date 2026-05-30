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
    code inside an otherwise-maintained directory).  Each entry is
    (repository-relative file, {clang major version: issue_hash}, reason).

    issue_hash is clang's issue_hash_content_of_line_in_context: a hash of
    the bug type, the enclosing function and the content of the reported
    line.  It is independent of absolute line numbers, so unrelated edits
    elsewhere in the file do not invalidate a suppression; it DOES change
    if the reported line is edited or the function renamed, which is
    exactly when the suppression should be re-reviewed.

    The hash is NOT stable across clang major versions for every finding -
    some are identical from 14 through 20, others change - so an entry
    carries one hash per version we support.  A finding matches if it
    matches any of them.

    Nor does every finding exist in every release: the analyser's checkers
    change, so several entries below are recorded only from clang 16
    onwards because earlier releases do not report them at all.  An entry
    is therefore stale only when it has a hash for the version actually
    run and that hash matched nothing; an entry with no hash for that
    version simply does not apply to the run.

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

       Each line looks like, keyed by the clang version you just ran:

           ('<file>', {19: '<issue_hash>'}, 'REASON HERE'),  # <bug type>

       Re-run under another supported clang version to collect that
       version's hash for the same finding and add it to the same dict.

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

# clang major versions this list records issue hashes for.  A run under any
# other version is rejected rather than quietly checking nothing: an
# unrecorded version has no hashes to match, so every finding would look new
# and every suppression would look inapplicable.
SUPPORTED_VERSIONS = (14, 15, 16, 17, 18, 19, 20)

# directory prefixes (repository-relative) whose findings are not counted
EXCLUDE_DIRS = [
    'modules/',                                  # git submodules
    'libraries/AP_Scripting/lua/src/',           # vendored Lua interpreter
    'libraries/AP_PiccoloCAN/piccolo_protocol/',  # generated packet code
]

# each entry: (file, {clang major version: issue_hash}, reason)
SUPPRESSIONS = [
    # The AK8963/AK09916/HMC5843 auxiliary bus drivers read the _started
    # member in their destructor and block_read() before any code sets it.
    # They are always heap-allocated via NEW_NOTHROW, and ArduPilot's
    # operator new (libraries/AP_Common/c++.cpp) uses calloc(), so _started
    # is reliably false.  The analyser does not model the zeroing allocator.
    ('libraries/AP_Compass/AP_Compass_AK09916.cpp', {16: 'f7eb1fcb5260ad3ca150dcf86a14e4d8', 17: 'f7eb1fcb5260ad3ca150dcf86a14e4d8', 18: 'f7eb1fcb5260ad3ca150dcf86a14e4d8', 19: 'f7eb1fcb5260ad3ca150dcf86a14e4d8', 20: 'f7eb1fcb5260ad3ca150dcf86a14e4d8'}, 'calloc-zeroed _started'),  # noqa:E501
    ('libraries/AP_Compass/AP_Compass_AK8963.cpp', {16: '11aa0f7e0ddce3de9ff31e25f42a5bb9', 17: '11aa0f7e0ddce3de9ff31e25f42a5bb9', 18: '11aa0f7e0ddce3de9ff31e25f42a5bb9', 19: '11aa0f7e0ddce3de9ff31e25f42a5bb9', 20: '11aa0f7e0ddce3de9ff31e25f42a5bb9'}, 'calloc-zeroed _started'),  # noqa:E501
    ('libraries/AP_Compass/AP_Compass_AK8963.cpp', {16: 'd0fe6f2c38d2de8b607d9070bc13442b', 17: 'd0fe6f2c38d2de8b607d9070bc13442b', 18: 'd0fe6f2c38d2de8b607d9070bc13442b', 19: 'd0fe6f2c38d2de8b607d9070bc13442b', 20: 'd0fe6f2c38d2de8b607d9070bc13442b'}, 'calloc-zeroed _started'),  # noqa:E501
    ('libraries/AP_Compass/AP_Compass_HMC5843.cpp', {16: '1a2901a5092c3562bca77d88e640dd98', 17: '1a2901a5092c3562bca77d88e640dd98', 18: '1a2901a5092c3562bca77d88e640dd98', 19: '1a2901a5092c3562bca77d88e640dd98', 20: '1a2901a5092c3562bca77d88e640dd98'}, 'calloc-zeroed _started'),  # noqa:E501
    ('libraries/AP_Compass/AP_Compass_HMC5843.cpp', {16: '9095515a32dcc9703804167d367ae803', 17: '9095515a32dcc9703804167d367ae803', 18: '9095515a32dcc9703804167d367ae803', 19: '9095515a32dcc9703804167d367ae803', 20: '9095515a32dcc9703804167d367ae803'}, 'calloc-zeroed _started'),  # noqa:E501

    # AP_SmartAudio::parse_response_buffer() reads header->length from the
    # response buffer.  On the "split response" path the buffer is a member
    # array filled across several read_response() calls, so the length byte
    # arrives in an earlier call; the analyser cannot model that and treats
    # it as garbage.  False positive.
    ('libraries/AP_VideoTX/AP_SmartAudio.cpp', {14: 'bd71e0e80b6d58c698473298370f2a95', 15: 'bd71e0e80b6d58c698473298370f2a95', 16: 'bd71e0e80b6d58c698473298370f2a95', 17: 'bd71e0e80b6d58c698473298370f2a95', 18: 'bd71e0e80b6d58c698473298370f2a95', 19: 'bd71e0e80b6d58c698473298370f2a95', 20: 'bd71e0e80b6d58c698473298370f2a95'}, 'cross-call member buffer, not garbage'),  # noqa:E501

    # Vendored crypto; a single file inside an otherwise-maintained
    # directory, so it cannot be excluded by directory without hiding real
    # findings in the rest of AP_CheckFirmware.
    # TODO: come back and look at this properly - confirm whether the
    # finding is genuine before leaving it suppressed long-term.
    ('libraries/AP_CheckFirmware/monocypher.cpp', {14: '32a1905e28c98420ef3991aa4587facf', 15: '32a1905e28c98420ef3991aa4587facf', 16: '32a1905e28c98420ef3991aa4587facf', 17: '32a1905e28c98420ef3991aa4587facf', 18: '32a1905e28c98420ef3991aa4587facf', 19: '32a1905e28c98420ef3991aa4587facf', 20: '32a1905e28c98420ef3991aa4587facf'}, 'vendored crypto'),  # noqa:E501

    # Dead stores in the EKF flow/velocity fusion code: values assigned on
    # badly-conditioned fault paths that return immediately, and intermediate
    # terms of the hand-derived fusion equations.  Harmless, and we do not
    # edit the safety-critical EKF math purely to satisfy the analyser.
    ('libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp', {14: '00370e2da55fd80ad77a363614b444dc', 15: '00370e2da55fd80ad77a363614b444dc', 16: '00370e2da55fd80ad77a363614b444dc', 17: '00370e2da55fd80ad77a363614b444dc', 18: '00370e2da55fd80ad77a363614b444dc', 19: '00370e2da55fd80ad77a363614b444dc', 20: '00370e2da55fd80ad77a363614b444dc'}, 'EKF fusion dead store'),  # noqa:E501
    ('libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp', {14: '1b8ea7d62f816c6e0b8297515247092a', 15: '1b8ea7d62f816c6e0b8297515247092a', 16: '1b8ea7d62f816c6e0b8297515247092a', 17: '1b8ea7d62f816c6e0b8297515247092a', 18: '1b8ea7d62f816c6e0b8297515247092a', 19: '1b8ea7d62f816c6e0b8297515247092a', 20: '1b8ea7d62f816c6e0b8297515247092a'}, 'EKF fusion dead store'),  # noqa:E501
    ('libraries/AP_NavEKF2/AP_NavEKF2_OptFlowFusion.cpp', {14: 'abd7c32145cfffd41345332aaec2b55b', 15: 'abd7c32145cfffd41345332aaec2b55b', 16: 'abd7c32145cfffd41345332aaec2b55b', 17: 'abd7c32145cfffd41345332aaec2b55b', 18: 'abd7c32145cfffd41345332aaec2b55b', 19: 'abd7c32145cfffd41345332aaec2b55b', 20: 'abd7c32145cfffd41345332aaec2b55b'}, 'EKF fusion dead store'),  # noqa:E501
    ('libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp', {14: 'ca29e66e3153db30393fcd784b82b388', 15: 'ca29e66e3153db30393fcd784b82b388', 16: '8e1001e64239f0d286b31971599fc653', 17: '8e1001e64239f0d286b31971599fc653', 18: '8e1001e64239f0d286b31971599fc653', 19: '8e1001e64239f0d286b31971599fc653', 20: '8e1001e64239f0d286b31971599fc653'}, 'EKF fusion dead store'),  # noqa:E501
    ('libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp', {14: 'b3de9e26ccc2629d60c79e4dcf6a124c', 15: 'b3de9e26ccc2629d60c79e4dcf6a124c', 16: '8f064045e38b9d36aa66a0a3d1b930c8', 17: '8f064045e38b9d36aa66a0a3d1b930c8', 18: '8f064045e38b9d36aa66a0a3d1b930c8', 19: '8f064045e38b9d36aa66a0a3d1b930c8', 20: '8f064045e38b9d36aa66a0a3d1b930c8'}, 'EKF fusion dead store'),  # noqa:E501
    ('libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp', {14: '79056142243cc0c47f6c8656a19ec2c9', 15: '79056142243cc0c47f6c8656a19ec2c9', 16: '79056142243cc0c47f6c8656a19ec2c9', 17: '79056142243cc0c47f6c8656a19ec2c9', 18: '79056142243cc0c47f6c8656a19ec2c9', 19: '79056142243cc0c47f6c8656a19ec2c9', 20: '79056142243cc0c47f6c8656a19ec2c9'}, 'EKF fusion dead store'),  # noqa:E501
    ('libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp', {14: '81f6dab0132a7eba4c3381c42e4a0279', 15: '81f6dab0132a7eba4c3381c42e4a0279', 16: '81f6dab0132a7eba4c3381c42e4a0279', 17: '81f6dab0132a7eba4c3381c42e4a0279', 18: '81f6dab0132a7eba4c3381c42e4a0279', 19: '81f6dab0132a7eba4c3381c42e4a0279', 20: '81f6dab0132a7eba4c3381c42e4a0279'}, 'EKF fusion dead store'),  # noqa:E501

    # Defensive default for a char* that is then formatted into a GCS message.
    # Each switch below currently assigns the string in every case, so the
    # analyser reports the initialiser as dead, but the default guards against
    # an unhandled (or, for Siyi, out-of-enum wire) value producing a garbage
    # pointer in the message.  Keep the default.
    ('libraries/AC_AttitudeControl/AC_WeatherVane.cpp', {16: '286b2c53c98cead5a97aac19b2e0f527', 17: '286b2c53c98cead5a97aac19b2e0f527', 18: '286b2c53c98cead5a97aac19b2e0f527', 19: '286b2c53c98cead5a97aac19b2e0f527', 20: '286b2c53c98cead5a97aac19b2e0f527'}, 'defensive GCS-message default'),  # noqa:E501
    ('libraries/AP_Mount/AP_Mount_Siyi.cpp', {16: '62f01dc1039cf75ce896e0de102b3a2a', 17: '62f01dc1039cf75ce896e0de102b3a2a', 18: '62f01dc1039cf75ce896e0de102b3a2a', 19: '62f01dc1039cf75ce896e0de102b3a2a', 20: '62f01dc1039cf75ce896e0de102b3a2a'}, 'defensive GCS-message default'),  # noqa:E501

    # announce_address_changes() reads backend->activeSettings.last_change_ms.
    # The backend is heap-allocated via NEW_NOTHROW, so operator new's calloc
    # zeroes activeSettings; last_change_ms is reliably 0 until set.  The
    # analyser does not model the zeroing allocator.  False positive.
    ('libraries/AP_Networking/AP_Networking.cpp', {16: '6cc1d11459ec87388afb46150a34f1d9', 17: '6cc1d11459ec87388afb46150a34f1d9', 18: '6cc1d11459ec87388afb46150a34f1d9', 19: '6cc1d11459ec87388afb46150a34f1d9', 20: '6cc1d11459ec87388afb46150a34f1d9'}, 'calloc-zeroed activeSettings'),  # noqa:E501

    # DefaultIntervalsFromFiles::_num_intervals is not set by the constructor
    # but is reliably 0 because every instance is heap-allocated via
    # NEW_NOTHROW (calloc-zeroed).  We don't add a redundant initialiser just
    # to satisfy the analyser.  The finding appears at both the read inside
    # set() and the return inside num_intervals().
    ('libraries/GCS_MAVLink/GCS_Common.cpp', {16: '5ae47b78289abe850dd5ac59ab903eb7', 17: '5ae47b78289abe850dd5ac59ab903eb7', 18: '5ae47b78289abe850dd5ac59ab903eb7', 19: '5ae47b78289abe850dd5ac59ab903eb7', 20: '5ae47b78289abe850dd5ac59ab903eb7'}, 'calloc-zeroed _num_intervals'),  # noqa:E501
    ('libraries/GCS_MAVLink/GCS.h', {16: '5e3cf745a4560062f85f12a6699d4031', 17: '5e3cf745a4560062f85f12a6699d4031', 18: '5e3cf745a4560062f85f12a6699d4031', 19: '5e3cf745a4560062f85f12a6699d4031', 20: '5e3cf745a4560062f85f12a6699d4031'}, 'calloc-zeroed _num_intervals'),  # noqa:E501

    # update_node_status() reads node->last_log_ms; the node_status_log_data
    # constructor only initialises id, but nodes are heap-allocated via
    # NEW_NOTHROW so calloc zeroes last_log_ms (reliably 0, which correctly
    # logs the first heartbeat).  Calloc-zeroed; suppress, no initialiser.
    ('libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp', {16: '216bc9c1b5a7451c7cd96a91b06db49c', 17: '216bc9c1b5a7451c7cd96a91b06db49c', 18: '216bc9c1b5a7451c7cd96a91b06db49c', 19: '216bc9c1b5a7451c7cd96a91b06db49c', 20: '216bc9c1b5a7451c7cd96a91b06db49c'}, 'calloc-zeroed last_log_ms'),  # noqa:E501

    # DroneCAN_Handle::handle_frame() mallocs payload.data and pushes it onto
    # the payloads ObjectBuffer.  The function returns early when
    # payloads.space() == 0, so the later push() always has room and succeeds,
    # transferring ownership (the data is freed on pop).  The analyser cannot
    # correlate the space() check with push() success.  False positive.
    ('libraries/AP_Scripting/AP_Scripting_helpers.cpp', {14: '6f2a019f9ae5ab51e9964b7fb137e75e', 15: '6f2a019f9ae5ab51e9964b7fb137e75e', 16: '6f2a019f9ae5ab51e9964b7fb137e75e', 17: '6f2a019f9ae5ab51e9964b7fb137e75e', 18: '6f2a019f9ae5ab51e9964b7fb137e75e', 19: '6f2a019f9ae5ab51e9964b7fb137e75e', 20: '6f2a019f9ae5ab51e9964b7fb137e75e'}, 'push() guarded by space() check'),  # noqa:E501

    # Compass::_detect_backends() passes NEW_NOTHROW AP_Compass_SITL() to
    # add_backend(), which drops (and thus leaks) the backend if the driver is
    # disabled or _backends is full.  A genuine but very narrow leak, only on
    # the SITL-only detection path and only in those edge cases.  Suppressed
    # rather than change the shared add_backend() ownership contract for it.
    ('libraries/AP_Compass/AP_Compass.cpp', {14: '2ed775e6ae2732b415f1ddfbdf3a657b', 15: '2ed775e6ae2732b415f1ddfbdf3a657b', 16: '2ed775e6ae2732b415f1ddfbdf3a657b', 17: '2ed775e6ae2732b415f1ddfbdf3a657b', 18: '2ed775e6ae2732b415f1ddfbdf3a657b', 19: '2ed775e6ae2732b415f1ddfbdf3a657b', 20: '2ed775e6ae2732b415f1ddfbdf3a657b'}, 'narrow SITL-only init leak'),  # noqa:E501

    # get_breach_distance(): the last of a series of parallel fence-type blocks
    # writes closest_m to compare against in the *next* block.  The final block
    # has no next block, so the write is dead - but it keeps the pattern intact
    # so adding a fence type after it cannot silently use a stale closest_m.
    ('libraries/AC_Fence/AC_Fence.cpp', {14: '7cfe9441affd5c5ec88cbf1ed91ca215', 15: '7cfe9441affd5c5ec88cbf1ed91ca215', 16: 'aa67db3b6d0d960c14d14ca641e334da', 17: 'aa67db3b6d0d960c14d14ca641e334da', 18: 'aa67db3b6d0d960c14d14ca641e334da', 19: 'aa67db3b6d0d960c14d14ca641e334da', 20: 'aa67db3b6d0d960c14d14ca641e334da'}, 'symmetry dead store'),  # noqa:E501

    # SIM_GPS_UBLOX: _next_nav_sv_info_time is a function-local that resets to 0
    # each call, so the SVINFO rate-limit never takes effect and the store is
    # dead.  This is a known bug (see the comment at its declaration); removing
    # it breaks CI, so it is left in place pending investigation.  Suppress.
    ('libraries/SITL/SIM_GPS_UBLOX.cpp', {14: '736239cf60ac0a8166d10eaee9d037dc', 15: '736239cf60ac0a8166d10eaee9d037dc', 16: 'a9e63d53c7d09fc6e3d3bbe5b60c92a4', 17: 'a9e63d53c7d09fc6e3d3bbe5b60c92a4', 18: 'a9e63d53c7d09fc6e3d3bbe5b60c92a4', 19: 'a9e63d53c7d09fc6e3d3bbe5b60c92a4', 20: 'a9e63d53c7d09fc6e3d3bbe5b60c92a4'}, 'known bug; documented at declaration'),  # noqa:E501

    # SIM_FlightAxis: valid_channels is built and (for Rev4Servos) reordered but
    # never sent to FlightAxis, so the final write is dead.  It is a deliberate
    # but unwired feature; suppress rather than delete it.
    ('libraries/SITL/SIM_FlightAxis.cpp', {14: '2242bd1f97efa1152c16425a0a47f64b', 15: '2242bd1f97efa1152c16425a0a47f64b', 16: '2242bd1f97efa1152c16425a0a47f64b', 17: '2242bd1f97efa1152c16425a0a47f64b', 18: '2242bd1f97efa1152c16425a0a47f64b', 19: '2242bd1f97efa1152c16425a0a47f64b', 20: '2242bd1f97efa1152c16425a0a47f64b'}, 'unwired validity-mask feature'),  # noqa:E501

    # AP_BattMonitor::reset_remaining() does 1U<<instance where instance is a
    # uint8_t from the Lua binding.  The analyser assumes get_integer() can
    # return a negative value (making the cast wrap and the shift UB), but
    # luaL_argcheck() raises a Lua error before any out-of-range value escapes.
    # The analyser does not model luaL_argcheck as no-return on failure.
    ('libraries/AP_BattMonitor/AP_BattMonitor.h', {18: '27c31cc6851afe2a39b2fb537d863925', 19: '27c31cc6851afe2a39b2fb537d863925', 20: '27c31cc6851afe2a39b2fb537d863925'}, 'luaL_argcheck prevents negative'),  # noqa:E501

    # gdl90Transmit() is always called with length=sizeof(msg), a positive
    # compile-time constant.  The analyser takes a uint16_t wraparound path
    # where length+2==0 and concludes the loop body never runs, leaving 'data'
    # unset.  That path is unreachable in practice.  False positive.
    ('libraries/AP_ADSB/AP_ADSB_uAvionix_UCP.cpp', {14: '2dc42e06ad435e48bbd4275166132d57', 15: '2dc42e06ad435e48bbd4275166132d57', 16: '2dc42e06ad435e48bbd4275166132d57', 17: '2dc42e06ad435e48bbd4275166132d57', 18: '2dc42e06ad435e48bbd4275166132d57', 19: '2dc42e06ad435e48bbd4275166132d57', 20: '2dc42e06ad435e48bbd4275166132d57'}, 'length=sizeof(msg) always >0'),  # noqa:E501

    # AP_HAL_SITL thread: the 'failed:' cleanup reads a->stack, which is not
    # set by the constructor but is reliably nullptr because 'a' is
    # heap-allocated via new (calloc-zeroed).  False positive.
    ('libraries/AP_HAL_SITL/Scheduler.cpp', {16: 'cfab46b418d2c4771b4f00d1f35c3506', 17: 'cfab46b418d2c4771b4f00d1f35c3506', 18: 'cfab46b418d2c4771b4f00d1f35c3506', 19: 'cfab46b418d2c4771b4f00d1f35c3506', 20: 'cfab46b418d2c4771b4f00d1f35c3506'}, 'calloc-zeroed stack field'),  # noqa:E501

    # PackedMessage::calculate_checksum(len) iterates ((const char*)this)[i]
    # for i < len.  The parameterless overload calls it with 3+sizeof(T), a
    # compile-time constant, but the analyser models sizeof(T) as a separate
    # symbolic (possibly-zero) value and concludes the loop may not run.
    # False positive (same sizeof-VLA imprecision as SPL06).
    ('libraries/SITL/SIM_PS_LightWare_SF45B.h', {14: '727a3c693086fc17de2df6605049bae9', 15: '727a3c693086fc17de2df6605049bae9', 16: '727a3c693086fc17de2df6605049bae9', 17: '727a3c693086fc17de2df6605049bae9', 18: '727a3c693086fc17de2df6605049bae9', 19: '727a3c693086fc17de2df6605049bae9', 20: '727a3c693086fc17de2df6605049bae9'}, 'sizeof(T) treated as symbolic zero'),  # noqa:E501

    # GPS_UBlox::send_ubx() accumulates a checksum by iterating over buf for
    # i < size.  The analyser models the uint16_t size parameter as possibly
    # zero, concludes the loop may not run, and reports chk[0] as garbage on
    # the next use.  Every caller passes sizeof(msg), a positive compile-time
    # constant.  False positive (same sizeof symbolic-zero pattern as SPL06).
    ('libraries/SITL/SIM_GPS_UBLOX.cpp', {14: '17e039c0f2fd5426d06d77711fcb8b1a', 15: '17e039c0f2fd5426d06d77711fcb8b1a', 16: '17e039c0f2fd5426d06d77711fcb8b1a', 17: '17e039c0f2fd5426d06d77711fcb8b1a', 18: '17e039c0f2fd5426d06d77711fcb8b1a', 19: '17e039c0f2fd5426d06d77711fcb8b1a', 20: '17e039c0f2fd5426d06d77711fcb8b1a'}, 'size=sizeof(msg) always >0'),  # noqa:E501

    # update() fills motor_pwm[] then calls escs_set_values(); pack_fast_throttle_command()
    # reads motor_values[0] unconditionally. The analyser treats _esc_count as
    # possibly 0, so it thinks the fill loop may not run and motor_values[0] is
    # garbage. In reality _init_done=true is only reachable via the one path
    # through init() that passes the _esc_count>0 check, sets _invalid_mask=false,
    # and falls through to line 186. There is no path where _init_done=true and
    # _esc_count=0. False positive.
    ('libraries/AP_FETtecOneWire/AP_FETtecOneWire.cpp', {14: '50e94a24c5e73bec76d04078b1c33889', 15: '50e94a24c5e73bec76d04078b1c33889', 16: '50e94a24c5e73bec76d04078b1c33889', 17: '50e94a24c5e73bec76d04078b1c33889', 18: '50e94a24c5e73bec76d04078b1c33889', 19: '50e94a24c5e73bec76d04078b1c33889', 20: '50e94a24c5e73bec76d04078b1c33889'}, '_init_done guarantees _esc_count > 0'),  # noqa:E501

    # calc_best_rally_or_home_location() declares a RallyLocation and passes it
    # to find_nearest_rally_point(), which only writes it when it returns true.
    # The rally location is used only inside that "if", but the analyser cannot
    # correlate the return value with the write and reports the alt as garbage
    # where rally_location_to_location() scales it.  False positive.
    ('libraries/AP_Rally/AP_Rally.cpp', {14: 'fb7eae08b82d796daa7da35a45c6aadb', 15: 'fb7eae08b82d796daa7da35a45c6aadb', 16: 'f566dc2cef00494c0703227395bb2aba', 17: 'f566dc2cef00494c0703227395bb2aba', 18: 'f566dc2cef00494c0703227395bb2aba', 19: 'f566dc2cef00494c0703227395bb2aba', 20: 'f566dc2cef00494c0703227395bb2aba'}, 'only read when find_nearest_rally_point() returned true'),  # noqa:E501

    # register_custom_mode() strdup()s the mode names and hands them to
    # ModeGuidedCustom, which returns them from name()/name4() and lives for
    # the rest of the vehicle's run.  The analyser does not model ownership
    # passing into the constructor, so it reports the successful path as a
    # leak.  False positive.
    ('ArduCopter/Copter.cpp', {14: '5c69972abc745659219fcf09ba540758', 15: '5c69972abc745659219fcf09ba540758', 16: '5c69972abc745659219fcf09ba540758', 17: '5c69972abc745659219fcf09ba540758', 18: '5c69972abc745659219fcf09ba540758', 19: '5c69972abc745659219fcf09ba540758', 20: '5c69972abc745659219fcf09ba540758'}, 'ownership passes to ModeGuidedCustom'),  # full_name_copy  # noqa:E501
    ('ArduCopter/Copter.cpp', {14: '3ff61749f1a31c8b1d3e821a99909517', 15: '3ff61749f1a31c8b1d3e821a99909517', 16: '3ff61749f1a31c8b1d3e821a99909517', 17: '3ff61749f1a31c8b1d3e821a99909517', 18: '3ff61749f1a31c8b1d3e821a99909517', 19: '3ff61749f1a31c8b1d3e821a99909517', 20: '3ff61749f1a31c8b1d3e821a99909517'}, 'ownership passes to ModeGuidedCustom'),  # short_name_copy  # noqa:E501

    # takeoff_calc_roll() initialises takeoff_roll_limit_cd from roll_limit_cd
    # and both arms of the following if/else overwrite it, so the initialiser
    # is dead today.  It is kept deliberately: this is the roll limit applied
    # during takeoff, and a future branch which forgot to set it would
    # otherwise constrain nav_roll_cd with an uninitialised value.
    ('ArduPlane/takeoff.cpp', {14: '41b24d56af730ff220192cac75cbd8f8', 15: '41b24d56af730ff220192cac75cbd8f8', 16: '41b24d56af730ff220192cac75cbd8f8', 17: '41b24d56af730ff220192cac75cbd8f8', 18: '41b24d56af730ff220192cac75cbd8f8', 19: '41b24d56af730ff220192cac75cbd8f8', 20: '41b24d56af730ff220192cac75cbd8f8'}, 'defensive initialiser for a flight limit'),  # noqa:E501

    # A genuinely dead store, not a false positive: the stop_vehicle handling
    # sits past the return.  The fix is carried by its own pull request,
    # ArduPilot/ardupilot#33899, so this branch does not duplicate the patch.
    # REMOVE this entry when that PR merges; until it is removed the
    # stale-suppression check will fail.
    ('libraries/AR_WPNav/AR_WPNav_OA.cpp', {14: '1b84540936de8ef0da729d004b6c7e11', 15: '1b84540936de8ef0da729d004b6c7e11', 16: '1b84540936de8ef0da729d004b6c7e11', 17: '1b84540936de8ef0da729d004b6c7e11', 18: '1b84540936de8ef0da729d004b6c7e11', 19: '1b84540936de8ef0da729d004b6c7e11', 20: '1b84540936de8ef0da729d004b6c7e11'}, 'fixed by #33899; remove entry when that merges'),  # noqa:E501
]

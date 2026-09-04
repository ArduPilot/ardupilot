#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Fly and validate vehicles using real ChibiOS firmware and Renode physics."""

import argparse
import contextlib
import math
import os
import subprocess
import sys
import time

from pathlib import Path

from pymavlink import DFReader
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import fat_image  # noqa: E402
import test_mission as common  # noqa: E402

CANBERRA = (-35.363261, 149.165230, 584.0, 353.0)
PHYSICS_RATE_HZ = 400
F405_PHYSICS_RATE_HZ = 125
FIRMWARE_SUFFIXES = ('.apj', '.bin', '.elf', '.hex')


def is_elf(path):
    try:
        with Path(path).open('rb') as stream:
            return stream.read(4) == b'\x7fELF'
    except OSError:
        return False


def add_launch_options(command, args):
    if not args.interactive:
        command.append('--unthrottled')
    if args.renode:
        command.extend(['--renode', args.renode])
    if args.data_cache:
        command.extend(['--data-cache', args.data_cache])
    if args.usb:
        command.append('--usb')
    if args.gdb:
        command.append('--gdb')


def check_usb_helper(process, log_path):
    if process is None:
        return
    returncode = process.poll()
    if returncode is None:
        return
    detail = 'USB/IP attachment helper stopped with status %d' % returncode
    tail = common.log_tail(log_path)
    if tail:
        detail += '\n--- USB/IP helper output (tail) ---\n' + tail
    raise RuntimeError(detail)


def start_usb_helper(args, root, output_dir):
    if not args.usb:
        return None, None
    log_path = output_dir / 'usbip.log'
    with log_path.open('w') as log:
        process = subprocess.Popen(
            [
                sys.executable,
                str(root / 'Tools' / 'renode' / 'usbip_attach.py'),
                '--port', '3240',
            ],
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    return process, log_path


def wait_interactive(args, board, uart_port, renode, renode_log,
                     physics, physics_log, usb_helper, usb_log):
    print('%s is running for interactive use' % board, flush=True)
    print('MAVLink: tcp:127.0.0.1:%u' % uart_port, flush=True)
    if args.usb:
        print('USB/IP: attaching automatically (status in %s)' % usb_log,
              flush=True)
    if args.gdb:
        print(
            'firmware is halted at reset; use continue in the GDB xterm',
            flush=True,
        )
    print('press Ctrl-C to stop Renode and the physics sidecar', flush=True)
    try:
        while True:
            common.check_process(renode, renode_log)
            check_sidecar(physics, physics_log)
            check_usb_helper(usb_helper, usb_log)
            time.sleep(0.25)
    except KeyboardInterrupt:
        print('stopping interactive simulation', flush=True)


def selected_firmware(args, default):
    return args.firmware if args.firmware is not None else default


def check_sidecar(process, log_path):
    returncode = process.poll()
    if returncode is None:
        return
    detail = 'physics sidecar stopped with status %u' % returncode
    tail = common.log_tail(log_path)
    if tail:
        detail += '\n--- physics output (tail) ---\n' + tail
    raise RuntimeError(detail)


def wait_for_sidecar(process, log_path, port, timeout=20):
    deadline = time.monotonic() + timeout
    marker = 'PHYSICS_PORT %u' % port
    while time.monotonic() < deadline:
        check_sidecar(process, log_path)
        try:
            if marker in log_path.read_text(errors='replace'):
                return
        except OSError:
            pass
        time.sleep(0.1)
    raise RuntimeError('physics sidecar did not start listening')


def recv_any(connection, renode, renode_log, physics, physics_log, deadline):
    while time.monotonic() < deadline:
        common.check_process(renode, renode_log)
        check_sidecar(physics, physics_log)
        message = connection.recv_match(blocking=True, timeout=1)
        if message is not None:
            return message
    raise RuntimeError('timed out waiting for vehicle telemetry')


def wait_ready(connection, renode, renode_log, physics, physics_log, deadline):
    required = (
        mavutil.mavlink.ESTIMATOR_ATTITUDE |
        mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
        mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
        mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
        mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS
    )
    errors = (
        mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
        mavutil.mavlink.ESTIMATOR_GPS_GLITCH |
        mavutil.mavlink.ESTIMATOR_ACCEL_ERROR
    )
    position = None
    stable_positions = 0
    ekf_ready = False
    gps_fix = 0
    latest_flags = 0
    last_report = time.monotonic()
    while time.monotonic() < deadline:
        common.check_process(renode, renode_log)
        check_sidecar(physics, physics_log)
        message = connection.recv_match(blocking=True, timeout=1)
        if message is None:
            if time.monotonic() - last_report < 10:
                continue
            print(
                'waiting for navigation: GPS fix=%u EKF=0x%x stable positions=%u' %
                (gps_fix, latest_flags, stable_positions),
                flush=True,
            )
            connection.mav.request_data_stream_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                2,
                1,
            )
            last_report = time.monotonic()
            continue
        message_type = message.get_type()
        if message_type == 'GLOBAL_POSITION_INT':
            position = message
            if message.lat and message.lon and abs(message.relative_alt) < 2000:
                stable_positions += 1
            else:
                stable_positions = 0
        elif message_type == 'EKF_STATUS_REPORT':
            latest_flags = message.flags
            ekf_ready = message.flags & required == required and not message.flags & errors
        elif message_type == 'GPS_RAW_INT':
            gps_fix = message.fix_type
        elif message_type == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
        if position is not None and stable_positions >= 10 and ekf_ready:
            return position
        if time.monotonic() - last_report >= 10:
            print(
                'waiting for navigation: GPS fix=%u EKF=0x%x stable positions=%u' %
                (gps_fix, latest_flags, stable_positions),
                flush=True,
            )
            connection.mav.request_data_stream_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                2,
                1,
            )
            last_report = time.monotonic()
    raise RuntimeError('position and EKF did not become healthy')


def is_armed(message):
    return bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def monitor_until(connection, renode, renode_log, physics, physics_log,
                  deadline, predicate, description):
    latest = {}
    last_report = 0
    while time.monotonic() < deadline:
        message = recv_any(
            connection, renode, renode_log, physics, physics_log, deadline)
        message_type = message.get_type()
        latest[message_type] = message
        if message_type == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
        now = time.monotonic()
        if (now - last_report >= 5 and
                all(name in latest for name in ('GLOBAL_POSITION_INT', 'ATTITUDE', 'VFR_HUD'))):
            position = latest['GLOBAL_POSITION_INT']
            attitude = latest['ATTITUDE']
            hud = latest['VFR_HUD']
            print(
                '%s: alt=%.1fm ground=%.1fm/s air=%.1fm/s roll=%.1f pitch=%.1f' % (
                    description,
                    position.relative_alt * 0.001,
                    hud.groundspeed,
                    hud.airspeed,
                    math.degrees(attitude.roll),
                    math.degrees(attitude.pitch),
                ),
                flush=True,
            )
            last_report = now
        if predicate(latest):
            return latest
    raise RuntimeError('timed out waiting for %s' % description)


def extract_flight_log(sd_image, output):
    extract_dir = output.parent / 'logs'
    extract_dir.mkdir()
    logs = fat_image.extract_files(
        sd_image, '/APM/LOGS', '*.BIN', extract_dir)
    if len(logs) != 1:
        raise RuntimeError('expected one DataFlash log, found %u' % len(logs))
    logs[0].replace(output)


def ordered_subsequence(values, expected):
    position = 0
    for value in values:
        if position < len(expected) and value == expected[position]:
            position += 1
    return position == len(expected)


def check_plane_log(path, home_lat, home_lon):
    if path.stat().st_size < 50000:
        raise RuntimeError('downloaded log is unexpectedly small (%u bytes)' % path.stat().st_size)
    reader = DFReader.DFReader_binary(str(path), zero_time_base=True)
    modes = []
    imu = {}
    baro = {}
    airspeeds = []
    airspeed_healthy = True
    max_altitude = 0.0
    final_altitude = None
    max_distance = 0.0
    max_pitch = 0.0
    max_roll = 0.0
    max_drops = 0
    failures = []
    try:
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            message_type = message.get_type()
            if message_type == 'MODE':
                modes.append(message.ModeNum)
            elif message_type == 'IMU':
                state = imu.setdefault(message.I, {'count': 0, 'healthy': True})
                state['count'] += 1
                state['healthy'] &= bool(message.GH and message.AH)
            elif message_type == 'BARO':
                state = baro.setdefault(message.I, {'count': 0, 'healthy': True, 'pressures': set()})
                state['count'] += 1
                state['healthy'] &= bool(message.H)
                state['pressures'].add(round(message.Press, 1))
            elif message_type == 'ARSP':
                airspeeds.append(message.Airspeed)
                airspeed_healthy &= bool(message.H)
            elif message_type == 'POS':
                max_altitude = max(max_altitude, message.RelHomeAlt)
                final_altitude = message.RelHomeAlt
                max_distance = max(
                    max_distance,
                    common.distance_metres(home_lat, home_lon, message.Lat, message.Lng),
                )
            elif message_type == 'ATT':
                max_pitch = max(max_pitch, abs(message.Pitch))
                max_roll = max(max_roll, abs(message.Roll))
            elif message_type == 'DSF':
                max_drops = max(max_drops, message.Dp)
            elif message_type in ('ERR', 'IREG'):
                failures.append(message_type)
    finally:
        reader.close()

    if not ordered_subsequence(modes, (13, 12, 26)):
        raise RuntimeError('log mode sequence does not contain TAKEOFF, LOITER, AUTOLAND: %s' % modes)
    if set(imu) != {0, 1, 2}:
        raise RuntimeError('expected three IMUs, found instances %s' % sorted(imu))
    for instance, state in imu.items():
        if state['count'] < 500 or not state['healthy']:
            raise RuntimeError('IMU%u was not continuously healthy (%s)' % (instance, state))
    if set(baro) != {0, 1}:
        raise RuntimeError('expected two barometers, found instances %s' % sorted(baro))
    for instance, state in baro.items():
        if state['count'] < 100 or not state['healthy'] or len(state['pressures']) < 20:
            raise RuntimeError('barometer %u was not healthy and dynamic (%s)' % (instance, state))
    if len(airspeeds) < 100 or not airspeed_healthy or max(airspeeds) < 20 or airspeeds[-1] > 3:
        raise RuntimeError('airspeed was not healthy across flight and landing')
    if max_altitude < 38 or max_distance < 50:
        raise RuntimeError('plane did not fly the expected path')
    if final_altitude is None or abs(final_altitude) > 3:
        raise RuntimeError('plane log ends at %.1fm relative altitude' % final_altitude)
    if max_pitch > 35 or max_roll > 70:
        raise RuntimeError('unstable attitude: pitch %.1f roll %.1f' % (max_pitch, max_roll))
    if max_drops != 0 or failures:
        raise RuntimeError('flight log contains drops or errors: drops=%u errors=%s' % (max_drops, failures))
    print(
        'plane log passed: 3 IMUs, 2 barometers, airspeed, %.1fm high, '
        'pitch<=%.1f, roll<=%.1f, no drops' %
        (max_altitude, max_pitch, max_roll),
        flush=True,
    )


def wait_copter_mission(connection, renode, renode_log, physics, physics_log, deadline):
    required_waypoints = set(range(2, 2 + len(common.WAYPOINT_OFFSETS)))
    reached_waypoints = set()
    last_sequence = None
    was_armed = False
    max_altitude = 0.0
    max_distance = 0.0
    max_roll = 0.0
    max_pitch = 0.0
    home = None
    final_position = None
    sensor_health = 0
    while time.monotonic() < deadline:
        message = recv_any(
            connection, renode, renode_log, physics, physics_log, deadline)
        message_type = message.get_type()
        if message_type == 'HEARTBEAT':
            armed = is_armed(message)
            was_armed |= armed
            if was_armed and not armed:
                missing = required_waypoints - reached_waypoints
                if missing:
                    raise RuntimeError('Copter landed before mission items %s' % sorted(missing))
                if final_position is None:
                    raise RuntimeError('Copter landed without position telemetry')
                return {
                    'max_altitude': max_altitude,
                    'max_distance': max_distance,
                    'max_roll': max_roll,
                    'max_pitch': max_pitch,
                    'final_position': final_position,
                    'sensor_health': sensor_health,
                }
        elif message_type == 'GLOBAL_POSITION_INT':
            final_position = message
            latitude = message.lat * 1.0e-7
            longitude = message.lon * 1.0e-7
            if home is None:
                home = (latitude, longitude)
            max_altitude = max(max_altitude, message.relative_alt * 0.001)
            max_distance = max(
                max_distance,
                common.distance_metres(home[0], home[1], latitude, longitude),
            )
        elif message_type == 'ATTITUDE':
            max_roll = max(max_roll, abs(math.degrees(message.roll)))
            max_pitch = max(max_pitch, abs(math.degrees(message.pitch)))
        elif message_type == 'MISSION_CURRENT' and message.seq != last_sequence:
            last_sequence = message.seq
            print('mission advanced to item %u' % message.seq, flush=True)
        elif message_type == 'MISSION_ITEM_REACHED':
            reached_waypoints.add(message.seq)
        elif message_type == 'SYS_STATUS':
            sensor_health |= message.onboard_control_sensors_health
        elif message_type == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
    raise RuntimeError('Copter mission did not land before the test timeout')


def check_copter_result(result, home_lat, home_lon):
    final = result['final_position']
    final_distance = common.distance_metres(
        home_lat, home_lon, final.lat * 1.0e-7, final.lon * 1.0e-7)
    required_sensors = (
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
        mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS
    )
    if result['sensor_health'] & required_sensors != required_sensors:
        raise RuntimeError(
            'Copter sensors were not healthy: 0x%x required 0x%x' %
            (result['sensor_health'], required_sensors))
    if result['max_altitude'] < 8 or result['max_distance'] < 8:
        raise RuntimeError('Copter did not fly the expected mission')
    if abs(final.relative_alt * 0.001) > 1 or final_distance > 3:
        raise RuntimeError(
            'Copter landing was inaccurate: altitude %.1fm distance %.1fm' %
            (final.relative_alt * 0.001, final_distance))
    if result['max_roll'] > 35 or result['max_pitch'] > 35:
        raise RuntimeError(
            'Copter was unstable: roll %.1f pitch %.1f' %
            (result['max_roll'], result['max_pitch']))
    print(
        'Copter mission passed: %.1fm high, %.1fm from home, '
        'roll<=%.1f, pitch<=%.1f, sensors healthy' %
        (result['max_altitude'], result['max_distance'],
         result['max_roll'], result['max_pitch']),
        flush=True,
    )


def quadplane_mission_items(home_lat, home_lon):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    items = [
        (frame, waypoint, 0, 1, 0, 0, 0, 0, home_lat, home_lon, 0),
        (frame, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF,
         0, 1, 0, 0, 0, float('nan'), home_lat, home_lon, 30),
    ]
    for north, east, altitude in ((120, 0, 40), (120, 100, 40), (0, 100, 35)):
        latitude, longitude = common.offset_location(
            home_lat, home_lon, north, east)
        items.append(
            (frame, waypoint, 0, 1, 0, 15, 0, float('nan'),
             latitude, longitude, altitude))
    approach_lat, approach_lon = common.offset_location(
        home_lat, home_lon, -80, 0)
    items.append(
        (frame, mavutil.mavlink.MAV_CMD_DO_LAND_START,
         0, 1, 0, 0, 0, 0, approach_lat, approach_lon, 35))
    items.append(
        (frame, waypoint, 0, 1, 0, 15, 0, float('nan'),
         approach_lat, approach_lon, 30))
    items.append(
        (frame, mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND,
         0, 1, 0, 0, 0, 1, home_lat, home_lon, 0))
    return items


def wait_quadplane_mission(connection, renode, renode_log, physics, physics_log,
                           deadline, home_lat, home_lon):
    result = {
        'max_altitude': 0.0,
        'max_distance': 0.0,
        'max_airspeed': 0.0,
        'max_roll': 0.0,
        'max_pitch': 0.0,
        'vtol_states': set(),
        'transition_started': False,
        'vtol_landing': False,
        'final_position': None,
    }
    was_armed = False
    last_sequence = None
    while time.monotonic() < deadline:
        message = recv_any(
            connection, renode, renode_log, physics, physics_log, deadline)
        message_type = message.get_type()
        if message_type == 'HEARTBEAT':
            armed = is_armed(message)
            was_armed |= armed
            if was_armed and not armed:
                return result
        elif message_type == 'GLOBAL_POSITION_INT':
            result['final_position'] = message
            result['max_altitude'] = max(
                result['max_altitude'], message.relative_alt * 0.001)
            result['max_distance'] = max(
                result['max_distance'],
                common.distance_metres(
                    home_lat, home_lon,
                    message.lat * 1.0e-7, message.lon * 1.0e-7),
            )
        elif message_type == 'ATTITUDE':
            result['max_roll'] = max(
                result['max_roll'], abs(math.degrees(message.roll)))
            result['max_pitch'] = max(
                result['max_pitch'], abs(math.degrees(message.pitch)))
        elif message_type == 'VFR_HUD':
            result['max_airspeed'] = max(result['max_airspeed'], message.airspeed)
        elif message_type == 'EXTENDED_SYS_STATE':
            result['vtol_states'].add(message.vtol_state)
        elif message_type == 'MISSION_CURRENT' and message.seq != last_sequence:
            last_sequence = message.seq
            print('QuadPlane mission advanced to item %u' % message.seq, flush=True)
        elif message_type == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
            result['transition_started'] |= 'Transition started' in message.text
            result['vtol_landing'] |= 'Land descend started' in message.text
    raise RuntimeError('QuadPlane mission did not disarm before the test timeout')


def check_quadplane_result(result, home_lat, home_lon):
    final = result['final_position']
    if final is None:
        raise RuntimeError('QuadPlane landed without position telemetry')
    final_distance = common.distance_metres(
        home_lat, home_lon, final.lat * 1.0e-7, final.lon * 1.0e-7)
    multicopter = mavutil.mavlink.MAV_VTOL_STATE_MC
    fixed_wing = mavutil.mavlink.MAV_VTOL_STATE_FW
    reported_both_states = {multicopter, fixed_wing}.issubset(
        result['vtol_states'])
    if not reported_both_states and not (
            result['transition_started'] and result['vtol_landing']):
        raise RuntimeError(
            'QuadPlane did not complete both VTOL and fixed-wing flight: %s' %
            sorted(result['vtol_states']))
    if (result['max_altitude'] < 28 or result['max_distance'] < 100 or
            result['max_airspeed'] < 11):
        raise RuntimeError('QuadPlane did not fly the expected mission: %s' % result)
    if abs(final.relative_alt * 0.001) > 1 or final_distance > 10:
        raise RuntimeError(
            'QuadPlane landing was inaccurate: altitude %.1fm distance %.1fm' %
            (final.relative_alt * 0.001, final_distance))
    if result['max_roll'] > 60 or result['max_pitch'] > 40:
        raise RuntimeError(
            'QuadPlane was unstable: roll %.1f pitch %.1f' %
            (result['max_roll'], result['max_pitch']))
    print(
        'QuadPlane mission passed: %.1fm high, %.1fm range, %.1fm/s airspeed, '
        'roll<=%.1f, pitch<=%.1f' %
        (result['max_altitude'], result['max_distance'], result['max_airspeed'],
         result['max_roll'], result['max_pitch']),
        flush=True,
    )


def build_physics(root):
    print('building standalone physics sidecar', flush=True)
    subprocess.run(['./waf', 'configure', '--board', 'sitl'], cwd=root, check=True)
    subprocess.run(['./waf', '--targets', 'tool/renode-physics'], cwd=root, check=True)


def build_plane(root, debug_symbols=False):
    defaults = root / 'Tools' / 'renode' / 'tests' / 'MatekH743-plane.parm'
    print('building MatekH743 ArduPlane firmware', flush=True)
    configure = [
        './waf', 'configure', '--board', 'MatekH743',
        '--default-parameters', str(defaults),
    ]
    if debug_symbols:
        configure.append('-g')
    subprocess.run(
        configure,
        cwd=root,
        check=True,
    )
    subprocess.run(['./waf', 'plane'], cwd=root, check=True)


def build_copter(root, debug_symbols=False):
    defaults = root / 'Tools' / 'renode' / 'tests' / 'KakuteF4-copter.parm'
    print('building KakuteF4 ArduCopter firmware', flush=True)
    configure = [
        './waf', 'configure', '--board', 'KakuteF4',
        '--default-parameters', str(defaults),
    ]
    if debug_symbols:
        configure.append('-g')
    subprocess.run(
        configure,
        cwd=root,
        check=True,
    )
    subprocess.run(['./waf', 'copter'], cwd=root, check=True)


def build_quadplane(root, debug_symbols=False):
    defaults = root / 'Tools' / 'renode' / 'tests' / 'CubeOrangePlus-quadplane.parm'
    print('building CubeOrangePlus ArduPlane firmware', flush=True)
    configure = [
        './waf', 'configure', '--board', 'CubeOrangePlus',
        '--default-parameters', str(defaults),
    ]
    if debug_symbols:
        configure.append('-g')
    subprocess.run(
        configure,
        cwd=root,
        check=True,
    )
    subprocess.run(['./waf', 'plane'], cwd=root, check=True)


def run_plane(args, root, output_dir):
    if not args.skip_build:
        build_physics(root)
        if args.firmware is None:
            build_plane(root, debug_symbols=args.gdb)
    firmware = selected_firmware(
        args, root / 'build' / 'MatekH743' / 'bin' / 'arduplane')
    physics_binary = root / 'build' / 'sitl' / 'tool' / 'renode-physics'
    for binary in (firmware, physics_binary):
        if not binary.is_file():
            raise RuntimeError('required binary does not exist: %s' % binary)

    state_dir = output_dir / 'state'
    state_dir.mkdir()
    physics_log = output_dir / 'physics.log'
    renode_log = output_dir / 'renode.log'
    flight_log = output_dir / 'flight.BIN'
    physics_port = common.unused_tcp_port()
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    while len({physics_port, uart_port, monitor_port}) != 3:
        monitor_port = common.unused_tcp_port()

    with physics_log.open('w') as log:
        physics = subprocess.Popen(
            [str(physics_binary), '--physics-port', str(physics_port), '--model', 'plane'],
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    renode = None
    connection = None
    usb_helper = None
    usb_log = None
    home_lat = None
    home_lon = None
    try:
        wait_for_sidecar(physics, physics_log, physics_port)
        command = [
            sys.executable,
            str(root / 'Tools' / 'renode' / 'run.py'),
            'MatekH743',
            '--vehicle', 'arduplane',
            '--firmware', str(firmware),
            '--state-dir', str(state_dir),
            '--uart-port', str(uart_port),
            '--port', str(monitor_port),
            '--device', '{"device":"ublox-gps","port":"SERIAL3"}',
            '--device', '{"device":"ist8310-compass","port":"I2C1"}',
            '--device', '{"device":"ms4525-airspeed","port":"I2C1"}',
            '--exec', 'sysbus.physics Connect %u "plane" %.7f %.7f %.1f %.1f %u' % (
                physics_port, *CANBERRA, PHYSICS_RATE_HZ),
        ]
        add_launch_options(command, args)
        env = os.environ.copy()
        env['XDG_CONFIG_HOME'] = str(state_dir)
        env['TMPDIR'] = str(state_dir)
        with renode_log.open('w') as log:
            renode = subprocess.Popen(
                command,
                cwd=root,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        usb_helper, usb_log = start_usb_helper(args, root, output_dir)
        if args.interactive:
            wait_interactive(
                args, 'MatekH743 Plane', uart_port, renode, renode_log,
                physics, physics_log, usb_helper, usb_log)
            return
        deadline = time.monotonic() + args.timeout
        connection = common.connect_mavlink(uart_port, renode, renode_log, deadline)
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2,
            1,
        )
        home = wait_ready(
            connection, renode, renode_log, physics, physics_log, deadline)
        home_lat = home.lat * 1.0e-7
        home_lon = home.lon * 1.0e-7
        print('MatekH743 Plane ready at %.7f %.7f' % (home_lat, home_lon), flush=True)

        common.set_mode(connection, renode, renode_log, 'TAKEOFF')
        common.send_command(
            connection,
            renode,
            renode_log,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, common.FORCE_ARM_MAGIC),
        )
        monitor_until(
            connection, renode, renode_log, physics, physics_log, deadline,
            lambda messages: (
                messages.get('GLOBAL_POSITION_INT') is not None and
                messages.get('HEARTBEAT') is not None and
                messages['GLOBAL_POSITION_INT'].relative_alt >= 38000 and
                is_armed(messages['HEARTBEAT'])
            ),
            'takeoff',
        )
        print('TAKEOFF reached 38m', flush=True)

        common.set_mode(connection, renode, renode_log, 'LOITER')
        orbit = {'degrees': 0.0, 'heading': None}

        def orbit_complete(messages):
            hud = messages.get('VFR_HUD')
            if hud is None:
                return False
            if orbit['heading'] is not None:
                orbit['degrees'] += abs((hud.heading - orbit['heading'] + 180) % 360 - 180)
            orbit['heading'] = hud.heading
            return orbit['degrees'] >= 350

        monitor_until(
            connection, renode, renode_log, physics, physics_log,
            deadline, orbit_complete, 'orbit')
        print('LOITER completed %.1f degrees' % orbit['degrees'], flush=True)

        common.set_mode(connection, renode, renode_log, 'AUTOLAND')
        landed = monitor_until(
            connection, renode, renode_log, physics, physics_log, deadline,
            lambda messages: (
                messages.get('HEARTBEAT') is not None and
                not is_armed(messages['HEARTBEAT'])
            ),
            'autoland',
        )
        final = landed.get('GLOBAL_POSITION_INT')
        hud = landed.get('VFR_HUD')
        if final is None or hud is None:
            raise RuntimeError('landing telemetry was incomplete')
        distance = common.distance_metres(
            home_lat, home_lon, final.lat * 1.0e-7, final.lon * 1.0e-7)
        if abs(final.relative_alt * 0.001) > 3 or distance > 125 or hud.groundspeed > 2:
            raise RuntimeError(
                'bad landing: altitude %.1fm distance %.1fm speed %.1fm/s' %
                (final.relative_alt * 0.001, distance, hud.groundspeed))
        check_sidecar(physics, physics_log)
        print('AUTOLAND stopped and disarmed %.1fm from home' % distance, flush=True)
    finally:
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        if usb_helper is not None:
            common.stop_process_group(usb_helper)
        if renode is not None:
            common.stop_process_group(renode)
        common.stop_process_group(physics)

    extract_flight_log(state_dir / 'sdcard.img', flight_log)
    check_plane_log(flight_log, home_lat, home_lon)


def run_copter(args, root, output_dir):
    if not args.skip_build:
        build_physics(root)
        if args.firmware is None:
            build_copter(root, debug_symbols=args.gdb)
    firmware = selected_firmware(
        args, root / 'build' / 'KakuteF4' / 'bin' / 'arducopter')
    physics_binary = root / 'build' / 'sitl' / 'tool' / 'renode-physics'
    for binary in (firmware, physics_binary):
        if not binary.is_file():
            raise RuntimeError('required binary does not exist: %s' % binary)

    state_dir = output_dir / 'state'
    state_dir.mkdir()
    physics_log = output_dir / 'physics.log'
    renode_log = output_dir / 'renode.log'
    telemetry_log = output_dir / 'flight.tlog'
    physics_port = common.unused_tcp_port()
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    while len({physics_port, uart_port, monitor_port}) != 3:
        monitor_port = common.unused_tcp_port()

    with physics_log.open('w') as log:
        physics = subprocess.Popen(
            [str(physics_binary), '--physics-port', str(physics_port), '--model', 'bfx'],
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    renode = None
    connection = None
    usb_helper = None
    usb_log = None
    try:
        wait_for_sidecar(physics, physics_log, physics_port)
        command = [
            sys.executable,
            str(root / 'Tools' / 'renode' / 'run.py'),
            'KakuteF4',
            '--vehicle', 'arducopter',
            '--firmware', str(firmware),
            '--state-dir', str(state_dir),
            '--uart-port', str(uart_port),
            '--port', str(monitor_port),
            '--device', '{"device":"ublox-gps","port":"SERIAL3"}',
            '--device', '{"device":"ist8310-compass","port":"I2C0"}',
            '--exec', 'sysbus.physics Connect %u "bfx" %.7f %.7f %.1f %.1f %u' % (
                physics_port, *CANBERRA, F405_PHYSICS_RATE_HZ),
        ]
        add_launch_options(command, args)
        env = os.environ.copy()
        env['XDG_CONFIG_HOME'] = str(state_dir)
        env['TMPDIR'] = str(state_dir)
        with renode_log.open('w') as log:
            renode = subprocess.Popen(
                command,
                cwd=root,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        usb_helper, usb_log = start_usb_helper(args, root, output_dir)
        if args.interactive:
            wait_interactive(
                args, 'KakuteF4 Copter', uart_port, renode, renode_log,
                physics, physics_log, usb_helper, usb_log)
            return
        deadline = time.monotonic() + args.timeout
        connection = common.connect_mavlink(uart_port, renode, renode_log, deadline)
        connection.setup_logfile(str(telemetry_log))
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            1,
            1,
        )
        home = wait_ready(
            connection, renode, renode_log, physics, physics_log, deadline)
        home_lat = home.lat * 1.0e-7
        home_lon = home.lon * 1.0e-7
        print('KakuteF4 Copter ready at %.7f %.7f' % (home_lat, home_lon), flush=True)
        mission = common.mission_items(home_lat, home_lon)
        common.upload_mission(connection, renode, renode_log, mission)
        connection.mav.mission_set_current_send(
            connection.target_system, connection.target_component, 1)
        common.set_mode(connection, renode, renode_log, 'AUTO')
        common.send_command(
            connection,
            renode,
            renode_log,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, common.FORCE_ARM_MAGIC),
        )
        print('KakuteF4 AUTO mission started', flush=True)
        result = wait_copter_mission(
            connection, renode, renode_log, physics, physics_log, deadline)
        check_sidecar(physics, physics_log)
        check_copter_result(result, home_lat, home_lon)
    finally:
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        if usb_helper is not None:
            common.stop_process_group(usb_helper)
        if renode is not None:
            common.stop_process_group(renode)
        common.stop_process_group(physics)


# device IDs reported by a real CubeOrangePlus
CUBEORANGEPLUS_SENSOR_IDS = {
    'INS_ACC_ID': 3408930,      # ICM42688 SPI4 CS4
    'INS_GYR_ID': 3408930,
    'INS_ACC2_ID': 2883874,     # ICM20948 SPI4 CS1
    'INS_GYR2_ID': 2883874,
    'INS_ACC3_ID': 3015690,     # ICM20649 SPI1 CS4
    'INS_GYR3_ID': 3015690,
    'COMPASS_DEV_ID': 590114,   # AK09916 behind the ICM20948
    'COMPASS_DEV_ID2': 0,
    'BARO1_DEVID': 721442,      # MS5611 SPI4 CS2
    'BARO2_DEVID': 721674,      # MS5611 SPI1 CS3
    'BARO3_DEVID': 0,
}


def check_sensor_ids(connection, renode, renode_log, deadline, expected):
    '''Fail unless the detected sensors match the real board.'''
    values = {}
    last_request = 0
    while set(values) != set(expected):
        if time.monotonic() >= deadline:
            raise RuntimeError('timed out fetching sensor IDs: missing %s' %
                               sorted(set(expected) - set(values)))
        common.check_process(renode, renode_log)
        if time.monotonic() - last_request >= 5:
            for name in expected:
                if name not in values:
                    connection.param_fetch_one(name)
            last_request = time.monotonic()
        message = connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if message is not None and message.param_id in expected:
            values[message.param_id] = int(message.param_value)
    mismatched = ['%s=%u (expected %u)' % (name, values[name], expected[name])
                  for name in sorted(expected) if values[name] != expected[name]]
    if mismatched:
        raise RuntimeError('sensor IDs differ from real hardware: %s' %
                           ', '.join(mismatched))
    print('sensor IDs match real hardware', flush=True)


def run_quadplane(args, root, output_dir):
    if not args.skip_build:
        build_physics(root)
        if args.firmware is None:
            build_quadplane(root, debug_symbols=args.gdb)
    firmware = selected_firmware(
        args, root / 'build' / 'CubeOrangePlus' / 'bin' / 'arduplane')
    physics_binary = root / 'build' / 'sitl' / 'tool' / 'renode-physics'
    for binary in (firmware, physics_binary):
        if not binary.is_file():
            raise RuntimeError('required binary does not exist: %s' % binary)

    state_dir = output_dir / 'state'
    state_dir.mkdir()
    physics_log = output_dir / 'physics.log'
    renode_log = output_dir / 'renode.log'
    telemetry_log = output_dir / 'flight.tlog'
    physics_port = common.unused_tcp_port()
    uart_port = common.unused_tcp_port()
    monitor_port = common.unused_tcp_port()
    while len({physics_port, uart_port, monitor_port}) != 3:
        monitor_port = common.unused_tcp_port()

    with physics_log.open('w') as log:
        physics = subprocess.Popen(
            [str(physics_binary), '--physics-port', str(physics_port),
             '--model', 'quadplane'],
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    renode = None
    connection = None
    usb_helper = None
    usb_log = None
    try:
        wait_for_sidecar(physics, physics_log, physics_port)
        command = [
            sys.executable,
            str(root / 'Tools' / 'renode' / 'run.py'),
            'CubeOrangePlus',
            '--vehicle', 'arduplane',
            '--firmware', str(firmware),
            '--state-dir', str(state_dir),
            '--uart-port', str(uart_port),
            '--port', str(monitor_port),
            '--imu', 'icm42688_ext',
            '--imu', 'icm20948_ext',
            '--imu', 'icm20649',
            '--device', '{"device":"ublox-gps","port":"SERIAL2"}',
            '--device', '{"device":"ms4525-airspeed","port":"I2C1"}',
            '--exec', 'sysbus.physics Connect %u "quadplane" %.7f %.7f %.1f %.1f %u' % (
                physics_port, *CANBERRA, PHYSICS_RATE_HZ),
        ]
        add_launch_options(command, args)
        env = os.environ.copy()
        env['XDG_CONFIG_HOME'] = str(state_dir)
        env['TMPDIR'] = str(state_dir)
        with renode_log.open('w') as log:
            renode = subprocess.Popen(
                command,
                cwd=root,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        usb_helper, usb_log = start_usb_helper(args, root, output_dir)
        if args.interactive:
            wait_interactive(
                args, 'CubeOrangePlus QuadPlane', uart_port, renode,
                renode_log, physics, physics_log, usb_helper, usb_log)
            return
        deadline = time.monotonic() + args.timeout
        connection = common.connect_mavlink(uart_port, renode, renode_log, deadline)
        connection.setup_logfile(str(telemetry_log))
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            2,
            1,
        )
        home = wait_ready(
            connection, renode, renode_log, physics, physics_log, deadline)
        home_lat = home.lat * 1.0e-7
        home_lon = home.lon * 1.0e-7
        print(
            'CubeOrangePlus QuadPlane ready at %.7f %.7f' %
            (home_lat, home_lon),
            flush=True,
        )
        check_sensor_ids(
            connection, renode, renode_log, deadline, CUBEORANGEPLUS_SENSOR_IDS)
        common.upload_mission(
            connection, renode, renode_log,
            quadplane_mission_items(home_lat, home_lon),
        )
        connection.mav.mission_set_current_send(
            connection.target_system, connection.target_component, 1)
        common.set_mode(connection, renode, renode_log, 'AUTO')
        common.send_command(
            connection,
            renode,
            renode_log,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, common.FORCE_ARM_MAGIC),
        )
        print('CubeOrangePlus QuadPlane AUTO mission started', flush=True)
        result = wait_quadplane_mission(
            connection, renode, renode_log, physics, physics_log,
            deadline, home_lat, home_lon)
        check_sidecar(physics, physics_log)
        check_quadplane_result(result, home_lat, home_lon)
    finally:
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        if usb_helper is not None:
            common.stop_process_group(usb_helper)
        if renode is not None:
            common.stop_process_group(renode)
        common.stop_process_group(physics)


def main(argv=None):
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=('plane', 'copter', 'quadplane'))
    parser.add_argument('--renode', help='Renode executable')
    parser.add_argument('--data-cache', help='directory for downloaded Renode model data')
    parser.add_argument('--skip-build', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--timeout', type=common.positive_float, default=600)
    parser.add_argument(
        '--interactive', action='store_true',
        help='leave the booting board and physics model running for a GCS')
    parser.add_argument(
        '--usb', action='store_true',
        help='enable Renode USB/IP emulation')
    parser.add_argument(
        '--firmware', type=Path,
        help='custom APJ, BIN, HEX, or ELF firmware; skips the board build')
    parser.add_argument(
        '--gdb', action='store_true',
        help='open GDB in an xterm with ELF firmware halted at reset')
    args = parser.parse_args(argv)
    if args.firmware is not None:
        args.firmware = args.firmware.expanduser().resolve()
        if not args.firmware.is_file():
            parser.error('--firmware does not exist: %s' % args.firmware)
        firmware_is_elf = is_elf(args.firmware)
        if (not firmware_is_elf and
                args.firmware.suffix.lower() not in FIRMWARE_SUFFIXES):
            parser.error('--firmware must be an APJ, BIN, HEX, or ELF file')
        if args.gdb and not firmware_is_elf:
            parser.error('--gdb requires ELF firmware')
    default_renode = root / 'build' / 'renode' / 'renode'
    if args.renode is None and default_renode.is_file():
        args.renode = str(default_renode)
    local_data_cache = root / 'build' / 'renode-data-all-mcus-20260825'
    if args.data_cache is None and local_data_cache.is_dir():
        args.data_cache = str(local_data_cache)
    if args.output_dir is None:
        output_root = root / 'build' / 'renode-test'
        output_root.mkdir(parents=True, exist_ok=True)
        names = {
            'plane': 'MatekH743-plane',
            'copter': 'KakuteF4-copter',
            'quadplane': 'CubeOrangePlus-quadplane',
        }
        output_dir = output_root / (names[args.scenario] + '-' + time.strftime('%Y%m%d-%H%M%S'))
    else:
        output_dir = args.output_dir.resolve()
    if output_dir.exists():
        parser.error('--output-dir already exists: %s' % output_dir)
    output_dir.mkdir(parents=True)

    try:
        if args.scenario == 'plane':
            run_plane(args, root, output_dir)
        elif args.scenario == 'copter':
            run_copter(args, root, output_dir)
        else:
            run_quadplane(args, root, output_dir)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Renode physics flight failed: %s' % error, file=sys.stderr)
        print('test artifacts: %s' % output_dir, file=sys.stderr)
        return 1
    print('test artifacts: %s' % output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())

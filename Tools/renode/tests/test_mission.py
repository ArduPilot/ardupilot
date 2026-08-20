#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Fly and validate a short Copter mission on CubeOrange under Renode."""

import argparse
import contextlib
import math
import os
import signal
import socket
import subprocess
import sys
import time

from pathlib import Path

from pymavlink import DFReader
from pymavlink import mavutil

BOARD = 'CubeOrange'
VEHICLE = 'arducopter'
EARTH_RADIUS_METRES = 6378137.0
FORCE_ARM_MAGIC = 2989
WAYPOINT_OFFSETS = ((10, 0), (10, 10), (0, 10), (-10, 10))


def positive_float(value):
    result = float(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('must be positive')
    return result


def unused_tcp_port():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        return listener.getsockname()[1]


def log_tail(path, line_count=100):
    try:
        lines = path.read_text(errors='replace').splitlines()
    except OSError:
        return ''
    return '\n'.join(lines[-line_count:])


def stop_process_group(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def check_process(process, log_path):
    returncode = process.poll()
    if returncode is None:
        return
    detail = 'Renode stopped with status %u' % returncode
    tail = log_tail(log_path)
    if tail:
        detail += '\n--- Renode output (tail) ---\n' + tail
    raise RuntimeError(detail)


def recv_match(connection, process, log_path, message_type, deadline):
    while time.monotonic() < deadline:
        check_process(process, log_path)
        message = connection.recv_match(type=message_type, blocking=True, timeout=1)
        if message is not None:
            return message
    raise RuntimeError('timed out waiting for %s' % message_type)


def connect_mavlink(port, process, log_path, deadline):
    last_error = None
    while time.monotonic() < deadline:
        check_process(process, log_path)
        try:
            connection = mavutil.mavlink_connection(
                'tcp:127.0.0.1:%u' % port,
                source_system=255,
                source_component=190,
            )
            recv_match(connection, process, log_path, 'HEARTBEAT', deadline)
            return connection
        except OSError as error:
            last_error = error
            time.sleep(0.2)
    raise RuntimeError('timed out connecting to MAVLink: %s' % last_error)


def set_parameter(connection, process, log_path, name, value, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            name.encode('ascii'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        retry_deadline = min(deadline, time.monotonic() + 2)
        while time.monotonic() < retry_deadline:
            message = recv_match(
                connection, process, log_path, 'PARAM_VALUE', retry_deadline)
            param_id = message.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode(errors='replace')
            if param_id.rstrip('\x00') == name:
                if not math.isclose(message.param_value, value, abs_tol=0.01):
                    raise RuntimeError(
                        '%s was set to %s instead of %s' %
                        (name, message.param_value, value))
                return
    raise RuntimeError('timed out setting %s' % name)


def offset_location(lat, lon, north, east):
    lat_rad = math.radians(lat)
    return (
        lat + math.degrees(north / EARTH_RADIUS_METRES),
        lon + math.degrees(east / (EARTH_RADIUS_METRES * math.cos(lat_rad))),
    )


def wait_for_global_position(connection, process, log_path, deadline):
    while time.monotonic() < deadline:
        position = recv_match(
            connection, process, log_path, 'GLOBAL_POSITION_INT', deadline)
        if position.lat != 0 and position.lon != 0:
            return position
    raise RuntimeError('simulation did not provide a valid global position')


def mission_items(home_lat, home_lon):
    waypoint = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    items = [
        (frame, waypoint, 0, 1, 0, 0, 0, 0, home_lat, home_lon, 0),
        (frame, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
         0, 1, 0, 0, 0, float('nan'), home_lat, home_lon, 10),
    ]
    for north, east in WAYPOINT_OFFSETS:
        lat, lon = offset_location(home_lat, home_lon, north, east)
        items.append((frame, waypoint, 0, 1, 0, 2, 0, float('nan'), lat, lon, 10))
    items.append((frame, mavutil.mavlink.MAV_CMD_NAV_LAND,
                  0, 1, 0, 0, 0, float('nan'), home_lat, home_lon, 0))
    return items


def send_mission_item(connection, item, sequence, use_int):
    frame, command, current, autocontinue, p1, p2, p3, p4, lat, lon, altitude = item
    common = (
        connection.target_system,
        connection.target_component,
        sequence,
        frame,
        command,
        current,
        autocontinue,
        p1,
        p2,
        p3,
        p4,
    )
    if use_int:
        connection.mav.mission_item_int_send(
            *common, int(round(lat * 1e7)), int(round(lon * 1e7)), altitude)
    else:
        connection.mav.mission_item_send(*common, lat, lon, altitude)


def upload_mission(connection, process, log_path, items, timeout=30):
    deadline = time.monotonic() + timeout
    connection.mav.mission_count_send(
        connection.target_system, connection.target_component, len(items))
    while time.monotonic() < deadline:
        message = recv_match(
            connection,
            process,
            log_path,
            ['MISSION_REQUEST', 'MISSION_REQUEST_INT', 'MISSION_ACK'],
            deadline,
        )
        if message.get_type() == 'MISSION_ACK':
            if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError('mission upload failed with result %u' % message.type)
            return
        if message.seq >= len(items):
            raise RuntimeError('vehicle requested invalid mission item %u' % message.seq)
        send_mission_item(
            connection, items[message.seq], message.seq,
            message.get_type() == 'MISSION_REQUEST_INT')
    raise RuntimeError('timed out uploading the mission')


def set_mode(connection, process, log_path, mode_name, timeout=20):
    mode_mapping = connection.mode_mapping()
    if mode_mapping is None or mode_name not in mode_mapping:
        raise RuntimeError('firmware does not advertise %s mode' % mode_name)
    mode = mode_mapping[mode_name]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        connection.mav.set_mode_send(
            connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
        )
        heartbeat = recv_match(connection, process, log_path, 'HEARTBEAT', deadline)
        if heartbeat.custom_mode == mode:
            return
    raise RuntimeError('timed out changing to %s mode' % mode_name)


def send_command(connection, process, log_path, command, params=(), timeout=20):
    values = list(params) + [0] * (7 - len(params))
    deadline = time.monotonic() + timeout
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        command,
        0,
        *values,
    )
    while time.monotonic() < deadline:
        ack = recv_match(connection, process, log_path, 'COMMAND_ACK', deadline)
        if ack.command != command:
            continue
        if ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            continue
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(
                'command %u failed with result %u' % (command, ack.result))
        return
    raise RuntimeError('timed out waiting for command %u' % command)


def wait_for_flight(connection, process, log_path, deadline):
    required_waypoints = set(range(2, 2 + len(WAYPOINT_OFFSETS)))
    reached_waypoints = set()
    was_armed = False
    last_sequence = None
    max_altitude = 0
    while time.monotonic() < deadline:
        check_process(process, log_path)
        message = connection.recv_match(blocking=True, timeout=1)
        if message is None:
            continue
        message_type = message.get_type()
        if message_type == 'HEARTBEAT':
            armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            was_armed |= armed
            if was_armed and not armed:
                missing = required_waypoints - reached_waypoints
                if missing:
                    raise RuntimeError(
                        'vehicle landed before reaching mission items %s' %
                        ', '.join(str(item) for item in sorted(missing)))
                if max_altitude < 8:
                    raise RuntimeError(
                        'vehicle reached only %.1fm relative altitude' % max_altitude)
                return
        elif message_type == 'MISSION_CURRENT' and message.seq != last_sequence:
            last_sequence = message.seq
            print('mission advanced to item %u' % message.seq, flush=True)
        elif message_type == 'MISSION_ITEM_REACHED':
            reached_waypoints.add(message.seq)
        elif message_type == 'GLOBAL_POSITION_INT':
            max_altitude = max(max_altitude, message.relative_alt * 0.001)
        elif message_type == 'STATUSTEXT':
            print('vehicle: %s' % message.text, flush=True)
    raise RuntimeError('mission did not land before the test timeout')


def latest_log_entry(connection, process, log_path, timeout=20):
    deadline = time.monotonic() + timeout
    connection.mav.log_request_list_send(
        connection.target_system, connection.target_component, 0, 0xffff)
    latest = None
    while time.monotonic() < deadline:
        entry = recv_match(connection, process, log_path, 'LOG_ENTRY', deadline)
        if entry.num_logs == 0:
            raise RuntimeError('vehicle reported no DataFlash logs')
        if latest is None or entry.id > latest.id:
            latest = entry
        if entry.id == entry.last_log_num:
            if entry.size == 0:
                raise RuntimeError('latest DataFlash log is empty')
            return entry
    raise RuntimeError('timed out listing DataFlash logs')


def download_log(connection, process, process_log, output, timeout=180):
    entry = latest_log_entry(connection, process, process_log)
    print('downloading log %u (%u bytes) over MAVLink' % (entry.id, entry.size),
          flush=True)
    data = bytearray()
    deadline = time.monotonic() + timeout
    retry_deadline = 0
    while len(data) < entry.size:
        if time.monotonic() >= retry_deadline:
            connection.mav.log_request_data_send(
                connection.target_system,
                connection.target_component,
                entry.id,
                len(data),
                entry.size - len(data),
            )
            retry_deadline = time.monotonic() + 3
        message = connection.recv_match(type='LOG_DATA', blocking=True, timeout=1)
        check_process(process, process_log)
        if message is None:
            if time.monotonic() >= deadline:
                raise RuntimeError('timed out downloading the DataFlash log')
            continue
        if message.id != entry.id or message.ofs < len(data):
            continue
        if message.ofs != len(data):
            retry_deadline = 0
            continue
        if message.count == 0:
            raise RuntimeError('vehicle returned an empty LOG_DATA packet')
        data.extend(message.data[:message.count])
        retry_deadline = time.monotonic() + 3
    connection.mav.log_request_end_send(
        connection.target_system, connection.target_component)
    output.write_bytes(data[:entry.size])
    return output


def distance_metres(lat1, lon1, lat2, lon2):
    mean_latitude = math.radians((lat1 + lat2) * 0.5)
    north = math.radians(lat2 - lat1) * EARTH_RADIUS_METRES
    east = math.radians(lon2 - lon1) * EARTH_RADIUS_METRES * math.cos(mean_latitude)
    return math.hypot(north, east)


def check_log(path, home_lat, home_lon):
    if path.stat().st_size < 50000:
        raise RuntimeError('downloaded log is unexpectedly small (%u bytes)' % path.stat().st_size)
    reader = DFReader.DFReader_binary(str(path), zero_time_base=True)
    position_count = 0
    max_altitude = 0
    final_altitude = None
    max_distance = 0
    saw_auto = False
    max_drops = 0
    try:
        while True:
            message = reader.recv_msg()
            if message is None:
                break
            message_type = message.get_type()
            if message_type == 'POS':
                position_count += 1
                max_altitude = max(max_altitude, message.RelHomeAlt)
                final_altitude = message.RelHomeAlt
                max_distance = max(
                    max_distance,
                    distance_metres(home_lat, home_lon, message.Lat, message.Lng),
                )
            elif message_type == 'MODE':
                saw_auto |= message.Mode == 3 or message.ModeNum == 3
            elif message_type == 'DSF':
                max_drops = max(max_drops, message.Dp)
    finally:
        reader.close()
    if position_count < 50:
        raise RuntimeError('log has only %u POS records' % position_count)
    if not saw_auto:
        raise RuntimeError('log does not contain AUTO mode')
    if max_altitude < 8:
        raise RuntimeError('log records only %.1fm maximum altitude' % max_altitude)
    if max_distance < 8:
        raise RuntimeError('log records only %.1fm maximum distance from home' % max_distance)
    if final_altitude is None or abs(final_altitude) > 1:
        raise RuntimeError('log ends at %.1fm relative altitude' % final_altitude)
    if max_drops != 0:
        raise RuntimeError('log records %u dropped messages' % max_drops)
    print(
        'log check passed: %u POS records, %.1fm high, %.1fm from home, no drops' %
        (position_count, max_altitude, max_distance),
        flush=True,
    )


def build_firmware(root):
    command = [
        sys.executable,
        str(root / 'Tools' / 'scripts' / 'sitl-on-hardware' / 'sitl-on-hw.py'),
        '--board', BOARD,
        '--vehicle', 'copter',
        '--frame', 'quad',
        '--defaults', str(root / 'Tools' / 'renode' / 'tests' / 'CubeOrange.parm'),
    ]
    print('building CubeOrange SITL-on-hardware Copter firmware', flush=True)
    subprocess.run(command, cwd=root, check=True)


def run_test(args, root, output_dir):
    if not args.skip_build:
        build_firmware(root)
    firmware = (args.firmware or root / 'build' / BOARD / 'bin' / VEHICLE).resolve()
    if not firmware.is_file():
        raise RuntimeError('firmware does not exist: %s' % firmware)

    state_dir = output_dir / 'state'
    state_dir.mkdir()
    process_log = output_dir / 'renode.log'
    flight_log = output_dir / 'flight.BIN'
    uart_port = unused_tcp_port()
    monitor_port = unused_tcp_port()
    while monitor_port == uart_port:
        monitor_port = unused_tcp_port()
    command = [
        sys.executable,
        str(root / 'Tools' / 'renode' / 'run.py'),
        BOARD,
        '--vehicle', VEHICLE,
        '--elf', str(firmware),
        '--state-dir', str(state_dir),
        '--uart-port', str(uart_port),
        '--port', str(monitor_port),
        '--unthrottled',
    ]
    if args.renode:
        command.extend(['--renode', args.renode])
    env = os.environ.copy()
    env['XDG_CONFIG_HOME'] = str(state_dir)
    env['TMPDIR'] = str(state_dir)
    with process_log.open('w') as log:
        process = subprocess.Popen(
            command,
            cwd=root,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    connection = None
    try:
        deadline = time.monotonic() + args.timeout
        connection = connect_mavlink(uart_port, process, process_log, deadline)
        print('connected to CubeOrange over MAVLink', flush=True)
        connection.mav.request_data_stream_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1,
        )
        set_parameter(connection, process, process_log, 'AUTO_OPTIONS', 3)
        position = wait_for_global_position(
            connection, process, process_log, deadline)
        home_lat = position.lat * 1e-7
        home_lon = position.lon * 1e-7
        items = mission_items(home_lat, home_lon)
        upload_mission(connection, process, process_log, items)
        connection.mav.mission_set_current_send(
            connection.target_system, connection.target_component, 1)
        set_mode(connection, process, process_log, 'AUTO')
        send_command(
            connection,
            process,
            process_log,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            (1, FORCE_ARM_MAGIC),
        )
        print('mission started', flush=True)
        wait_for_flight(connection, process, process_log, deadline)
        print('mission landed and disarmed', flush=True)
        download_log(connection, process, process_log, flight_log)
        check_log(flight_log, home_lat, home_lon)
    except (OSError, RuntimeError, ValueError) as error:
        tail = log_tail(process_log)
        detail = str(error)
        if tail:
            detail += '\n--- Renode output (tail) ---\n' + tail
        raise RuntimeError(detail) from error
    finally:
        if connection is not None:
            with contextlib.suppress(OSError):
                connection.close()
        stop_process_group(process)
    print('test artifacts: %s' % output_dir)


def main():
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--renode', help='Renode executable (defaults to build/renode/renode when present)')
    parser.add_argument('--firmware', type=Path,
                        help='existing arducopter ELF (implies --skip-build)')
    parser.add_argument('--skip-build', action='store_true',
                        help='use build/CubeOrange/bin/arducopter without rebuilding')
    parser.add_argument('--output-dir', type=Path,
                        help='new directory for the downloaded log and Renode state')
    parser.add_argument('--timeout', type=positive_float, default=300,
                        help='overall boot and flight timeout in seconds (default: 300)')
    args = parser.parse_args()
    default_renode = root / 'build' / 'renode' / 'renode'
    if args.renode is None and default_renode.is_file():
        args.renode = str(default_renode)

    if args.firmware is not None:
        args.skip_build = True

    if args.output_dir is None:
        output_root = root / 'build' / 'renode-test'
        output_root.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d-%H%M%S')
        output_dir = output_root / ('CubeOrange-mission-' + stamp)
    else:
        output_dir = args.output_dir.resolve()
    if output_dir.exists():
        parser.error('--output-dir already exists: %s' % output_dir)
    output_dir.mkdir(parents=True)

    try:
        run_test(args, root, output_dir)
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print('Renode mission test failed: %s' % error, file=sys.stderr)
        print('test artifacts: %s' % output_dir, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())

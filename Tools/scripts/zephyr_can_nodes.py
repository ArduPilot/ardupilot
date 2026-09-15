#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
'''
List the DroneCAN nodes reachable through the mr_vmu_rt1176, with details.

No arguments - everything is hardcoded for this bench: finds the board's
USB CDC port by its /dev/serial/by-id/ ArduPilot name, then monitors BOTH
physical CAN buses (15s each) using the dronecan python package's
MAVLink-CAN transport (ArduPilot CAN_FORWARD frame forwarding - functional
on AP_HAL_Zephyr since the 2026-08-11 ISR-safe RX fix). Reports node IDs,
names, software versions, unique IDs and health, then fetches
@SYS/can0_stats.txt / can1_stats.txt (AP_HAL_Zephyr CANIface::get_stats():
frame counters + controller bus state + TEC/REC error counters).

    python3 Tools/scripts/zephyr_can_nodes.py
'''

import glob
import os
import sys
import time

# hardcoded bench settings
BAUD = 115200
WAIT_PER_BUS_S = 15.0
BUSES = (1, 2)          # both physical CAN buses, 1-based
OUR_NODE_ID = 126       # monitor's own DroneCAN node id (kept clear of peripherals)

HEALTH = {0: 'OK', 1: 'WARNING', 2: 'ERROR', 3: 'CRITICAL'}


def find_port():
    ports = sorted(glob.glob('/dev/serial/by-id/usb-ArduPilot_*'))
    app = [p for p in ports if '-BL_' not in p]
    ports = app or ports
    if not ports:
        print('no ArduPilot USB CDC port found under /dev/serial/by-id/')
        sys.exit(1)
    return ports[0]


def monitor_bus(port, bus):
    '''Return [(node_id, name, health, uptime, swver, uid)] via dronecan mavcan.'''
    import dronecan

    from dronecan.app import node_monitor
    from dronecan.driver import mavcan as dc_mavcan

    # dronecan's MAVCAN.__del__ is broken at teardown: it join()s its worker
    # from the wrong side of the fork ("can only join a child process") and
    # put_nowait()s into a full exit queue (queue.Full), spraying "Exception
    # ignored" tracebacks over otherwise-good output. Neutralise it BEFORE
    # make_node forks the worker (the child inherits the patched class), and
    # do the real cleanup ourselves via the explicit close() below.
    dc_mavcan.MAVCAN.__del__ = lambda self: None

    node = dronecan.make_node('mavcan:' + port, node_id=OUR_NODE_ID,
                              bitrate=1000000, bus_number=bus)
    monitor = node_monitor.NodeMonitor(node)
    deadline = time.time() + WAIT_PER_BUS_S
    while time.time() < deadline:
        try:
            node.spin(0.2)
        except dronecan.transport.TransferError:
            pass  # tolerate the odd malformed transfer mid-monitor

    rows = []
    for entry in monitor.find_all(lambda e: True):
        st = entry.status
        name = '?'
        swver = '?'
        uid = '?'
        if entry.info is not None:
            raw = entry.info.name
            name = raw.decode() if isinstance(raw, bytes) else str(raw)
            swver = f'{entry.info.software_version.major}.{entry.info.software_version.minor}'
            uid = bytes(entry.info.hardware_version.unique_id).hex()
        health = HEALTH.get(st.health, str(st.health))
        rows.append((entry.node_id, name, health, st.uptime_sec, swver, uid))
    try:
        node.close()
    except Exception:  # noqa: BLE001
        pass  # upstream close() can raise queue.Full; worker dies with us anyway
    return rows


def fetch_stats(port):
    from pymavlink import mavftp
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(port, baud=BAUD)
    if m.wait_heartbeat(timeout=20) is None:
        print('no heartbeat for stats fetch')
        return
    ftp = mavftp.MAVFTP(m, target_system=m.target_system, target_component=m.target_component)
    for fname in ('can0_stats.txt', 'can1_stats.txt'):
        out = f'/tmp/{fname}'
        try:
            os.remove(out)
        except FileNotFoundError:
            pass
        ret = ftp.cmd_get([f'@SYS/{fname}', out])
        ftp.process_ftp_reply(ret.operation_name, timeout=10)
        if os.path.exists(out) and os.path.getsize(out) > 0:
            print(f'\n=== @SYS/{fname} ===')
            with open(out) as f:
                print(f.read().rstrip())
    m.close()


def main():
    port = find_port()
    all_rows = {}
    for bus in BUSES:
        print(f'monitoring DroneCAN on bus {bus} via {port} for {WAIT_PER_BUS_S:.0f}s ...')
        try:
            rows = monitor_bus(port, bus)
        except Exception as e:  # noqa: BLE001
            print(f'  bus {bus} monitor failed: {e}')
            continue
        for row in rows:
            all_rows.setdefault(row[0], (bus, row))
        if not rows:
            print(f'  no nodes seen on bus {bus}')

    if all_rows:
        print(f'\n{len(all_rows)} DroneCAN node(s) total:\n')
        print(f'{"bus":>3} {"node":>4}  {"name":<34} {"health":<8} {"uptime":>8}  {"sw":<5} unique-id')
        for nid in sorted(all_rows):
            bus, (nid_, name, health, uptime, swver, uid) = all_rows[nid]
            print(f'{bus:>3} {nid_:>4}  {name:<34} {health:<8} {uptime:>7}s  {swver:<5} {uid}')
    else:
        print('\nno DroneCAN nodes seen on either bus.')
        print('  - CAN_P1_DRIVER/CAN_P2_DRIVER set and rebooted since?')
        print('  - peripherals powered and terminated?')

    fetch_stats(port)


if __name__ == '__main__':
    main()

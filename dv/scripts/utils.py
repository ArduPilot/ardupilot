from pymavlink import mavutil

import logging, coloredlogs

coloredlogs.install(
    level='DEBUG',
    # fmt="%(asctime)s %(hostname)s %(name)s[%(process)d] %(levelname)s %(message)s"
    fmt="%(name)s[%(process)d] %(levelname)s %(message)s",
    level_styles={
        'info': {
            'color': 'green',
            'bold': True,
        },
        'debug': {
            'color': 'white'
        },
        'warn': {
            'color': 'yellow',
        },
        'error': {
            'color': 'red',
            'bold': True,
        },
    },
)
logger = logging.getLogger(__name__)


def get_tcp_master(connection='tcp:127.0.0.1:5760'):
    logger.info(f"Probing autopilot at {connection}")
    try:
        some_master = mavutil.mavlink_connection(connection)
    except Exception as e:
        logger.error(type(e))
        logger.error(e)
        logger.error(e.args)
        return None

    if some_master.wait_heartbeat(timeout=10) is None:
        return None

    return some_master


def get_serial_master(lport):
    for baud_flightstack in [921600, 500000, 115200, 57600]:
        logger.info(f"Probing autopilot at {lport}:{baud_flightstack}")
        some_master = mavutil.mavlink_connection(lport, baud=baud_flightstack)

        res = None

        for serial_tries in range(10):
            try:
                res = some_master.wait_heartbeat(timeout=4)
                break
            except serial.serialutil.SerialException:
                logger.warning(
                    f"The serial might be occupied by the mavlink-router, waiting untill it is free: {serial_tries}")
                time.sleep(3)

        if res is not None:
            return some_master

    return None

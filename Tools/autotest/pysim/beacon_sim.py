'''
MAVLink RANGING_BEACON simulator for AP_Beacon_Sine (BCN_TYPE=4).

AP_FLAKE8_CLEAN
'''

import math
import random
import struct

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message as _MAVLink_message

_MAV_COMP_ID_USER66 = 90        # AP_Beacon_Sine filter requires this component ID
_BEACON_SIM_SYSID = 200         # source system ID used in outgoing packets


# RANGING_BEACON (msg ID 513, crc_extra 99) is a custom ArduPilot extension that is not
# yet in the released pymavlink package, so the framing class is defined manually here.
class MAVLink_ranging_beacon_message(_MAVLink_message):
    id       = 513
    msgname  = 'RANGING_BEACON'
    fieldnames = [
        'time_usec', 'target_system', 'target_component', 'beacon_id',
        'range', 'lat', 'lon', 'alt', 'alt_type',
        'hacc_est', 'vacc_est', 'carrier_freq', 'range_accuracy',
        'sequence', 'status',
    ]
    ordered_fieldnames = [
        'time_usec', 'range', 'lat', 'lon', 'alt',
        'hacc_est', 'vacc_est', 'range_accuracy',
        'beacon_id', 'carrier_freq',
        'target_system', 'target_component', 'alt_type', 'sequence', 'status',
    ]
    fieldtypes = [
        'uint64_t', 'uint8_t', 'uint8_t', 'uint16_t',
        'uint32_t', 'int32_t', 'int32_t', 'float', 'uint8_t',
        'uint32_t', 'uint32_t', 'uint16_t', 'uint32_t',
        'uint8_t', 'uint8_t',
    ]
    fielddisplays_by_name: dict = {}
    fieldenums_by_name:    dict = {}
    fieldunits_by_name:    dict = {}
    native_format = bytearray(b'<QIiifIIIHHBBBBB')
    orders = [0, 10, 11, 8, 1, 2, 3, 4, 12, 5, 6, 9, 7, 13, 14]
    lengths       = [1] * 15
    array_lengths = [0] * 15
    crc_extra = 99
    unpacker = struct.Struct('<QIiifIIIHHBBBBB')
    instance_field  = None
    instance_offset = -1

    def __init__(self, time_usec, target_system, target_component, beacon_id,
                 range, lat, lon, alt, alt_type,
                 hacc_est, vacc_est, carrier_freq, range_accuracy,
                 sequence, status):
        _MAVLink_message.__init__(self, self.id, self.msgname)
        self._fieldnames      = self.fieldnames
        self._instance_field  = self.instance_field
        self._instance_offset = self.instance_offset
        self.time_usec        = time_usec
        self.target_system    = target_system
        self.target_component = target_component
        self.beacon_id        = beacon_id
        self.range            = range
        self.lat              = lat
        self.lon              = lon
        self.alt              = alt
        self.alt_type         = alt_type
        self.hacc_est         = hacc_est
        self.vacc_est         = vacc_est
        self.carrier_freq     = carrier_freq
        self.range_accuracy   = range_accuracy
        self.sequence         = sequence
        self.status           = status

    def pack(self, mav, force_mavlink1=False):
        return self._pack(
            mav, self.crc_extra,
            self.unpacker.pack(
                self.time_usec, self.range, self.lat, self.lon, self.alt,
                self.hacc_est, self.vacc_est, self.range_accuracy,
                self.beacon_id, self.carrier_freq,
                self.target_system, self.target_component,
                self.alt_type, self.sequence, self.status,
            ),
            force_mavlink1=force_mavlink1,
        )


class SITLBeaconSimulator:
    '''Simulate RANGING_BEACON MAVLink messages for AP_Beacon_Sine (BCN_TYPE=4).

    Install as a message hook via suite.install_message_hook_context(sim).
    Fires on every received MAVLink message but uses sim-time rate limiting so
    beacons are emitted at a regular interval independent of which messages arrive.

    Vehicle position is read from pymavlink's message cache (SIMSTATE for lat/lng,
    SIM_STATE for MSL altitude) — safe to use from within a hook without re-entering
    recv_match/parse_char.

    suite:         TestSuite instance (provides get_distance(), get_sim_time_cached(),
                   and mav.write())
    beacons:       list of up to 4 (lat_deg, lon_deg, alt_m_MSL) tuples
    noise_sigma_m: 1-sigma Gaussian range noise added to each measurement (metres)
    rate_hz:       full cycle rate in simulation Hz — total messages per second is
                   rate_hz * len(beacons), evenly spaced in round-robin order.
                   Example: 4 beacons at 2 Hz → 8 messages/s, one every 125 ms.
    '''

    def __init__(self, suite, beacons, noise_sigma_m=0.5, rate_hz=10.0):
        if len(beacons) > 4:
            raise ValueError("AP_Beacon supports at most 4 beacons (AP_BEACON_MAX_BEACONS=4)")
        self._suite = suite
        self._beacons = list(beacons)
        self._noise_sigma_m = noise_sigma_m
        self._rate_hz = rate_hz
        self._seq = 0
        self._next_beacon = 0          # round-robin index
        self._next_send_time = None    # sim-time of next scheduled send (set on first call)

    def __call__(self, mav, msg):
        '''Message hook entry point — called for every received MAVLink message.'''
        sim_now = self._suite.get_sim_time_cached()

        # Initialise schedule on the very first call
        if self._next_send_time is None:
            self._next_send_time = sim_now
            return

        if sim_now < self._next_send_time:
            return

        # Read physical SITL truth from pymavlink's message cache.
        # mav.messages is populated before hooks are called so this is safe to
        # use from within a hook — it never re-enters recv_match/parse_char.
        simstate = self._suite.mav.messages.get('SIMSTATE')
        if simstate is None:
            return
        veh_lat = simstate.lat * 1e-7
        veh_lng = simstate.lng * 1e-7

        # Baro-derived EKF alt closely tracks physical truth for ranging purposes.
        gpi = self._suite.mav.messages.get('GLOBAL_POSITION_INT')
        veh_alt = gpi.alt * 1e-3 if gpi is not None else 0.0  # mm → m MSL
        veh_loc = mavutil.location(veh_lat, veh_lng, veh_alt, 0)

        # Send the next beacon in round-robin order
        i = self._next_beacon
        blat, blon, balt = self._beacons[i]

        bcn_loc = mavutil.location(blat, blon, balt, 0)
        h = self._suite.get_distance(veh_loc, bcn_loc)   # horizontal (m)
        v = veh_alt - balt                                 # vertical (m)
        range_m = math.sqrt(h * h + v * v)
        if self._noise_sigma_m > 0:
            range_m += random.gauss(0, self._noise_sigma_m)
        range_m = max(0.1, range_m)

        self._seq = (self._seq + 1) & 0xFF
        self._send(i, range_m, blat, blon, balt)

        # Advance round-robin and schedule the next send
        self._next_beacon = (i + 1) % len(self._beacons)
        self._next_send_time += 1.0 / (self._rate_hz * len(self._beacons))

    def _send(self, beacon_id, range_m, blat, blon, balt):
        msg = MAVLink_ranging_beacon_message(
            0,                                    # time_usec
            1,                                    # target_system
            _MAV_COMP_ID_USER66,                  # target_component
            beacon_id,                            # beacon_id
            min(int(range_m * 1000), 0xFFFFFFFF), # range (mm), clamped to uint32
            int(blat * 1e7),                      # lat (degE7)
            int(blon * 1e7),                      # lon (degE7)
            float(balt),                          # alt (m MSL)
            0,                                    # alt_type = WGS84
            0,                                    # hacc_est
            0,                                    # vacc_est
            0,                                    # carrier_freq
            int(self._noise_sigma_m * 1000),      # range_accuracy (mm)
            self._seq,                            # sequence
            0,                                    # status
        )

        # Pack using the MAVLink protocol object so sysid/compid/seq are set
        # correctly in the frame header. Temporarily override src fields so the
        # packet appears to come from the beacon peripheral (sysid=200, comp=90).
        mav_proto = self._suite.mav.mav
        old_sys  = mav_proto.srcSystem
        old_comp = mav_proto.srcComponent
        try:
            mav_proto.srcSystem    = _BEACON_SIM_SYSID
            mav_proto.srcComponent = _MAV_COMP_ID_USER66
            self._suite.mav.write(msg.pack(mav_proto))
        finally:
            mav_proto.srcSystem    = old_sys
            mav_proto.srcComponent = old_comp

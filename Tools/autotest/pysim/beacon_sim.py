'''
MAVLink RANGING_BEACON simulator for AP_Beacon_MAVlink (BCN_TYPE=4).

AP_FLAKE8_CLEAN
'''

import math
import random
import struct

from pymavlink import mavutil
from pymavlink.dialects.v20.ardupilotmega import MAVLink_message as _MAVLink_message

_BEACON_SIM_SYSID = 200         # source system ID used in outgoing packets
_BEACON_SIM_COMPID = 200        # source component ID used in outgoing packets


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
    '''Simulate RANGING_BEACON MAVLink messages for AP_Beacon_MAVlink (BCN_TYPE=4).

    Install as a message hook via suite.install_message_hook_context(sim).
    Fires on every received MAVLink message but uses sim-time rate limiting so
    beacons are emitted at a regular interval independent of which messages arrive.

    Vehicle position is read from pymavlink's message cache (SIMSTATE for lat/lng,
    SIM_STATE for MSL altitude) — safe to use from within a hook without re-entering
    recv_match/parse_char.

    suite:         TestSuite instance (provides get_distance(), get_sim_time_cached(),
                   and mav.write())
    beacons:       list of (lat_deg, lon_deg, alt_m_MSL) tuples, one per physical beacon;
                   may be longer than num_slots
    noise_sigma_m: 1-sigma Gaussian range noise added to each measurement (metres)
    range_bias_m:  optional list with a constant range error (metres) per beacon,
                   e.g. to model multipath or beacon position errors
    alt_offset_m:  offset added to the reported beacon altitude (metres); ranges
                   still use the true altitude, so this models a beacon system
                   whose vertical datum differs from the EKF one
    rate_hz:       full cycle rate in simulation Hz — total messages per second is
                   rate_hz * active slots, evenly spaced in round-robin order.
                   Example: 4 slots at 2 Hz → 8 messages/s, one every 125 ms.
    num_slots:     number of beacon IDs (slots) reported to the autopilot. Must not
                   exceed AP_BEACON_MAX_BEACONS of the firmware, which drops higher IDs.
    rotation_s:    when there are more beacons than slots, every rotation_s seconds
                   a new random set of beacons is picked and assigned to the slots,
                   each beacon to at most one slot.  With no more beacons than slots
                   each beacon keeps its own slot.
    seed:          seed for the beacon selection and range noise (constant by default so tests are repeatable)
    '''

    def __init__(self, suite, beacons, noise_sigma_m=0.5, rate_hz=10.0, range_bias_m=None, alt_offset_m=0.0,
                 num_slots=5, rotation_s=10.0, seed=1):
        if num_slots < 1:
            raise ValueError("num_slots must be at least 1")
        self._suite = suite
        self._beacons = list(beacons)
        self._num_slots = num_slots
        self._rotation_s = rotation_s
        self._random = random.Random(seed)
        self._slots = list(range(min(num_slots, len(self._beacons))))   # slot -> beacon index
        self._next_rotation_time = None
        self._noise_sigma_m = noise_sigma_m
        self._rate_hz = rate_hz
        self._range_bias_m = list(range_bias_m) if range_bias_m is not None else [0.0] * len(self._beacons)
        self._alt_offset_m = alt_offset_m
        self._seq = 0
        self._next_beacon = 0          # round-robin index
        self._next_send_time = None    # sim-time of next scheduled send (set on first call)

    def set_beacon(self, index, beacon, range_bias_m=None):
        '''Move physical beacon index to beacon (lat_deg, lon_deg, alt_m_MSL),
        optionally changing its constant range error.'''
        self._beacons[index] = beacon
        if range_bias_m is not None:
            self._range_bias_m[index] = range_bias_m

    def __call__(self, mav, msg):
        '''Message hook entry point — called for every received MAVLink message.'''
        sim_now = self._suite.get_sim_time_cached()

        # Initialise schedule on the very first call
        if self._next_send_time is None:
            self._next_send_time = sim_now
            self._next_rotation_time = sim_now
            return

        if sim_now < self._next_send_time:
            return

        if len(self._beacons) > self._num_slots and sim_now >= self._next_rotation_time:
            self._slots = self._random.sample(range(len(self._beacons)), self._num_slots)
            self._next_rotation_time = max(self._next_rotation_time + self._rotation_s, sim_now)

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

        # Send the next slot in round-robin order
        slot = self._next_beacon
        i = self._slots[slot]
        blat, blon, balt = self._beacons[i]

        bcn_loc = mavutil.location(blat, blon, balt, 0)
        h = self._suite.get_distance(veh_loc, bcn_loc)   # horizontal (m)
        v = veh_alt - balt                                 # vertical (m)
        range_m = math.sqrt(h * h + v * v) + self._range_bias_m[i]
        if self._noise_sigma_m > 0:
            range_m += self._random.gauss(0, self._noise_sigma_m)
        range_m = max(0.1, range_m)

        self._seq = (self._seq + 1) & 0xFF
        self._send(slot, range_m, blat, blon, balt + self._alt_offset_m)

        # Advance round-robin and schedule the next send
        self._next_beacon = (slot + 1) % len(self._slots)
        self._next_send_time += 1.0 / (self._rate_hz * len(self._slots))

    def _send(self, beacon_id, range_m, blat, blon, balt):
        msg = MAVLink_ranging_beacon_message(
            0,                                    # time_usec
            1,                                    # target_system
            1,                                    # target_component
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
            mav_proto.srcComponent = _BEACON_SIM_COMPID
            self._suite.mav.write(msg.pack(mav_proto))
        finally:
            mav_proto.srcSystem    = old_sys
            mav_proto.srcComponent = old_comp

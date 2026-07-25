'''
module for loading/saving waypoints

Copyright the ArduPilot Project
Released under GNU LGPL version 3 or later
'''

import time, copy
import logging
import re

from . import mavutil
try:
    from google.protobuf import text_format
    import mission_pb2
    HAVE_PROTOBUF = True
except ImportError:
    HAVE_PROTOBUF = False


class MAVWPError(Exception):
    '''MAVLink WP error class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg


class MissionItemProtocol(object):
    '''Base class for transferring items based on the MISSION_ITEM protocol'''
    def __init__(self, target_system=0, target_component=0):
        self.wpoints = []
        self.target_system = target_system
        self.target_component = target_component
        self.last_change = 0
        self.colour_for_polygon = {
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION : (255,0,0),
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION : (0,255,0),
            mavutil.mavlink.MAV_CMD_DO_LAND_START: (255, 127, 0),
            mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START: (127, 255, 0),
        }

    def count(self):
        '''return number of waypoints'''
        return len(self.wpoints)

    def wp(self, i):
        '''alias for backwards compatability'''
        return self.item(i)

    def item(self, i):
        '''return an item'''
        try:
            the_wp = self.wpoints[i]
        except:
            the_wp = None

        return the_wp

    def add(self, w, comment=''):
        '''add a waypoint'''
        if type(w) == list:
            w = copy.deepcopy(w)
            n = self.count()
            for p in w:
                p.seq = n
                n += 1
            self.wpoints.extend(w)
        else:
            w = copy.copy(w)
            if comment:
                w.comment = comment
            w.seq = self.count()
            self.wpoints.append(w)
        self.last_change = time.time()

    def insert(self, idx, w, comment=''):
        '''insert a waypoint'''
        if idx >= self.count():
            self.add(w, comment)
            return
        if idx < 0:
            return
        w = copy.copy(w)
        if comment:
            w.comment = comment
        w.seq = idx
        self.wpoints.insert(idx, w)
        self.last_change = time.time()
        self.reindex()

    def reindex(self):
        '''reindex waypoints'''
        for i in range(self.count()):
            w = self.wpoints[i]
            w.seq = i
        self.last_change = time.time()

    def set(self, w, idx):
        '''set a waypoint'''
        w.seq = idx
        if w.seq == self.count():
            return self.add(w)
        if self.count() <= idx:
            raise MAVWPError('adding waypoint at idx=%u past end of list (count=%u)' % (idx, self.count()))
        self.wpoints[idx] = w
        self.last_change = time.time()

    def remove(self, w):
        '''remove a waypoint'''
        if isinstance(w, list):
            for point in w:
                self.wpoints.remove(point)
        else:
            self.wpoints.remove(w)
        self.last_change = time.time()
        self.reindex()

    def clear(self):
        '''clear waypoint list'''
        self.wpoints = []
        self.last_change = time.time()

    def _read_waypoints_v110(self, file):
        '''read a version 110 waypoint'''
        comment = ''
        for line in file:
            if line.startswith('#'):
                comment = line[1:].lstrip()
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 12:
                raise MAVWPError("invalid waypoint line with %u values" % len(a))
            args = [
                self.target_system,
                self.target_component,
                int(a[0]),    # seq
                int(a[2]),    # frame
                int(a[3]),    # command
                int(a[1]),    # current
                int(a[11]),   # autocontinue
                float(a[4]),  # param1,
                float(a[5]),  # param2,
                float(a[6]),  # param3
                float(a[7]),  # param4
                float(a[8]),  # x (latitude)
                float(a[9]),  # y (longitude)
                float(a[10]), # z (altitude)
            ]
            if mavutil.mavlink20():
                fn = mavutil.mavlink.MAVLink_mission_item_message
                args.append(self.mav_mission_type())
            elif mavutil.mavlink10():
                fn = mavutil.mavlink.MAVLink_mission_item_message
                if self.mav_mission_type() != mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                    raise ValueError("Not using mavlink2")
            else:
                fn = mavutil.mavlink.MAVLink_waypoint_message
            w = fn(*args)
            if w.command == 0 and w.seq == 0 and self.count() == 0:
                # special handling for Mission Planner created home wp
                w.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
            self.add(w, comment)
            comment = ''

    def _read_waypoints_pb_110(self, file):
        if not HAVE_PROTOBUF:
            raise MAVWPError(
                'Cannot read mission file in protobuf format without protobuf '
                'library. Try "easy_install protobuf".')
        explicit_seq = False
        warned_seq = False
        mission = mission_pb2.Mission()
        text_format.Merge(file.read(), mission)
        defaults = mission_pb2.Waypoint()
        # Set defaults (may be overridden in file).
        defaults.current = False
        defaults.autocontinue = True
        defaults.param1 = 0.0
        defaults.param2 = 0.0
        defaults.param3 = 0.0
        defaults.param4 = 0.0
        defaults.x = 0.0
        defaults.y = 0.0
        defaults.z = 0.0
        # Use defaults specified in mission file, if there are any.
        if mission.defaults:
            defaults.MergeFrom(mission.defaults)
        for seq, waypoint in enumerate(mission.waypoint):
            # Consecutive sequence numbers are automatically assigned
            # UNLESS the mission file specifies sequence numbers of
            # its own.
            if waypoint.seq:
                explicit_seq = True
            else:
                if explicit_seq and not warned_seq:
                    logging.warn(
                            'Waypoint file %s: mixes explicit and implicit '
                            'sequence numbers' % (file,))
                    warned_seq = True
            # The first command has current=True, the rest have current=False.
            if seq > 0:
                current = defaults.current
            else:
                current = True
            w = mavutil.mavlink.MAVLink_mission_item_message(
                self.target_system, self.target_component,
                   waypoint.seq or seq,
                   waypoint.frame,
                   waypoint.command,
                   waypoint.current or current,
                   waypoint.autocontinue or defaults.autocontinue,
                   waypoint.param1 or defaults.param1,
                   waypoint.param2 or defaults.param2,
                   waypoint.param3 or defaults.param3,
                   waypoint.param4 or defaults.param4,
                   waypoint.x or defaults.x,
                   waypoint.y or defaults.y,
                   waypoint.z or defaults.z)
            self.add(w)

    def load(self, filename):
        '''load waypoints from a file.
        returns number of waypoints loaded'''
        f = open(filename, mode='r')
        version_line = f.readline().strip()
        if version_line == "QGC WPL 100":
            readfn = self._read_waypoints_v100
        elif version_line == "QGC WPL 110":
            readfn = self._read_waypoints_v110
        elif version_line == "QGC WPL PB 110":
            readfn = self._read_waypoints_pb_110
        else:
            f.close()
            raise MAVWPError("Unsupported waypoint format '%s'" % version_line)

        self.clear()
        readfn(f)
        f.close()

        return len(self.wpoints)

    def save_as_pb(self, filename):
        mission = mission_pb2.Mission()
        for w in self.wpoints:
            waypoint = mission.waypoint.add()
            waypoint.command = w.command
            waypoint.frame = w.frame
            waypoint.seq = w.seq
            waypoint.current = w.current
            waypoint.autocontinue = w.autocontinue
            waypoint.param1 = w.param1
            waypoint.param2 = w.param2
            waypoint.param3 = w.param3
            waypoint.param4 = w.param4
            waypoint.x = w.x
            waypoint.y = w.y
            waypoint.z = w.z
        with open(filename, 'w') as f:
            f.write('QGC WPL PB 110\n')
            f.write(text_format.MessageToString(mission))

    def save(self, filename):
        '''save waypoints to a file'''
        f = open(filename, mode='w')
        f.write("QGC WPL 110\n")
        for w in self.wpoints:
            if getattr(w, 'comment', None):
                f.write("# %s\n" % w.comment)
            f.write("%u\t%u\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%u\n" % (
                w.seq, w.current, w.frame, w.command,
                w.param1, w.param2, w.param3, w.param4,
                w.x, w.y, w.z, w.autocontinue))
        f.close()

    def view_indexes(self, done=None):
        '''return a list waypoint indexes in view order'''
        ret = []
        if done is None:
            done = set()
        idx = 0

        # find first point not done yet
        while idx < self.count():
            if not idx in done:
                break
            idx += 1

        while idx < self.count():
            w = self.wp(idx)
            if idx in done:
                if self.is_location_wp(w):
                    ret.append(idx)
                break
            if w.command in [mavutil.mavlink.MAV_CMD_DO_LAND_START, mavutil.mavlink.MAV_CMD_DO_RETURN_PATH_START]:
                # these are starting points; we should never fly to
                # one of these.... but we want an edge *from* one of these
                if len(ret) == 0:
                    ret.append(idx)
                    done.add(idx)
                idx += 1
                continue
            done.add(idx)
            if w.command == mavutil.mavlink.MAV_CMD_DO_JUMP:
                idx = int(w.param1)
                w = self.wp(idx)
                if self.is_location_wp(w):
                    ret.append(idx)
                continue
            if self.is_location_wp(w):
                ret.append(idx)
            if w.command in [ mavutil.mavlink.MAV_CMD_NAV_LAND,
                              mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND ]:
                # stop at landing points
                return ret
            idx += 1
        return ret

    def polygon(self, done=None):
        '''return a polygon for the waypoints'''
        indexes = self.view_indexes(done)
        points = []
        for idx in indexes:
            w = self.wp(idx)
            if w.command in self.colour_for_polygon:
                points.append((w.x, w.y, self.colour_for_polygon[w.command]))
            else:
                points.append((w.x, w.y))
        return points

    def polygon_list(self):
        '''return a list of polygons for the waypoints'''
        done = set()
        ret = []
        while len(done) != self.count():
            p = self.polygon(done)
            if len(p) > 0:
                ret.append(p)
        return ret

    def view_list(self):
        '''return a list of polygon indexes lists for the waypoints'''
        done = set()
        ret = []
        while len(done) != self.count():
            p = self.view_indexes(done)
            if len(p) > 0:
                ret.append(p)
        return ret

class MAVWPLoader(MissionItemProtocol):
    '''MAVLink waypoint loader'''
    def mav_mission_type(self):
        '''returns type of mission this object transfers'''
        return mavutil.mavlink.MAV_MISSION_TYPE_MISSION

    def wp_is_loiter(self, i):
        '''return true if waypoint is a loiter waypoint'''
        loiter_cmds = [mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT]

        if (self.wpoints[i].command in loiter_cmds):
            return True

        return False

    def add_latlonalt(self, lat, lon, altitude, terrain_alt=False):
        '''add a point via latitude/longitude/altitude'''
        if terrain_alt:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT
        else:
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        p = mavutil.mavlink.MAVLink_mission_item_message(self.target_system,
                                                         self.target_component,
                                                         0,
                                                         frame,
                                                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                         0, 0, 0, 0, 0, 0,
                                                         lat, lon, altitude)
        self.add(p)

    def _read_waypoints_v100(self, file):
        '''read a version 100 waypoint'''
        cmdmap = {
            2 : mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            3 : mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            4 : mavutil.mavlink.MAV_CMD_NAV_LAND,
            24: mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            26: mavutil.mavlink.MAV_CMD_NAV_LAND,
            25: mavutil.mavlink.MAV_CMD_NAV_WAYPOINT ,
            27: mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
            }
        comment = ''
        for line in file:
            if line.startswith('#'):
                comment = line[1:].lstrip()
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 13:
                raise MAVWPError("invalid waypoint line with %u values" % len(a))
            if mavutil.mavlink10():
                fn = mavutil.mavlink.MAVLink_mission_item_message
            else:
                fn = mavutil.mavlink.MAVLink_waypoint_message
            w = fn(self.target_system, self.target_component,
                   int(a[0]),    # seq
                   int(a[1]),    # frame
                   int(a[2]),    # action
                   int(a[7]),    # current
                   int(a[12]),   # autocontinue
                   float(a[5]),  # param1,
                   float(a[6]),  # param2,
                   float(a[3]),  # param3
                   float(a[4]),  # param4
                   float(a[9]),  # x, latitude
                   float(a[8]),  # y, longitude
                   float(a[10])  # z
                   )
            if not w.command in cmdmap:
                raise MAVWPError("Unknown v100 waypoint action %u" % w.command)

            if self.mav_mission_type() != mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                w.mission_type = self.mav_mission_type()

            w.command = cmdmap[w.command]
            self.add(w, comment)
            comment = ''

    def is_location_command(self, cmd):
        '''see if cmd is a MAV_CMD with a latitude/longitude'''
        mav_cmd = mavutil.mavlink.enums['MAV_CMD']
        if not cmd in mav_cmd:
            return False
        return getattr(mav_cmd[cmd],'has_location',True)

    def is_location_wp(self, w):
        '''see if w.command is a MAV_CMD with a latitude/longitude'''
        if w.command == mavutil.mavlink.MAV_CMD_DO_LAND_START:
            # technically stores a location, but we do not navigate there!
            return False
        if w.x == 0 and w.y == 0:
            return False
        return self.is_location_command(w.command)


def get_first_line_from_file(filename):
    '''return line from filename that doesn't start with a #'''
    f = open(filename, mode='r')
    while True:
        line = f.readline().strip()
        if len(line) == 0:
            return None
        if not line.startswith("#"):
            break

    f.close()
    return line

class MissionItemProtocol_Fence(MissionItemProtocol):
    '''New mission-item-protocol-based class for sending fence points to
    autopilot'''

    def mav_mission_type(self):
        '''returns type of mission this object transfers'''
        return mavutil.mavlink.MAV_MISSION_TYPE_FENCE

    def is_location_command(self, cmd):
        '''returns true if cmd nominates a location in param5/param6/param7'''
        return True

    def load(self, filename):
        '''try to load from an old "FENCE_POINT" format file (just a list of
        latlon pairs - or fall back to QGC formats'''

        if not mavutil.mavlink20():
            raise ValueError("Must be using mavlink2")

        version_line = get_first_line_from_file(filename)

        if (version_line is None or
            not re.match(r"[-0-9.]+\s+[-0-9.]+", version_line)):
            return super(MissionItemProtocol_Fence, self).load(filename)

        # shamelessly copy-and-pasted from traditional loader, below
        f = open(filename, mode='r')
        points = []
        for line in f:
            if line.startswith('#'):
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 2:
                raise MAVFenceError("invalid fence point line: %s" % line)
            points.append((float(a[0]),float(a[1])))
        f.close()

        # 1 return point
        # at least 3 vertex points
        # 1 closing point
        if len(points) < 5:
            print("Insufficient points in file")
            return

        items = []
        # return point:
        (ret_lat, ret_lng) = points[0]
        items.append(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                self.target_system,
                self.target_component,
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame
                mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT, # command
                0,  # current
                0,  # autocontinue
                0,  # param1,
                0,  # param2,
                0,  # param3
                0,  # param4
                int(ret_lat * 1e7),  # x (latitude)
                int(ret_lng * 1e7),  # y (longitude)
                0,      # z (altitude)
                self.mav_mission_type(),
            ))
        for i in range(1, len(points)-1):
            (lat, lng) = points[i]
            item = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.target_system,
                self.target_component,
                0,    # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame
                mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, # command
                0,  # current
                0,  # autocontinue
                len(points)-2,  # param1,
                0,  # param2,
                0,  # param3
                0,  # param4
                int(lat * 1e7),  # x (latitude)
                int(lng * 1e7),  # y (longitude)
                0,      # z (altitude)
                self.mav_mission_type(),
            )
            items.append(item)

        self.clear()
        self.add(items)

class MissionItemProtocol_Rally(MissionItemProtocol):
    '''New mission-item-protocol-based class for sending rally points to
    autopilot'''

    def mav_mission_type(self):
        '''returns type of mission this object transfers'''
        return mavutil.mavlink.MAV_MISSION_TYPE_RALLY

    def is_location_command(self, cmd):
        '''returns true if cmd nominates a location in param5/param6/param7'''
        return True

    def load(self, filename):
        '''attempts to load from legacy rally file'''
        if not mavutil.mavlink20():
            raise ValueError("Must be using mavlink2")

        version_line = get_first_line_from_file(filename)
        if version_line is None or not re.match("^RALLY ", version_line):
            return super(MissionItemProtocol_Rally, self).load(filename)

        points = []
        # this code shamelessly copy-and-pasted from the old
        # MAVWPLoader, below
        seq = 0
        f = open(filename, mode='r')
        for line in f:
            if line.startswith('#'):
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 7:
                raise MAVRallyError("invalid rally file line: %s" % line)

            if (a[0].lower() == "rally"):
                lat_deg = float(a[1])
                lng_deg = float(a[2])
                alt = float(a[3])
#                break_alt = float(a[4])
#                land_dir = float(a[5])
#                flags = int(a[6])
                w = mavutil.mavlink.MAVLink_mission_item_int_message(
                    self.target_system, self.target_component,
                    seq,    # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
                    mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT,        # command
                    0,  # current
                    0,  # autocontinue
                    0,  # param1,
                    0,  # param2,
                    0,  # param3
                    0,  # param4
                    int(lat_deg * 1e7),  # x (latitude)
                    int(lng_deg * 1e7),  # y (longitude)
                    alt * 1e3,      # z (altitude)
                    self.mav_mission_type()
                )
                points.append(w)
                seq += 1
        f.close()

        self.clear()
        for point in points:
            self.add(point)


class MAVRallyError(Exception):
    '''MAVLink rally point error class'''
    def __init__(self, msg):
        Exception.__init__(self, msg)
        self.message = msg


class MAVRallyLoader(object):
    '''MAVLink Rally points and Rally Land points loader'''
    def __init__(self, target_system=0, target_component=0):
        self.rally_points = []
        self.target_system = target_system
        self.target_component = target_component
        self.last_change = time.time()

    def rally_count(self):
        '''return number of rally points'''
        return len(self.rally_points)

    def rally_point(self, i):
        '''return rally point i'''
        return self.rally_points[i]

    def reindex(self):
        '''reset counters and indexes'''
        for i in range(self.rally_count()):
            self.rally_points[i].count = self.rally_count()
            self.rally_points[i].idx = i
        self.last_change = time.time()
            
    def append_rally_point(self, p):
        '''add rallypoint to end of list'''
        if (self.rally_count() > 9):
           print("Can't have more than 10 rally points, not adding.")
           return

        self.rally_points.append(p)
        self.reindex()

    def create_and_append_rally_point(self, lat, lon, alt, break_alt, land_dir, flags):
        '''add a point via latitude/longitude'''
        p = mavutil.mavlink.MAVLink_rally_point_message(self.target_system, self.target_component,
                                                        self.rally_count(), 0, int(lat), int(lon), int(alt), int(break_alt), int(land_dir), flags)
        self.append_rally_point(p)

    def clear(self):
        '''clear all point lists (rally and rally_land)'''
        self.rally_points = []
        self.last_change = time.time()

    def remove(self, i):
        '''remove a rally point'''
        if i < 1 or i > self.rally_count():
            print("Invalid rally point number %u" % i)
        self.rally_points.pop(i-1)
        self.reindex()

    def move(self, i, lat, lng, change_time=True):
        '''move a rally point'''
        if i < 1 or i > self.rally_count():
            print("Invalid rally point number %u" % i)
        self.rally_points[i-1].lat = int(lat*1e7)
        self.rally_points[i-1].lng = int(lng*1e7)
        if change_time:
            self.last_change = time.time()

    def set_alt(self, i, alt, break_alt=None, change_time=True):
        '''set rally point altitude(s)'''
        if i < 1 or i > self.rally_count():
            print("Invalid rally point number %u" % i)
            return
        self.rally_points[i-1].alt = int(alt)
        if break_alt is not None:
            self.rally_points[i-1].break_alt = break_alt
        if change_time:
            self.last_change = time.time()

    def load(self, filename):
        '''load rally and rally_land points from a file.
         returns number of points loaded'''
        f = open(filename, mode='r')
        self.clear()
        for line in f:
            if line.startswith('#'):
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 7:
                raise MAVRallyError("invalid rally file line: %s" % line)

            if (a[0].lower() == "rally"):
                self.create_and_append_rally_point(float(a[1]) * 1e7, float(a[2]) * 1e7,
                                                   float(a[3]), float(a[4]), float(a[5]) * 100.0, int(a[6]))
        f.close()
        return len(self.rally_points)

    def save(self, filename):
        '''save fence points to a file'''
        f = open(filename, mode='w')
        for p in self.rally_points:
            f.write("RALLY %f\t%f\t%f\t%f\t%f\t%d\n" % (p.lat * 1e-7, p.lng * 1e-7, p.alt,
                                                        p.break_alt, p.land_dir, p.flags))
        f.close()

class MAVFenceError(Exception):
        '''MAVLink fence error class'''
        def __init__(self, msg):
            Exception.__init__(self, msg)
            self.message = msg

class MAVFenceLoader(object):
    '''MAVLink geo-fence loader'''
    def __init__(self, target_system=0, target_component=0):
        self.points = []
        self.target_system = target_system
        self.target_component = target_component
        self.last_change = time.time()

    def count(self):
        '''return number of points'''
        return len(self.points)

    def point(self, i):
        '''return a point'''
        return self.points[i]

    def add(self, p):
        '''add a point'''
        self.points.append(p)
        self.reindex()

    def reindex(self):
        '''reindex waypoints'''
        for i in range(self.count()):
            w = self.points[i]
            w.idx = i
            w.count = self.count()
            w.target_system = self.target_system
            w.target_component = self.target_component
        self.last_change = time.time()

    def add_latlon(self, lat, lon):
        '''add a point via latitude/longitude'''
        p = mavutil.mavlink.MAVLink_fence_point_message(self.target_system, self.target_component,
                                                        self.count(), 0, lat, lon)
        self.add(p)

    def clear(self):
        '''clear point list'''
        self.points = []
        self.last_change = time.time()

    def load(self, filename):
        '''load points from a file.
        returns number of points loaded'''
        f = open(filename, mode='r')
        self.clear()
        for line in f:
            if line.startswith('#'):
                continue
            line = line.strip()
            if not line:
                continue
            a = line.split()
            if len(a) != 2:
                raise MAVFenceError("invalid fence point line: %s" % line)
            self.add_latlon(float(a[0]), float(a[1]))
        f.close()
        return len(self.points)

    def save(self, filename):
        '''save fence points to a file'''
        f = open(filename, mode='w')
        for p in self.points:
            f.write("%f\t%f\n" % (p.lat, p.lng))
        f.close()

    def move(self, i, lat, lng, change_time=True):
        '''move a fence point'''
        if i < 0 or i >= self.count():
            print("Invalid fence point number %u" % i)
        self.points[i].lat = lat
        self.points[i].lng = lng
        # ensure we close the polygon
        if i == 1:
                self.points[self.count()-1].lat = lat
                self.points[self.count()-1].lng = lng
        if i == self.count() - 1:
                self.points[1].lat = lat
                self.points[1].lng = lng
        if change_time:
            self.last_change = time.time()

    def remove(self, i, change_time=True):
        '''remove a fence point'''
        if i < 0 or i >= self.count():
            print("Invalid fence point number %u" % i)
        self.points.pop(i)
         # ensure we close the polygon
        if i == 1:
                self.points[self.count()-1].lat = self.points[1].lat
                self.points[self.count()-1].lng = self.points[1].lng
        if i == self.count():
                self.points[1].lat = self.points[self.count()-1].lat
                self.points[1].lng = self.points[self.count()-1].lng
        if change_time:
            self.last_change = time.time()

    def polygon(self):
            '''return a polygon for the fence'''
            points = []
            for fp in self.points[1:]:
                    points.append((fp.lat, fp.lng))
            return points

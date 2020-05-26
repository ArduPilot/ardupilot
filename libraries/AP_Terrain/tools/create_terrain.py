#!/usr/bin/env python
'''
create ardupilot terrain database files
'''

from MAVProxy.modules.mavproxy_map import srtm
import math, struct, os, sys
import crc16, time, struct

# MAVLink sends 4x4 grids
TERRAIN_GRID_MAVLINK_SIZE = 4

# a 2k grid_block on disk contains 8x7 of the mavlink grids.  Each
# grid block overlaps by one with its neighbour. This ensures that
# the altitude at any point can be calculated from a single grid
# block
TERRAIN_GRID_BLOCK_MUL_X = 7
TERRAIN_GRID_BLOCK_MUL_Y = 8

# this is the spacing between 32x28 grid blocks, in grid_spacing units
TERRAIN_GRID_BLOCK_SPACING_X = ((TERRAIN_GRID_BLOCK_MUL_X-1)*TERRAIN_GRID_MAVLINK_SIZE)
TERRAIN_GRID_BLOCK_SPACING_Y = ((TERRAIN_GRID_BLOCK_MUL_Y-1)*TERRAIN_GRID_MAVLINK_SIZE)

# giving a total grid size of a disk grid_block of 32x28
TERRAIN_GRID_BLOCK_SIZE_X = (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_X)
TERRAIN_GRID_BLOCK_SIZE_Y = (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_Y)

# format of grid on disk
TERRAIN_GRID_FORMAT_VERSION = 1

IO_BLOCK_SIZE = 2048

GRID_SPACING = 100

def to_float32(f):
    '''emulate single precision float'''
    return struct.unpack('f', struct.pack('f',f))[0]

LOCATION_SCALING_FACTOR = to_float32(0.011131884502145034)
LOCATION_SCALING_FACTOR_INV = to_float32(89.83204953368922)

def longitude_scale(lat):
    '''get longitude scale factor'''
    scale = to_float32(math.cos(to_float32(math.radians(lat))))
    return max(scale, 0.01)

def get_distance_NE_e7(lat1, lon1, lat2, lon2):
    '''get distance tuple between two positions in 1e7 format'''
    return ((lat2 - lat1) * LOCATION_SCALING_FACTOR, (lon2 - lon1) * LOCATION_SCALING_FACTOR * longitude_scale(lat1*1.0e-7))

def add_offset(lat_e7, lon_e7, ofs_north, ofs_east):
    '''add offset in meters to a position'''
    dlat = int(float(ofs_north) * LOCATION_SCALING_FACTOR_INV)
    dlng = int((float(ofs_east) * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat_e7*1.0e-7))
    return (int(lat_e7+dlat), int(lon_e7+dlng))

def east_blocks(lat_e7, lon_e7):
    '''work out how many blocks per stride on disk'''
    lat2_e7 = lat_e7
    lon2_e7 = lon_e7 + 10*1000*1000

    # shift another two blocks east to ensure room is available
    lat2_e7, lon2_e7 = add_offset(lat2_e7, lon2_e7, 0, 2*GRID_SPACING*TERRAIN_GRID_BLOCK_SIZE_Y)
    offset = get_distance_NE_e7(lat_e7, lon_e7, lat2_e7, lon2_e7)
    return int(offset[1] / (GRID_SPACING*TERRAIN_GRID_BLOCK_SPACING_Y))

def pos_from_file_offset(lat_degrees, lon_degrees, file_offset):
    '''return a lat/lon in 1e7 format given a file offset'''

    ref_lat = int(lat_degrees*10*1000*1000)
    ref_lon = int(lon_degrees*10*1000*1000)

    stride = east_blocks(ref_lat, ref_lon)
    blocks = file_offset // IO_BLOCK_SIZE
    grid_idx_x = blocks // stride
    grid_idx_y = blocks % stride

    idx_x = grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X
    idx_y = grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y
    offset = (idx_x * GRID_SPACING, idx_y * GRID_SPACING)

    (lat_e7, lon_e7) = add_offset(ref_lat, ref_lon, offset[0], offset[1])

    offset = get_distance_NE_e7(ref_lat, ref_lon, lat_e7, lon_e7)
    grid_idx_x = int(idx_x / TERRAIN_GRID_BLOCK_SPACING_X)
    grid_idx_y = int(idx_y / TERRAIN_GRID_BLOCK_SPACING_Y)

    (lat_e7, lon_e7) = add_offset(ref_lat, ref_lon,
                                  grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X * float(GRID_SPACING),
                                  grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y * float(GRID_SPACING))

    return (lat_e7, lon_e7)
            
class GridBlock(object):
    def __init__(self, lat_int, lon_int, lat, lon):
        '''
        a grid block is a structure in a local file containing height
        information. Each grid block is 2048 bytes in size, to keep file IO to
        block oriented SD cards efficient
        '''

        # crc of whole block, taken with crc=0
        self.crc = 0

        # format version number
        self.version = TERRAIN_GRID_FORMAT_VERSION

        # grid spacing in meters
        self.spacing = GRID_SPACING

        # heights in meters over a 32*28 grid
        self.height = []
        for x in range(TERRAIN_GRID_BLOCK_SIZE_X):
            self.height.append([0]*TERRAIN_GRID_BLOCK_SIZE_Y)

        # bitmap of 4x4 grids filled in from GCS (56 bits are used)
        self.bitmap = (1<<56)-1

        lat_e7 = int(lat * 1.0e7)
        lon_e7 = int(lon * 1.0e7)

        # grids start on integer degrees. This makes storing terrain data on
        # the SD card a bit easier. Note that this relies on the python floor
        # behaviour with integer division
        self.lat_degrees = lat_int
        self.lon_degrees = lon_int

        # create reference position for this rounded degree position
        ref_lat = self.lat_degrees*10*1000*1000
        ref_lon = self.lon_degrees*10*1000*1000

        # find offset from reference
        offset = get_distance_NE_e7(ref_lat, ref_lon, lat_e7, lon_e7)

        offset = (round(offset[0]), round(offset[1]))

        # get indices in terms of grid_spacing elements
        idx_x = int(offset[0] / GRID_SPACING)
        idx_y = int(offset[1] / GRID_SPACING)

        # find indexes into 32*28 grids for this degree reference. Note
        # the use of TERRAIN_GRID_BLOCK_SPACING_{X,Y} which gives a one square
        # overlap between grids
        self.grid_idx_x = idx_x // TERRAIN_GRID_BLOCK_SPACING_X
        self.grid_idx_y = idx_y // TERRAIN_GRID_BLOCK_SPACING_Y

        # calculate lat/lon of SW corner of 32*28 grid_block
        (ref_lat, ref_lon) = add_offset(ref_lat, ref_lon,
                                        self.grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X * float(GRID_SPACING),
                                        self.grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y * float(GRID_SPACING))
        self.lat = ref_lat
        self.lon = ref_lon

    def fill(self, gx, gy, altitude):
        '''fill a square'''
        self.height[gx][gy] = int(altitude)

    def blocknum(self):
        '''find IO block number'''
        stride = east_blocks(self.lat_degrees*1e7, self.lon_degrees*1e7)
        return stride * self.grid_idx_x + self.grid_idx_y

class DataFile(object):
    def __init__(self, lat, lon):
        if lat < 0:
            NS = 'S'
        else:
            NS = 'N'
        if lon < 0:
            EW = 'W'
        else:
            EW = 'E'
        name = "terrain/%c%02u%c%03u.DAT" % (NS, min(abs(int(lat)), 99),
                                        EW, min(abs(int(lon)), 999))
        try:
            os.mkdir("terrain")
        except Exception:
            pass
        if not os.path.exists(name):
            self.fh = open(name, 'w+b')
        else:
            self.fh = open(name, 'r+b')

    def seek_offset(self, block):
        '''seek to right offset'''
        # work out how many longitude blocks there are at this latitude
        file_offset = block.blocknum() * IO_BLOCK_SIZE
        self.fh.seek(file_offset)

    def pack(self, block):
        '''pack into a block'''
        buf = bytes()
        buf += struct.pack("<QiiHHH", block.bitmap, block.lat, block.lon, block.crc, block.version, block.spacing)
        for gx in range(TERRAIN_GRID_BLOCK_SIZE_X):
            buf += struct.pack("<%uh" % TERRAIN_GRID_BLOCK_SIZE_Y, *block.height[gx])
        buf += struct.pack("<HHhb", block.grid_idx_x, block.grid_idx_y, block.lon_degrees, block.lat_degrees)
        return buf

    def write(self, block):
        '''write a grid block'''
        self.seek_offset(block)
        block.crc = 0
        buf = self.pack(block)
        block.crc = crc16.crc16xmodem(buf)
        buf = self.pack(block)
        self.fh.write(buf)

    def check_filled(self, block):
        '''read a grid block and check if already filled'''
        self.seek_offset(block)
        buf = self.fh.read(IO_BLOCK_SIZE)
        if len(buf) != IO_BLOCK_SIZE:
            return False
        (bitmap, lat, lon, crc, version, spacing) = struct.unpack("<QiiHHH", buf[:22])
        if (version != TERRAIN_GRID_FORMAT_VERSION or
            abs(lat - block.lat)>2 or
            abs(lon - block.lon)>2 or
            spacing != GRID_SPACING or
            bitmap != (1<<56)-1):
            return False
        buf = buf[:16] + struct.pack("<H", 0) + buf[18:]
        crc2 = crc16.crc16xmodem(buf[:1821])
        if crc2 != crc:
            return False
        return True

def create_degree(lat, lon):
    '''create data file for one degree lat/lon'''
    lat_int = int(math.floor(lat))
    lon_int = int(math.floor((lon)))

    tiles = {}

    dfile = DataFile(lat_int, lon_int)

    print("Creating for %d %d" % (lat_int, lon_int))

    total_blocks = east_blocks(lat_int*1e7, lon_int*1e7) * TERRAIN_GRID_BLOCK_SIZE_Y

    for blocknum in range(total_blocks):
        (lat_e7, lon_e7) = pos_from_file_offset(lat_int, lon_int, blocknum * IO_BLOCK_SIZE)
        lat = lat_e7 * 1.0e-7
        lon = lon_e7 * 1.0e-7
        grid = GridBlock(lat_int, lon_int, lat, lon)
        if grid.blocknum() != blocknum:
            continue
        if not args.force and dfile.check_filled(grid):
            continue
        for gx in range(TERRAIN_GRID_BLOCK_SIZE_X):
            for gy in range(TERRAIN_GRID_BLOCK_SIZE_Y):
                lat_e7, lon_e7 = add_offset(lat*1.0e7, lon*1.0e7, gx*GRID_SPACING, gy*GRID_SPACING)
                lat2_int = int(math.floor(lat_e7*1.0e-7))
                lon2_int = int(math.floor(lon_e7*1.0e-7))
                tile_idx = (lat2_int, lon2_int)
                while not tile_idx in tiles:
                    tile = downloader.getTile(lat2_int, lon2_int)
                    waited = False
                    if tile == 0:
                        print("waiting on download of %d,%d" % (lat2_int, lon2_int))
                        time.sleep(0.3)
                        waited = True
                        continue
                    if waited:
                        print("downloaded %d,%d" % (lat2_int, lon2_int))
                    tiles[tile_idx] = tile
                altitude = tiles[tile_idx].getAltitudeFromLatLon(lat_e7*1.0e-7, lon_e7*1.0e-7)
                grid.fill(gx, gy, altitude)
        dfile.write(grid)

from argparse import ArgumentParser
parser = ArgumentParser(description='terrain data creator')

parser.add_argument("lat", type=float, default=-35.363261)
parser.add_argument("lon", type=float, default=149.165230)
parser.add_argument("--force", action='store_true', help="overwrite existing full blocks")
parser.add_argument("--radius", type=int, default=100, help="radius in km")
parser.add_argument("--debug", action='store_true', default=False)
parser.add_argument("--spacing", type=int, default=100, help="grid spacing in meters")
args = parser.parse_args()

downloader = srtm.SRTMDownloader(debug=args.debug)
downloader.loadFileList()

GRID_SPACING = args.spacing

done = set()

for dx in range(-args.radius, args.radius):
    for dy in range(-args.radius, args.radius):
        (lat2,lon2) = add_offset(args.lat*1e7, args.lon*1e7, dx*1000.0, dy*1000.0)
        lat_int = int(round(lat2 * 1.0e-7))
        lon_int = int(round(lon2 * 1.0e-7))
        tag = (lat_int, lon_int)
        if tag in done:
            continue
        done.add(tag)
        create_degree(lat_int, lon_int)

create_degree(args.lat, args.lon)

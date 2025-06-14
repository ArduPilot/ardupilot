#!/usr/bin/env python3

# flake8: noqa
'''
generate field tables from IGRF13. Note that this requires python3
'''

import igrf
import numpy as np
import datetime
from pathlib import Path
from pymavlink.rotmat import Vector3, Matrix3
import math

import argparse
parser = argparse.ArgumentParser(description='generate mag tables')
parser.add_argument('--sampling-res', type=int, default=10, help='sampling resolution, degrees')
parser.add_argument('--check-error', action='store_true', help='check max error')
parser.add_argument('--filename', type=str, default='tables.cpp', help='tables file')

args = parser.parse_args()

if not Path("AP_Declination.h").is_file():
    raise OSError("Please run this tool from the AP_Declination directory")


def write_table(f,name, table):
    '''write one table'''
    f.write("__EXTFLASHFUNC__ const float AP_Declination::%s[%u][%u] = {\n" %
                (name, NUM_LAT, NUM_LON))
    for i in range(NUM_LAT):
        f.write("    {")
        for j in range(NUM_LON):
            f.write("%.5ff" % table[i][j])
            if j != NUM_LON-1:
                f.write(",")
        f.write("}")
        if i != NUM_LAT-1:
            f.write(",")
        f.write("\n")
    f.write("};\n\n")

date = datetime.datetime.now()

SAMPLING_RES = args.sampling_res
SAMPLING_MIN_LAT = -90
SAMPLING_MAX_LAT = 90
SAMPLING_MIN_LON = -180
SAMPLING_MAX_LON = 180

lats = np.arange(SAMPLING_MIN_LAT, SAMPLING_MAX_LAT+SAMPLING_RES, SAMPLING_RES)
lons = np.arange(SAMPLING_MIN_LON, SAMPLING_MAX_LON+SAMPLING_RES, SAMPLING_RES)

NUM_LAT = lats.size
NUM_LON = lons.size

intensity_table = np.empty((NUM_LAT, NUM_LON))
inclination_table = np.empty((NUM_LAT, NUM_LON))
declination_table = np.empty((NUM_LAT, NUM_LON))

max_error = 0
max_error_pos = None
max_error_field = None

def get_igrf(lat, lon):
    '''return field as [declination_deg, inclination_deg, intensity_gauss]'''
    mag = igrf.igrf(date, glat=lat, glon=lon, alt_km=0., isv=0, itype=1)
    intensity = float(mag.total/1e5)
    inclination = float(mag.incl)
    declination = float(mag.decl)
    return [declination, inclination, intensity]

def interpolate_table(table, latitude_deg, longitude_deg):
    '''interpolate inside a table for a given lat/lon in degrees'''
    # round down to nearest sampling resolution
    min_lat = int(math.floor(latitude_deg / SAMPLING_RES) * SAMPLING_RES)
    min_lon = int(math.floor(longitude_deg / SAMPLING_RES) * SAMPLING_RES)

    # find index of nearest low sampling point
    min_lat_index = int(math.floor(-(SAMPLING_MIN_LAT) + min_lat) / SAMPLING_RES)
    min_lon_index = int(math.floor(-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES)

    # calculate intensity
    data_sw = table[min_lat_index][min_lon_index]
    data_se = table[min_lat_index][min_lon_index + 1]
    data_ne = table[min_lat_index + 1][min_lon_index + 1]
    data_nw = table[min_lat_index + 1][min_lon_index]

    # perform bilinear interpolation on the four grid corners
    data_min = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_se - data_sw) + data_sw
    data_max = ((longitude_deg - min_lon) / SAMPLING_RES) * (data_ne - data_nw) + data_nw

    value = ((latitude_deg - min_lat) / SAMPLING_RES) * (data_max - data_min) + data_min
    return value


'''
calculate magnetic field intensity and orientation, interpolating in tables

returns array [declination_deg, inclination_deg, intensity] or None
'''
def interpolate_field(latitude_deg, longitude_deg):
    # limit to table bounds
    if latitude_deg < SAMPLING_MIN_LAT:
        return None
    if latitude_deg >= SAMPLING_MAX_LAT:
        return None
    if longitude_deg < SAMPLING_MIN_LON:
        return None
    if longitude_deg >= SAMPLING_MAX_LON:
        return None

    intensity_gauss = interpolate_table(intensity_table, latitude_deg, longitude_deg)
    declination_deg = interpolate_table(declination_table, latitude_deg, longitude_deg)
    inclination_deg = interpolate_table(inclination_table, latitude_deg, longitude_deg)

    return [declination_deg, inclination_deg, intensity_gauss]

def field_to_Vector3(mag):
    '''return mGauss field from dec, inc and intensity'''
    R = Matrix3()
    mag_ef = Vector3(mag[2]*1000.0, 0.0, 0.0)
    R.from_euler(0.0, -math.radians(mag[1]), math.radians(mag[0]))
    return R * mag_ef

def test_error(lat, lon):
    '''check for error from lat,lon'''
    global max_error, max_error_pos, max_error_field
    mag1 = get_igrf(lat, lon)
    mag2 = interpolate_field(lat, lon)
    ef1 = field_to_Vector3(mag1)
    ef2 = field_to_Vector3(mag2)
    err = (ef1 - ef2).length()
    if err > max_error or err > 100:
        print(lat, lon, err, ef1, ef2)
        max_error = err
        max_error_pos = (lat, lon)
        max_error_field = ef1 - ef2

def test_max_error(lat, lon):
    '''check for maximum error from lat,lon over SAMPLING_RES range'''
    steps = 3
    delta = SAMPLING_RES/steps
    for i in range(steps):
        for j in range(steps):
            lat2 = lat + i * delta
            lon2 = lon + j * delta
            if lat2 >= SAMPLING_MAX_LAT or lon2 >= SAMPLING_MAX_LON:
                continue
            if lat2 <= SAMPLING_MIN_LAT or lon2 <= SAMPLING_MIN_LON:
                continue
            test_error(lat2, lon2)

for i,lat in enumerate(lats):
    for j,lon in enumerate(lons):
        mag = get_igrf(lat, lon)
        declination_table[i][j] = mag[0]
        inclination_table[i][j] = mag[1]
        intensity_table[i][j] = mag[2]

with open(args.filename, 'w') as f:
    f.write('''// this is an auto-generated file from the IGRF tables. Do not edit
// To re-generate run generate/generate.py

#include "AP_Declination.h"

''')

    f.write('''const float AP_Declination::SAMPLING_RES = %u;
const float AP_Declination::SAMPLING_MIN_LAT = %u;
const float AP_Declination::SAMPLING_MAX_LAT = %u;
const float AP_Declination::SAMPLING_MIN_LON = %u;
const float AP_Declination::SAMPLING_MAX_LON = %u;

''' % (SAMPLING_RES,
           SAMPLING_MIN_LAT,
           SAMPLING_MAX_LAT,
           SAMPLING_MIN_LON,
           SAMPLING_MAX_LON))


    write_table(f,'declination_table', declination_table)
    write_table(f,'inclination_table', inclination_table)
    write_table(f,'intensity_table', intensity_table)

if args.check_error:
    print("Checking for maximum error")
    for lat in range(-60,60,1):
        for lon in range(-180,180,1):
            test_max_error(lat, lon)
    print("Generated with max error %.2f %s at (%.2f,%.2f)" % (
        max_error, max_error_field, max_error_pos[0], max_error_pos[1]))

print("Table generated in %s" % args.filename)

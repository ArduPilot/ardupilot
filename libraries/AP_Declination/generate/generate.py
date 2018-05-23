#!/usr/bin/env python
'''
generate field tables from IGRF12
'''

import pyigrf12 as igrf
import numpy as np
import datetime
from pathlib import Path
import sys
from math import sqrt, pi, atan2, asin

if not Path("AP_Declination.h").is_file():
    raise OSError("Please run this tool from the AP_Declination directory")


def field_to_angles(x, y, z):
    intensity = sqrt(x**2+y**2+z**2)
    declination = np.degrees(atan2(y, x))
    inclination = np.degrees(asin(z / intensity))
    return [intensity, inclination, declination]


def write_table(f,name, table):
    '''write one table'''
    f.write("const float AP_Declination::%s[%u][%u] = {\n" %
                (name, NUM_LAT, NUM_LON))
    for i in range(NUM_LAT):
        f.write("    {")
        for j in range(NUM_LON):
            f.write("%.5f" % table[i][j])
            if j != NUM_LON-1:
                f.write(",")
        f.write("}")
        if i != NUM_LAT-1:
            f.write(",")
        f.write("\n")
    f.write("};\n\n")




if __name__ == '__main__':

    date = datetime.datetime.now()
   # date = datetime.date(2018,2,20)

    SAMPLING_RES = 10
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

    for i,lat in enumerate(lats):
        for j,lon in enumerate(lons):
            mag = igrf.runigrf(date=date, glat=lat, glon=lon, alt=0., isv=0, itype=1)
            intensity, I, D = field_to_angles(mag.Bnorth, mag.Beast, mag.Bvert)
            intensity_table[i][j] = intensity / 1e5
            inclination_table[i][j] = I
            declination_table[i][j] = D

    with open("tables.cpp", 'w') as f:
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


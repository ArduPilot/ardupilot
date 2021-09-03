#!/usr/bin/env python3
'''
generate some test vectors for autotest
'''


from pymavlink import mavextra
import random
import argparse
parser = argparse.ArgumentParser(description='generate test vectors')
parser.add_argument('--num-samples', type=int, default=100, help='number of samples')

args = parser.parse_args()

for i in range(args.num_samples):
    lat = random.uniform(-89,89)
    lon = random.uniform(-180,180)
    mavextra.earth_field = None
    m = mavextra.expected_earth_field_lat_lon(lat, lon)
    print("{%f, %f, {%.3f, %.3f, %.3f}}," % (lat, lon, m.x, m.y, m.z))

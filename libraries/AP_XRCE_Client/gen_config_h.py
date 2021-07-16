#!/usr/bin/env python
'''
 process a *.h.in file to produce a *.h file
'''

import sys
import re

config = {
    'PROJECT_VERSION_MAJOR' : 1,
    'PROJECT_VERSION_MINOR' : 0,
    'PROJECT_VERSION_PATCH' : 0,
    'PROJECT_VERSION' : 2,
    'UCLIENT_MAX_OUTPUT_BEST_EFFORT_STREAMS' : 4,
    'UCLIENT_MAX_OUTPUT_RELIABLE_STREAMS' : 4,
    'UCLIENT_MAX_INPUT_BEST_EFFORT_STREAMS' : 4,
    'UCLIENT_MAX_INPUT_RELIABLE_STREAMS' : 2,
    'UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS' : 2,
    'UCLIENT_MIN_SESSION_CONNECTION_INTERVAL' : 2,
    'UCLIENT_MIN_HEARTBEAT_TIME_INTERVAL' : 1,
    'UCLIENT_UDP_TRANSPORT_MTU' : 300,
    'UCLIENT_TCP_TRANSPORT_MTU' : 350,
    'UCLIENT_SERIAL_TRANSPORT_MTU' : 250,
    'UCLIENT_CUSTOM_TRANSPORT_MTU' : 400,
    'CONFIG_MACHINE_ENDIANNESS' : 0,
}

defines = {
    "UCLIENT_PROFILE_UDP" : 0,
    "UCLIENT_PROFILE_TCP" : 0,
    "UCLIENT_PROFILE_SERIAL" : 1,
    "UCLIENT_PROFILE_CUSTOM_TRANSPORT" : 0,
    "UCLIENT_PROFILE_DISCOVERY" : 0,
    "UCLIENT_PLATFORM_POSIX" : 1,
    "UCLIENT_PLATFORM_POSIX_NOPOLL" : 0,
    "UCLIENT_PLATFORM_WINDOWS" : 0,
    "UCLIENT_PLATFORM_FREERTOS_PLUS_TCP" : 0,
    "UCLIENT_PLATFORM_ZEPHYR" : 0,
    "UCLIENT_EXTERNAL_TCP" : 0,
    "UCLIENT_EXTERNAL_UDP" : 0,
    "UCLIENT_EXTERNAL_SERIAL" : 0,
}

h_in = sys.argv[1]
h = sys.argv[2]
print("Processing %s to %s" % (h_in, h))

txt = open(h_in, 'r').read()

for c in sorted(config.keys(), key=len, reverse=True):
    txt = txt.replace("@%s@" % c, str(config[c]))

for c in sorted(defines.keys(), key=len, reverse=True):
    if defines[c] != 0:
        txt = txt.replace("#cmakedefine %s" % c, "#define %s %u" % (c, defines[c]))
    else:
        txt = txt.replace("#cmakedefine %s" % c, "// #define %s %u" % (c, defines[c]))

matches = re.findall(r'@(\w+)@', txt)
if len(matches) > 0:
    print("Missing config elements: ", matches)
    sys.exit(1)

matches = re.findall(r'#cmakedefine\s+\w+', txt)
if len(matches) > 0:
    print("Missing #cmakedefine elements: ", matches)
    sys.exit(1)
    
open(h, 'w').write(txt)

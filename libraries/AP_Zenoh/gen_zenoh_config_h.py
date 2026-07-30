#!/usr/bin/env python3
'''
Process zenoh-pico config.h.in and zenoh-pico.h.in templates
to produce the generated headers needed in the build folder.
'''

import re
import sys

config = {
    'FRAG_MAX_SIZE': 4096,
    'BATCH_UNICAST_SIZE': 2048,
    'BATCH_MULTICAST_SIZE': 2048,
    'Z_CONFIG_SOCKET_TIMEOUT': 100,
    'Z_TRANSPORT_LEASE': 10000,
    'Z_TRANSPORT_LEASE_EXPIRE_FACTOR': 3,
    'ZP_PERIODIC_SCHEDULER_MAX_TASKS': 64,

    'Z_FEATURE_MULTI_THREAD': 1,
    'Z_FEATURE_PUBLICATION': 1,
    'Z_FEATURE_ADVANCED_PUBLICATION': 0,
    'Z_FEATURE_SUBSCRIPTION': 1,
    'Z_FEATURE_ADVANCED_SUBSCRIPTION': 0,
    'Z_FEATURE_QUERY': 1,
    'Z_FEATURE_QUERYABLE': 1,
    'Z_FEATURE_LIVELINESS': 1,
    'Z_FEATURE_RAWETH_TRANSPORT': 0,
    'Z_FEATURE_INTEREST': 1,
    'Z_FEATURE_LINK_TCP': 1,
    'Z_FEATURE_LINK_BLUETOOTH': 0,
    'Z_FEATURE_LINK_WS': 0,
    'Z_FEATURE_LINK_SERIAL': 0,
    'Z_FEATURE_LINK_SERIAL_USB': 0,
    'Z_FEATURE_LINK_TLS': 0,
    'Z_FEATURE_SCOUTING': 1,
    'Z_FEATURE_LINK_UDP_MULTICAST': 1,
    'Z_FEATURE_LINK_UDP_UNICAST': 1,
    'Z_FEATURE_MULTICAST_TRANSPORT': 1,
    'Z_FEATURE_UNICAST_TRANSPORT': 1,
    'Z_FEATURE_FRAGMENTATION': 1,
    'Z_FEATURE_ENCODING_VALUES': 1,
    'Z_FEATURE_TCP_NODELAY': 1,
    'Z_FEATURE_LOCAL_SUBSCRIBER': 0,
    'Z_FEATURE_LOCAL_QUERYABLE': 0,
    'Z_FEATURE_SESSION_CHECK': 1,
    'Z_FEATURE_BATCHING': 1,
    'Z_FEATURE_BATCH_TX_MUTEX': 0,
    'Z_FEATURE_BATCH_PEER_MUTEX': 0,
    'Z_FEATURE_MATCHING': 1,
    'Z_FEATURE_RX_CACHE': 0,
    'Z_FEATURE_UNICAST_PEER': 1,
    'Z_FEATURE_AUTO_RECONNECT': 1,
    'Z_FEATURE_MULTICAST_DECLARATIONS': 0,
    'Z_FEATURE_PERIODIC_TASKS': 0,
    'Z_FEATURE_ADMIN_SPACE': 0,

    'ZENOH_PICO': '1.7.2',
    'ZENOH_PICO_MAJOR': 1,
    'ZENOH_PICO_MINOR': 7,
    'ZENOH_PICO_PATCH': 2,
    'ZENOH_PICO_TWEAK': 0,
}

defines = {
    'Z_FEATURE_UNSTABLE_API': 0,
}

header_input = sys.argv[1]
header_output = sys.argv[2]
print("Processing %s to %s" % (header_input, header_output))

header = open(header_input, 'r').read()

for c in sorted(config.keys(), key=len, reverse=True):
    val = config[c]
    if isinstance(val, str):
        header = header.replace("@%s@" % c, '"%s"' % val)
    else:
        header = header.replace("@%s@" % c, str(val))

for c in sorted(defines.keys(), key=len, reverse=True):
    if defines[c] != 0:
        header = header.replace("#cmakedefine %s" % c, "#define %s %u" % (c, defines[c]))
    else:
        header = header.replace("#cmakedefine %s" % c, "// #define %s" % c)

matches = re.findall(r'@(\w+)@', header)
if len(matches) > 0:
    print("Missing config elements: ", matches)
    sys.exit(1)

matches = re.findall(r'#cmakedefine\s+\w+', header)
if len(matches) > 0:
    print("Missing #cmakedefine elements: ", matches)
    sys.exit(1)

open(header_output, 'w').write(header)

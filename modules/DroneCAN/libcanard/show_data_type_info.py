#!/usr/bin/env python3
#
# Copyright (c) 2016 UAVCAN Team
#
# This script can be used to quickly obtain data type signature value for any UAVCAN data type.
# Execute with --help to get usage info.
#

''''which python3 >/dev/null 2>&1 && exec python3 "$0" "$@" # '''
''''which python  >/dev/null 2>&1 && exec python  "$0" "$@" # '''
''''which python2 >/dev/null 2>&1 && exec python2 "$0" "$@" # '''
''''exec echo "Python not found" >&2 # '''

# We can't import from __future__ here because of the wickedness above.
import sys
import getopt

try:
    import dronecan
except ImportError:
    sys.stderr.write('pyDroneCAN is not installed. Please install from PIP: sudo pip3 install dronecan\n')
    exit(1)

def printUsage():
    print("""show_data_type_info:
    [-h, --help]: show this help
    [-c, --custom] [path/to/custom/types]: path to your custom types '00.mymsgtype.uavcan'
    """)
    
if __name__ == "__main__":
    # decode options given to the script
    try:
        opts, args = getopt.getopt(sys.argv[1:],'hc:',['help','custom='])
    except getopt.GetoptError:
        printUsage()
        sys.exit()
    
    # include/execute options given to the script
    for opt, arg in opts:
        if opt in ('-h','--help'):
            printUsage()
            sys.exit()
        elif opt in ('-c','--custom'):
            dronecan.load_dsdl(arg)
    
    longest_name = max(map(len, dronecan.TYPENAMES.keys()))
    
    header = 'Full Data Type Name'.ljust(longest_name) + ' | DDTID |   Type Signature   |  Max Bit Len  '
    print(header)
    print('-' * len(header))
    
    for typename, typedef in sorted(dronecan.TYPENAMES.items()):
        ddtid = typedef.default_dtid if typedef.default_dtid is not None else 'N/A'
        s = '%-*s   % 5s   0x%016x' % (longest_name, typename, ddtid, typedef.get_data_type_signature())
        try:
            s += '   % 5d' % typedef.get_max_bitlen()
        except Exception:
            s += '   % 5d / %-5d' % (typedef.get_max_bitlen_request(), typedef.get_max_bitlen_response())
        print(s)

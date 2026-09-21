#!/usr/bin/env python3

'''
Check that AP_SerialManager_default_protocol_check.cpp keeps up with the
SerialProtocol enumeration.

serial_protocol_compiled_in() in
libraries/AP_SerialManager/AP_SerialManager_default_protocol_check.cpp gates a
serial port's default protocol against the feature that provides it, e.g.

    (p != AP_SerialManager::SerialProtocol_SLCAN || (AP_CAN_SLCAN_ENABLED)) &&

The helper falls through to "true" for any protocol it does not name, so a
protocol added to the enum without a term there is silently treated as
always-compiled-in -- which defeats the check for that protocol.  There is no
way to notice that from the C++ alone.

This script closes that gap.  It parses the enumerator names from
AP_SerialManager.h and the protocols named by serial_protocol_compiled_in(),
and requires every enumerator to be accounted for as exactly one of:

  * feature-gated -- named in serial_protocol_compiled_in(); or
  * listed in UNGATED below, an explicit register of protocols deliberately
    not gated (with the reason).

A newly-added protocol that is in neither is an error: whoever adds it must
decide which bucket it belongs in.  A name in the C++/UNGATED that is not a
real enumerator (typo, renamed or removed protocol) is also an error.

AP_FLAKE8_CLEAN
'''

import re
import sys

from pathlib import Path


class SerialProtocolGateChecker:

    # The enum sentinel is not a real protocol and is never a port default.
    SENTINEL = 'SerialProtocol_NumProtocols'

    # Protocols deliberately NOT gated by serial_protocol_compiled_in(), each
    # with the reason.  Two kinds live here:
    #
    #   * "no feature" -- no separable compile-time feature, so a port
    #     defaulting to them can never select compiled-out handling; and
    #
    #   * "TODO gate on <MACRO>" -- these DO have a feature and should
    #     eventually get a term in serial_protocol_compiled_in().  Gating them
    #     is behaviour-neutral today (the feature is on in every current build)
    #     and is left to a follow-up.  They are registered here, with the macro
    #     to gate on, so the gap stays explicit and reviewable, not silent.
    #
    # Adding a protocol here (or a term in the C++) is what makes this check
    # pass for it; do not add one without understanding which case it is.
    UNGATED = {
        # always compiled in -- no separable feature
        'SerialProtocol_None':      'no feature: selects "port unused"',
        'SerialProtocol_Console':   'no feature: console is always present when SerialManager is present',

        # feature-gated but not yet gated here -- TODO, behaviour-neutral follow-up
        'SerialProtocol_MAVLink':      'TODO gate on HAL_GCS_ENABLED',
        'SerialProtocol_MAVLink2':     'TODO gate on HAL_GCS_ENABLED',
        'SerialProtocol_MAVLinkHL':    'TODO gate on HAL_GCS_ENABLED',
        'SerialProtocol_GPS':          'TODO gate on AP_GPS_ENABLED',
        'SerialProtocol_GPS2':         'TODO gate on AP_GPS_ENABLED',
        'SerialProtocol_Rangefinder':  'TODO gate on AP_RANGEFINDER_ENABLED',
        'SerialProtocol_Lidar360':     'TODO gate on AP_PROXIMITY_ENABLED',
        'SerialProtocol_AirSpeed':     'TODO gate on AP_AIRSPEED_ENABLED',
        'SerialProtocol_AHRS':         'TODO gate on AP_EXTERNAL_AHRS_ENABLED',
        'SerialProtocol_Sbus1':        'TODO gate on AP_SBUSOUTPUT_ENABLED',
        'SerialProtocol_ESCTelemetry': 'TODO gate on HAVE_AP_BLHELI_SUPPORT',
        'SerialProtocol_IBUS_Telem':   'TODO gate on AP_IBUS_TELEM_ENABLED',
        'SerialProtocol_IOMCU':        'TODO gate on HAL_WITH_IO_MCU',

        # deprecated / no in-tree consumer
        'SerialProtocol_Aerotenna_USD1': 'deprecated: historic rangefinder',
        'SerialProtocol_CoDevESC':       'no in-tree consumer',
    }

    def __init__(self):
        serialmanager = Path(__file__).resolve().parents[2] / 'libraries/AP_SerialManager'
        self.header = serialmanager / 'AP_SerialManager.h'
        self.check_cpp = serialmanager / 'AP_SerialManager_default_protocol_check.cpp'

    def parse_enum(self):
        '''return the ordered list of SerialProtocol_* enumerators (sans sentinel)'''
        m = re.search(r'enum\s+SerialProtocol\s*\{(.*?)\}',
                      self.header.read_text(), re.DOTALL)
        if m is None:
            raise ValueError('could not find "enum SerialProtocol" in the header')
        names = []
        for line in m.group(1).splitlines():
            line = line.split('//')[0]  # drop comments (they mention e.g. _IQ)
            em = re.match(r'\s*(SerialProtocol_\w+)', line)
            if em and em.group(1) != self.SENTINEL:
                names.append(em.group(1))
        if not names:
            raise ValueError('parsed no enumerators from "enum SerialProtocol"')
        return names

    def parse_gated(self):
        '''return the set of protocols named by serial_protocol_compiled_in()'''
        tokens = re.findall(r'p != AP_SerialManager::(SerialProtocol_\w+)',
                            self.check_cpp.read_text())
        return set(tokens)

    def run(self):
        enum = self.parse_enum()
        enum_set = set(enum)
        gated = self.parse_gated()
        ungated = set(self.UNGATED)

        errors = []

        # every enumerator must be gated or explicitly registered as ungated
        unclassified = [p for p in enum if p not in gated and p not in ungated]
        if unclassified:
            errors.append(
                'these SerialProtocol enumerators are neither gated in '
                'serial_protocol_compiled_in() nor listed in UNGATED in this '
                'script -- add a term for each in '
                'AP_SerialManager_default_protocol_check.cpp, or register it in '
                'UNGATED with the reason:\n' +
                '\n'.join('    %s' % p for p in unclassified))

        # names in the C++/register that are not (any longer) real enumerators
        stale_gated = sorted(gated - enum_set)
        if stale_gated:
            errors.append(
                'serial_protocol_compiled_in() names protocols that are not in '
                'the SerialProtocol enum (typo, or renamed/removed protocol):\n' +
                '\n'.join('    %s' % p for p in stale_gated))
        stale_ungated = sorted(ungated - enum_set)
        if stale_ungated:
            errors.append(
                'UNGATED lists protocols that are not in the SerialProtocol '
                'enum (typo, or renamed/removed protocol):\n' +
                '\n'.join('    %s' % p for p in stale_ungated))

        # a protocol cannot be both gated and declared ungated
        contradictory = sorted(gated & ungated)
        if contradictory:
            errors.append(
                'these protocols are both gated in serial_protocol_compiled_in() '
                'and listed in UNGATED -- remove them from UNGATED:\n' +
                '\n'.join('    %s' % p for p in contradictory))

        todo = sorted(p for p, why in self.UNGATED.items()
                      if why.startswith('TODO') and p in enum_set)
        print('serial protocol gate coverage: %u enumerators, %u gated, '
              '%u ungated (%u of them TODO to gate).' %
              (len(enum), len(gated & enum_set),
               len(ungated & enum_set), len(todo)))
        if todo:
            print('known ungated-but-feature-gated protocols (follow-up):')
            for p in todo:
                print('    %-32s %s' % (p, self.UNGATED[p]))

        if errors:
            print('\nFAIL:', file=sys.stderr)
            for e in errors:
                print(e, file=sys.stderr)
            return 1

        print('OK: every SerialProtocol is gated or explicitly registered.')
        return 0


if __name__ == '__main__':
    sys.exit(SerialProtocolGateChecker().run())

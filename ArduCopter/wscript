#!/usr/bin/env python
# encoding: utf-8

def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AP_ADSB',
            'AC_AttitudeControl',
            'AC_InputManager',
            'AC_Fence',
            'AC_Avoidance',
            'AC_PID',
            'AC_PrecLand',
            'AC_Sprayer',
            'AC_WPNav',
            'AP_Camera',
            'AP_EPM',
            'AP_Frsky_Telem',
            'AP_IRLock',
            'AP_InertialNav',
            'AP_LandingGear',
            'AP_Menu',
            'AP_Motors',
            'AP_Parachute',
            'AP_RCMapper',
            'AP_Relay',
            'AP_ServoRelayEvents',
            'AP_Avoidance',
            'AP_AdvancedFailsafe'
        ],
    )

    frames = (
        'quad', 'tri', 'hexa', 'y6', 'octa', 'octa-quad', 'heli', 'single',
        'coax',
    )
    for frame in frames:
        frame_config = frame.upper().replace('-', '_') + '_FRAME'
        bld.ap_program(
            program_name='arducopter-%s' % frame,
            program_groups=['bin', 'copter'],
            use=vehicle + '_libs',
            defines=['FRAME_CONFIG=%s' % frame_config],
        )

#!/usr/bin/env python3

def build(bld):
    vehicle = bld.path.name
    bld.ap_stlib(
        name=vehicle + '_libs',
        ap_vehicle=vehicle,
        ap_libraries=bld.ap_common_vehicle_libraries() + [
            'AC_AttitudeControl',
            'AC_InputManager',
            'AC_PrecLand',
            'AC_Sprayer',
            'AC_Autorotation',
            'AC_WPNav',
            'AP_Camera',
            'AP_IRLock',
            'AP_Motors',
            'AP_Avoidance',
            'AP_AdvancedFailsafe',
            'AP_SmartRTL',
            'AP_WheelEncoder',
            'AP_Winch',
            'AP_LTM_Telem',
            'AP_Devo_Telem',
            'AC_AutoTune',
            'AP_KDECAN',
            'AP_SurfaceDistance'
        ],
    )

    bld.ap_program(
        program_name='arducopter',
        program_groups=['bin', 'copter'],
        use=vehicle + '_libs',
        defines=['FRAME_CONFIG=MULTICOPTER_FRAME'],
        )

    bld.ap_program(
        program_name='arducopter-heli',
        program_groups=['bin', 'heli'],
        use=vehicle + '_libs',
        defines=['FRAME_CONFIG=HELI_FRAME'],
        )

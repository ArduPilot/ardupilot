#!/usr/bin/env python
#
#
# Unit and regression tests for the LogAnalyzer code
#
#

# TODO: implement more unit+regression tests

from __future__ import print_function

import DataflashLog
import traceback
from VehicleType import VehicleType

try:

	# test DataflashLog reading 1
	logdata = DataflashLog.DataflashLog()
	logdata.read("examples/robert_lefebvre_octo_PM.log", ignoreBadlines=False)
	assert(logdata.filename        == "examples/robert_lefebvre_octo_PM.log")
	assert(logdata.vehicleType     == VehicleType.Copter)
	assert(logdata.vehicleTypeString     == "ArduCopter")
	assert(logdata.firmwareVersion == "V3.0.1")
	assert(logdata.firmwareHash    == "5c6503e2")
	assert(logdata.freeRAM         == 1331)
	assert(logdata.hardwareType    == "APM 2")
	assert(len(logdata.formats) == 27)
	assert(logdata.formats['GPS'].labels == ['Status', 'Time', 'NSats', 'HDop', 'Lat', 'Lng', 'RelAlt', 'Alt', 'Spd', 'GCrs'])
	assert(logdata.formats['ATT'].labels == ['RollIn', 'Roll', 'PitchIn', 'Pitch', 'YawIn', 'Yaw', 'NavYaw'])
	assert(logdata.parameters == {'RC7_REV': 1.0, 'MNT_MODE': 3.0, 'LOITER_LON_P': 1.0, 'FLTMODE1': 1.0, 'FLTMODE3': 0.0, 'FLTMODE2': 6.0, 'TUNE_HIGH': 10000.0, 'FLTMODE4': 5.0, 'FLTMODE6': 2.0, 'SYSID_SW_TYPE': 10.0, 'LOITER_LON_D': 0.0, 'RC5_REV': 1.0, 'THR_RATE_IMAX': 300.0, 'MNT_RC_IN_PAN': 0.0, 'RC2_MIN': 1110.0, 'LOITER_LON_I': 0.5, 'HLD_LON_P': 1.0, 'STB_RLL_I': 0.0, 'LOW_VOLT': 10.5, 'MNT_CONTROL_Y': 0.0, 'MNT_CONTROL_X': 0.0, 'FRAME': 1.0, 'MNT_CONTROL_Z': 0.0, 'OF_PIT_IMAX': 100.0, 'AHRS_ORIENTATION': 0.0, 'SIMPLE': 0.0, 'RC2_MAX': 1929.0, 'MNT_JSTICK_SPD': 0.0, 'RC8_FUNCTION': 0.0, 'INS_ACCSCAL_X': 0.992788, 'ACRO_P': 4.5, 'MNT_ANGMIN_ROL': -4500.0, 'OF_RLL_P': 2.5, 'STB_RLL_P': 3.5, 'STB_YAW_P': 3.0, 'SR0_RAW_SENS': 2.0, 'FLTMODE5': 0.0, 'RATE_YAW_I': 0.02, 'MAG_ENABLE': 1.0, 'MNT_RETRACT_Y': 0.0, 'MNT_RETRACT_X': 0.0, 'RATE_YAW_IMAX': 800.0, 'WPNAV_SPEED_DN': 150.0, 'WP_YAW_BEHAVIOR': 2.0, 'RC11_REV': 1.0, 'SYSID_THISMAV': 1.0, 'SR0_EXTRA1': 10.0, 'SR0_EXTRA2': 10.0, 'ACRO_BAL_PITCH': 200.0, 'STB_YAW_I': 0.0, 'INS_ACCSCAL_Z': 0.97621, 'INS_ACCSCAL_Y': 1.00147, 'LED_MODE': 9.0, 'FS_GCS_ENABLE': 0.0, 'MNT_RC_IN_ROLL': 0.0, 'INAV_TC_Z': 8.0, 'RATE_PIT_IMAX': 4500.0, 'HLD_LON_IMAX': 3000.0, 'THR_RATE_I': 0.0, 'SR3_EXTRA1': 0.0, 'STB_PIT_IMAX': 800.0, 'AHRS_TRIM_Z': 0.0, 'RC2_REV': 1.0, 'INS_MPU6K_FILTER': 20.0, 'THR_MIN': 130.0, 'AHRS_TRIM_Y': 0.021683, 'RC11_DZ': 0.0, 'THR_MAX': 1000.0, 'SR3_EXTRA2': 0.0, 'MNT_NEUTRAL_Z': 0.0, 'THR_MID': 300.0, 'MNT_NEUTRAL_X': 0.0, 'AMP_PER_VOLT': 18.002001, 'SR0_POSITION': 3.0, 'MNT_STAB_PAN': 0.0, 'FS_BATT_ENABLE': 0.0, 'LAND_SPEED': 50.0, 'OF_PIT_D': 0.12, 'SR0_PARAMS': 50.0, 'COMPASS_ORIENT': 0.0, 'WPNAV_ACCEL': 200.0, 'THR_ACCEL_IMAX': 5000.0, 'SR3_POSITION': 0.0, 'WPNAV_RADIUS': 100.0, 'WP_TOTAL': 14.0, 'RC8_MAX': 1856.0, 'OF_PIT_P': 2.5, 'SR3_RAW_SENS': 0.0, 'RTL_ALT_FINAL': 0.0, 'SR3_PARAMS': 0.0, 'SR0_EXTRA3': 2.0, 'LOITER_LAT_I': 0.5, 'RC6_DZ': 0.0, 'RC4_TRIM': 1524.0, 'RATE_RLL_P': 0.07, 'LOITER_LAT_D': 0.0, 'STB_PIT_P': 3.5, 'OF_PIT_I': 0.5, 'RATE_RLL_I': 1.0, 'AHRS_TRIM_X': 0.003997, 'RC3_REV': 1.0, 'STB_PIT_I': 0.0, 'FS_THR_ENABLE': 0.0, 'LOITER_LAT_P': 1.0, 'AHRS_RP_P': 0.1, 'FENCE_ACTION': 1.0, 'TOY_RATE': 1.0, 'RATE_RLL_D': 0.006, 'RC5_MIN': 1151.0, 'RC5_TRIM': 1676.0, 'STB_RLL_IMAX': 800.0, 'RC4_DZ': 40.0, 'AHRS_YAW_P': 0.1, 'RC11_TRIM': 1500.0, 'MOT_TCRV_ENABLE': 1.0, 'CAM_TRIGG_TYPE': 1.0, 'STB_YAW_IMAX': 800.0, 'RC4_MAX': 1942.0, 'LOITER_LAT_IMAX': 400.0, 'CH7_OPT': 9.0, 'RC11_FUNCTION': 7.0, 'SR0_EXT_STAT': 2.0, 'SONAR_TYPE': 0.0, 'RC3_MAX': 1930.0, 'RATE_YAW_D': 0.0, 'FENCE_ALT_MAX': 30.0, 'COMPASS_MOT_Y': 0.0, 'AXIS_ENABLE': 1.0, 'FENCE_ENABLE': 0.0, 'RC10_DZ': 0.0, 'PILOT_VELZ_MAX': 250.0, 'BATT_CAPACITY': 1760.0, 'FS_THR_VALUE': 975.0, 'RC4_MIN': 1115.0, 'MNT_ANGMAX_TIL': 4500.0, 'RTL_LOIT_TIME': 5000.0, 'ARMING_CHECK': 1.0, 'THR_RATE_P': 6.0, 'OF_RLL_IMAX': 100.0, 'RC6_MIN': 971.0, 'SR0_RAW_CTRL': 0.0, 'RC6_MAX': 2078.0, 'RC5_MAX': 1829.0, 'LOITER_LON_IMAX': 400.0, 'MNT_STAB_TILT': 0.0, 'MOT_TCRV_MIDPCT': 52.0, 'COMPASS_OFS_Z': -5.120774, 'COMPASS_OFS_Y': 46.709824, 'COMPASS_OFS_X': -20.490345, 'THR_ALT_I': 0.0, 'RC10_TRIM': 1500.0, 'INS_PRODUCT_ID': 88.0, 'RC11_MIN': 1100.0, 'FS_GPS_ENABLE': 1.0, 'HLD_LAT_IMAX': 3000.0, 'RC3_TRIM': 1476.0, 'RC6_FUNCTION': 0.0, 'TRIM_THROTTLE': 260.0, 'MNT_STAB_ROLL': 0.0, 'INAV_TC_XY': 2.5, 'RC1_DZ': 30.0, 'MNT_RETRACT_Z': 0.0, 'THR_ACC_ENABLE': 1.0, 'LOG_BITMASK': 830.0, 'TUNE_LOW': 0.0, 'CIRCLE_RATE': 5.0, 'CAM_DURATION': 10.0, 'MNT_NEUTRAL_Y': 0.0, 'RC10_MIN': 1100.0, 'INS_ACCOFFS_X': -0.019376, 'THR_RATE_D': 0.0, 'INS_ACCOFFS_Z': 1.370947, 'RC4_REV': 1.0, 'CIRCLE_RADIUS': 10.0, 'RATE_RLL_IMAX': 4500.0, 'HLD_LAT_P': 1.0, 'AHRS_GPS_MINSATS': 6.0, 'FLOW_ENABLE': 0.0, 'RC8_REV': 1.0, 'SONAR_GAIN': 0.2, 'RC2_TRIM': 1521.0, 'WP_INDEX': 0.0, 'RC1_REV': 1.0, 'RC7_DZ': 0.0, 'AHRS_GPS_USE': 1.0, 'MNT_ANGMIN_PAN': -4500.0, 'SR3_RC_CHAN': 0.0, 'COMPASS_LEARN': 0.0, 'ACRO_TRAINER': 1.0, 'CAM_SERVO_OFF': 1100.0, 'RC5_DZ': 0.0, 'SCHED_DEBUG': 0.0, 'RC11_MAX': 1900.0, 'AHRS_WIND_MAX': 0.0, 'SR3_EXT_STAT': 0.0, 'MNT_ANGMAX_PAN': 4500.0, 'MNT_ANGMAX_ROL': 4500.0, 'RC_SPEED': 490.0, 'SUPER_SIMPLE': 0.0, 'VOLT_DIVIDER': 10.0, 'COMPASS_MOTCT': 0.0, 'SR3_RAW_CTRL': 0.0, 'SONAR_ENABLE': 0.0, 'INS_ACCOFFS_Y': 0.362242, 'SYSID_SW_MREV': 120.0, 'WPNAV_LOIT_SPEED': 1000.0, 'BATT_MONITOR': 4.0, 'MNT_RC_IN_TILT': 8.0, 'CH8_OPT': 0.0, 'RTL_ALT': 1000.0, 'SR0_RC_CHAN': 2.0, 'RC1_MIN': 1111.0, 'RSSI_PIN': -1.0, 'MOT_TCRV_MAXPCT': 93.0, 'GND_ABS_PRESS': 101566.97, 'RC1_MAX': 1936.0, 'FENCE_TYPE': 3.0, 'RC5_FUNCTION': 0.0, 'OF_RLL_D': 0.12, 'BATT_VOLT_PIN': 13.0, 'WPNAV_SPEED': 1000.0, 'RC7_MAX': 1884.0, 'CAM_SERVO_ON': 1300.0, 'RATE_PIT_I': 1.0, 'RC7_MIN': 969.0, 'AHRS_COMP_BETA': 0.1, 'OF_RLL_I': 0.5, 'COMPASS_DEC': 0.0, 'RC3_MIN': 1113.0, 'RC2_DZ': 30.0, 'FENCE_RADIUS': 30.0, 'HLD_LON_I': 0.0, 'ACRO_BAL_ROLL': 200.0, 'COMPASS_AUTODEC': 1.0, 'SR3_EXTRA3': 0.0, 'COMPASS_USE': 1.0, 'RC10_MAX': 1900.0, 'RATE_PIT_P': 0.07, 'GND_TEMP': 21.610104, 'RC7_TRIM': 970.0, 'RC10_REV': 1.0, 'RATE_YAW_P': 0.2, 'THR_ALT_P': 1.0, 'RATE_PIT_D': 0.006, 'ESC': 0.0, 'MNT_ANGMIN_TIL': -4500.0, 'SERIAL3_BAUD': 57.0, 'RC8_MIN': 968.0, 'THR_ALT_IMAX': 300.0, 'SYSID_MYGCS': 255.0, 'INS_GYROFFS_Y': 0.581989, 'TUNE': 0.0, 'RC8_TRIM': 970.0, 'RC3_DZ': 30.0, 'AHRS_GPS_GAIN': 1.0, 'THR_ACCEL_D': 0.0, 'TELEM_DELAY': 0.0, 'THR_ACCEL_I': 0.5, 'COMPASS_MOT_X': 0.0, 'COMPASS_MOT_Z': 0.0, 'RC10_FUNCTION': 0.0, 'INS_GYROFFS_X': -0.001698, 'INS_GYROFFS_Z': 0.01517, 'RC6_TRIM': 1473.0, 'THR_ACCEL_P': 1.2, 'RC8_DZ': 0.0, 'HLD_LAT_I': 0.0, 'RC7_FUNCTION': 0.0, 'RC6_REV': 1.0, 'BATT_CURR_PIN': 12.0, 'WPNAV_SPEED_UP': 250.0, 'RC1_TRIM': 1524.0})
	assert(logdata.messages == {})
	assert(logdata.modeChanges == {2204: ('LOITER', 269), 4594: ('STABILIZE', 269), 644: ('ALT_HOLD', 269), 4404: ('ALT_HOLD', 269)})
	assert(logdata.channels['GPS']['NSats'].min() == 6)
	assert(logdata.channels['GPS']['NSats'].max() == 8)
	assert(logdata.channels['GPS']['HDop'].listData[0]   == (552, 4.68))
	assert(logdata.channels['GPS']['HDop'].listData[44]  == (768, 4.67))
	assert(logdata.channels['GPS']['HDop'].listData[157] == (1288, 2.28))
	assert(logdata.channels['CTUN']['ThrOut'].listData[5]   == (321, 139))
	assert(logdata.channels['CTUN']['ThrOut'].listData[45]  == (409, 242))
	assert(logdata.channels['CTUN']['ThrOut'].listData[125] == (589, 266))
	assert(logdata.channels['CTUN']['CRate'].listData[3]   == (317, 35))
	assert(logdata.channels['CTUN']['CRate'].listData[51]  == (421, 31))
	assert(logdata.channels['CTUN']['CRate'].listData[115] == (563, -8))
	assert(int(logdata.filesizeKB) == 307)
	assert(logdata.durationSecs    == 155)
	assert(logdata.lineCount       == 4750)

	# test LogIterator class
	lit = DataflashLog.LogIterator(logdata)
	assert(lit.currentLine == 0)
	assert(lit.iterators == {'CURR': (0, 310), 'ERR': (0, 307), 'NTUN': (0, 2206), 'CTUN': (0, 308), 'GPS': (0, 552), 'CMD': (0, 607), 'D32': (0, 305), 'ATT': (0, 311), 'EV': (0, 306), 'DU32': (0, 309), 'PM': (0, 479)})
	lit.jump(500)
	assert(lit.iterators == {'CURR': (9, 514), 'ERR': (1, 553), 'NTUN': (0, 2206), 'CTUN': (87, 500), 'GPS': (0, 552), 'CMD': (0, 607), 'D32': (0, 305), 'ATT': (83, 501), 'EV': (4, 606), 'DU32': (9, 513), 'PM': (1, 719)})
	assert(lit['CTUN']['ThrIn']   == 450)
	assert(lit['ATT']['RollIn']   == 11.19)
	assert(lit['CURR']['CurrTot'] == 25.827288)
	assert(lit['D32']['Value']    == 11122)
	next(lit)
	assert(lit.iterators == {'CURR': (9, 514), 'ERR': (1, 553), 'NTUN': (0, 2206), 'CTUN': (88, 502), 'GPS': (0, 552), 'CMD': (0, 607), 'D32': (0, 305), 'ATT': (83, 501), 'EV': (4, 606), 'DU32': (9, 513), 'PM': (1, 719)})
	lit.jump(4750)
	next(lit)
	assert(lit.currentLine == 4751)
	assert(lit['ATT']['Roll'] == 2.99)


	# TODO: unit test DataflashLog reading 2
	# ...

	# TODO: unit test the log test classes
	# ...


	print("All unit/regression tests GOOD\n")

except Exception as e:
	print("Error found: " + traceback.format_exc())
	print("UNIT TEST FAILED\n")
